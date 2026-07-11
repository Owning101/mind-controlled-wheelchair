#!/usr/bin/env python3
"""Sync German vocabulary from an Obsidian note into Anki.

Reads lines like "gehen -> to go" from an Obsidian markdown file, enriches
changed entries with OpenAI, and creates or updates Basic notes through
AnkiConnect.
"""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import os
import re
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Any


if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


DEFAULT_NOTE_PATH = Path(r"C:\Users\Admin\iCloudDrive\iCloud~md~obsidian\V1Deu\DeuVokab.md")
DEFAULT_DECK = "DeuObsidian"
DEFAULT_STATE = Path("output") / "obsidian_anki_sync_state.json"
DEFAULT_ANKI_URL = "http://127.0.0.1:8765"
DEFAULT_MODEL = "gpt-4.1-mini"
TAG = "obsidian_deu"
GENERATED_TAG = "auto_generated"


@dataclass(frozen=True)
class VocabEntry:
    german: str
    user_translation: str
    source_line: int

    @property
    def key(self) -> str:
        normalized = re.sub(r"\s+", " ", self.german.casefold()).strip()
        return hashlib.sha1(normalized.encode("utf-8")).hexdigest()[:16]

    @property
    def content_hash(self) -> str:
        raw = f"{self.german}\n{self.user_translation}"
        return hashlib.sha1(raw.encode("utf-8")).hexdigest()


def normalize_text(value: str) -> str:
    value = value.replace("\ufeff", "").replace("\u00a0", " ")
    value = value.replace("\u2013", "-").replace("\u2014", "-")
    return re.sub(r"\s+", " ", value).strip()


def parse_vocab_note(path: Path) -> list[VocabEntry]:
    text = path.read_text(encoding="utf-8-sig")
    entries: list[VocabEntry] = []
    seen: set[str] = set()

    for line_number, raw_line in enumerate(text.splitlines(), start=1):
        line = normalize_text(raw_line)
        if not line or line.startswith("#") or line.startswith("- ["):
            continue

        german = ""
        translation = ""
        if "->" in line:
            german, translation = line.split("->", 1)
        else:
            match = re.match(r"^(?P<german>.+?)\s*\((?P<translation>.+)\)\s*$", line)
            if match:
                german = match.group("german")
                translation = match.group("translation")
            else:
                german = line

        entry = VocabEntry(
            german=normalize_text(german).strip(" -"),
            user_translation=normalize_text(translation).strip(" -"),
            source_line=line_number,
        )
        if not entry.german:
            continue
        if entry.key in seen:
            print(f"Skipping duplicate word on line {line_number}: {entry.german}")
            continue
        seen.add(entry.key)
        entries.append(entry)

    return entries


def load_state(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"version": 1, "entries": {}}
    return json.loads(path.read_text(encoding="utf-8"))


def save_state(path: Path, state: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_text(json.dumps(state, indent=2, ensure_ascii=False), encoding="utf-8")
    tmp_path.replace(path)


def post_json(url: str, payload: dict[str, Any], headers: dict[str, str] | None = None, timeout: int = 60) -> dict[str, Any]:
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=body,
        headers={"Content-Type": "application/json", **(headers or {})},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code} from {url}: {detail}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Could not connect to {url}: {exc.reason}") from exc


def anki(action: str, params: dict[str, Any] | None = None, *, anki_url: str = DEFAULT_ANKI_URL) -> Any:
    response = post_json(anki_url, {"action": action, "version": 6, "params": params or {}}, timeout=30)
    if response.get("error"):
        raise RuntimeError(f"AnkiConnect {action} failed: {response['error']}")
    return response.get("result")


def ensure_deck(deck: str, anki_url: str) -> None:
    anki("createDeck", {"deck": deck}, anki_url=anki_url)


def find_existing_notes(deck: str, anki_url: str) -> dict[str, int]:
    query = f'deck:"{deck}" tag:{TAG}'
    note_ids = anki("findNotes", {"query": query}, anki_url=anki_url)
    if not note_ids:
        return {}

    notes = anki("notesInfo", {"notes": note_ids}, anki_url=anki_url)
    existing: dict[str, int] = {}
    for note in notes:
        front = note.get("fields", {}).get("Front", {}).get("value", "")
        text_front = re.sub(r"<[^>]+>", "", html.unescape(front)).strip()
        key = hashlib.sha1(re.sub(r"\s+", " ", text_front.casefold()).encode("utf-8")).hexdigest()[:16]
        existing[key] = int(note["noteId"])
    return existing


def extract_response_text(response: dict[str, Any]) -> str:
    if isinstance(response.get("output_text"), str):
        return response["output_text"]
    pieces: list[str] = []
    for item in response.get("output", []):
        for content in item.get("content", []):
            if isinstance(content.get("text"), str):
                pieces.append(content["text"])
    return "\n".join(pieces).strip()


def parse_json_object(text: str) -> dict[str, Any]:
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        match = re.search(r"\{.*\}", text, flags=re.DOTALL)
        if not match:
            raise
        return json.loads(match.group(0))


def enrich_entry(entry: VocabEntry, model: str) -> dict[str, Any]:
    api_key = os.environ.get("OPENAI_API_KEY", "").strip()
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY is not set. Set it before running smart sync.")

    prompt = {
        "task": "Create Anki back-side material for a German vocabulary card.",
        "requirements": [
            "Return only valid JSON.",
            "Keep the English translation concise.",
            "Correct the user's translation if it is wrong or incomplete.",
            "Give one or two natural German example sentences.",
            "Use beginner/intermediate friendly examples unless the word is slang or regional.",
        ],
        "schema": {
            "ai_translation": "English translation or short explanation",
            "translation_note": "Short correction/note, or empty string if user's translation is fine",
            "examples": ["German sentence - English meaning"],
        },
        "german": entry.german,
        "user_translation": entry.user_translation,
    }
    payload = {
        "model": model,
        "input": (
            "You are a careful German-English vocabulary tutor. "
            "Output only JSON with keys ai_translation, translation_note, examples.\n\n"
            f"{json.dumps(prompt, ensure_ascii=False)}"
        ),
    }
    response = post_json(
        "https://api.openai.com/v1/responses",
        payload,
        headers={"Authorization": f"Bearer {api_key}"},
        timeout=90,
    )
    parsed = parse_json_object(extract_response_text(response))
    examples = parsed.get("examples") or []
    if isinstance(examples, str):
        examples = [examples]
    return {
        "ai_translation": normalize_text(str(parsed.get("ai_translation", ""))),
        "translation_note": normalize_text(str(parsed.get("translation_note", ""))),
        "examples": [normalize_text(str(example)) for example in examples[:2] if normalize_text(str(example))],
    }


def build_front(entry: VocabEntry) -> str:
    return f"<h2>{html.escape(entry.german)}</h2>"


def build_back(entry: VocabEntry, enrichment: dict[str, Any]) -> str:
    user_translation = html.escape(entry.user_translation or "(no user translation)")
    ai_translation = html.escape(enrichment.get("ai_translation") or "(no AI translation)")
    note = html.escape(enrichment.get("translation_note") or "")
    examples = enrichment.get("examples") or []

    parts = [
        "<section>",
        "<h3>Your translation</h3>",
        f"<p>{user_translation}</p>",
        "<h3>AI translation/check</h3>",
        f"<p>{ai_translation}</p>",
    ]
    if note:
        parts.append(f"<p><em>{note}</em></p>")
    if examples:
        parts.extend(["<h3>Examples</h3>", "<ul>"])
        for example in examples:
            parts.append(f"<li>{html.escape(str(example))}</li>")
        parts.append("</ul>")
    parts.append("</section>")
    return "\n".join(parts)


def add_or_update_note(entry: VocabEntry, enrichment: dict[str, Any], note_id: int | None, deck: str, anki_url: str) -> int:
    fields = {"Front": build_front(entry), "Back": build_back(entry, enrichment)}
    if note_id:
        anki("updateNoteFields", {"note": {"id": note_id, "fields": fields}}, anki_url=anki_url)
        anki("addTags", {"notes": [note_id], "tags": f"{TAG} {GENERATED_TAG}"}, anki_url=anki_url)
        return note_id

    result = anki(
        "addNote",
        {
            "note": {
                "deckName": deck,
                "modelName": "Basic",
                "fields": fields,
                "options": {"allowDuplicate": False, "duplicateScope": "deck"},
                "tags": [TAG, GENERATED_TAG],
            }
        },
        anki_url=anki_url,
    )
    if not result:
        raise RuntimeError(f"Anki refused to add note for {entry.german!r}. It may already exist.")
    return int(result)


def sync_once(args: argparse.Namespace) -> int:
    note_path = Path(args.note_path)
    state_path = Path(args.state)
    entries = parse_vocab_note(note_path)
    state = load_state(state_path)
    state_entries: dict[str, Any] = state.setdefault("entries", {})

    if args.dry_run:
        print(f"Parsed {len(entries)} entries from {note_path}")
        for entry in entries[: args.limit or len(entries)]:
            print(f"line {entry.source_line}: {entry.german} -> {entry.user_translation}")
        return 0

    if not args.no_ai and not os.environ.get("OPENAI_API_KEY"):
        raise RuntimeError("OPENAI_API_KEY is missing. Use --no-ai for a dry/simple sync or set the env var.")

    ensure_deck(args.deck, args.anki_url)
    discovered_notes = find_existing_notes(args.deck, args.anki_url)

    added = updated = skipped = 0
    limit = args.limit or len(entries)
    for entry in entries[:limit]:
        previous = state_entries.get(entry.key, {})
        note_id = previous.get("note_id") or discovered_notes.get(entry.key)
        changed = previous.get("content_hash") != entry.content_hash
        previous_enrichment = previous.get("enrichment") or {}
        needs_ai_enrichment = (
            not args.no_ai
            and (
                not previous.get("ai_enriched")
                or not previous_enrichment.get("ai_translation")
            )
        )

        if not args.force and note_id and not changed and not needs_ai_enrichment:
            skipped += 1
            continue

        if args.no_ai:
            enrichment = previous_enrichment or {
                "ai_translation": "",
                "translation_note": "AI enrichment was skipped for this run.",
                "examples": [],
            }
            ai_enriched = False
        elif not args.force and not changed and previous_enrichment.get("ai_translation"):
            enrichment = previous_enrichment
            ai_enriched = bool(previous.get("ai_enriched"))
        else:
            enrichment = enrich_entry(entry, args.model)
            ai_enriched = True

        saved_note_id = add_or_update_note(entry, enrichment, int(note_id) if note_id else None, args.deck, args.anki_url)
        state_entries[entry.key] = {
            "note_id": saved_note_id,
            "german": entry.german,
            "user_translation": entry.user_translation,
            "content_hash": entry.content_hash,
            "enrichment": enrichment,
            "ai_enriched": ai_enriched,
            "updated_at": int(time.time()),
        }
        save_state(state_path, state)
        if note_id:
            updated += 1
        else:
            added += 1
        print(f"{'Updated' if note_id else 'Added'}: {entry.german}")
        time.sleep(args.pause)

    print(f"Done. Added {added}, updated {updated}, skipped {skipped}.")
    return 0


def watch(args: argparse.Namespace) -> int:
    note_path = Path(args.note_path)
    last_signature: tuple[int, int] | None = None
    print(f"Watching {note_path}. Press Ctrl+C to stop.")
    while True:
        try:
            stat = note_path.stat()
            signature = (int(stat.st_mtime), stat.st_size)
            if signature != last_signature:
                last_signature = signature
                sync_once(args)
            time.sleep(args.interval)
        except KeyboardInterrupt:
            print("Stopped.")
            return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Sync Obsidian German vocab notes to Anki.")
    parser.add_argument("--note-path", default=str(DEFAULT_NOTE_PATH))
    parser.add_argument("--deck", default=DEFAULT_DECK)
    parser.add_argument("--state", default=str(DEFAULT_STATE))
    parser.add_argument("--anki-url", default=DEFAULT_ANKI_URL)
    parser.add_argument("--model", default=os.environ.get("OPENAI_MODEL", DEFAULT_MODEL))
    parser.add_argument("--watch", action="store_true", help="Keep checking the note for changes.")
    parser.add_argument("--interval", type=int, default=30, help="Seconds between watch checks.")
    parser.add_argument("--pause", type=float, default=0.2, help="Pause between Anki writes.")
    parser.add_argument("--limit", type=int, help="Only process the first N parsed entries.")
    parser.add_argument("--force", action="store_true", help="Regenerate and update all processed notes.")
    parser.add_argument("--dry-run", action="store_true", help="Parse and print entries without calling Anki/OpenAI.")
    parser.add_argument("--no-ai", action="store_true", help="Sync without calling OpenAI.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.watch:
            return watch(args)
        return sync_once(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
