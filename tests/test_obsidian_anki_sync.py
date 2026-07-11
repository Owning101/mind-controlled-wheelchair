import contextlib
import io
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import obsidian_anki_sync as sync


class ObsidianAnkiSyncTests(unittest.TestCase):
    def test_parse_arrow_and_parentheses_entries(self):
        with tempfile.TemporaryDirectory() as tmp:
            note = Path(tmp) / "vocab.md"
            note.write_text(
                "gehen -> to go\n\nBlamieren (embarrass)\n# heading\n",
                encoding="utf-8",
            )

            entries = sync.parse_vocab_note(note)

        self.assertEqual([entry.german for entry in entries], ["gehen", "Blamieren"])
        self.assertEqual([entry.user_translation for entry in entries], ["to go", "embarrass"])

    def test_build_back_includes_user_ai_and_examples(self):
        entry = sync.VocabEntry("gehen", "to go", 1)
        back = sync.build_back(
            entry,
            {
                "ai_translation": "to go; to walk",
                "translation_note": "The meaning depends on context.",
                "examples": ["Ich gehe nach Hause. - I am going home."],
            },
        )

        self.assertIn("Your translation", back)
        self.assertIn("to go", back)
        self.assertIn("to go; to walk", back)
        self.assertIn("Ich gehe nach Hause.", back)

    def test_extract_nvidia_response_text(self):
        text = sync.extract_nvidia_response_text(
            {
                "choices": [
                    {
                        "message": {
                            "content": '{"ai_translation":"suspicious","translation_note":"","examples":[]}'
                        }
                    }
                ]
            }
        )

        self.assertIn("suspicious", text)

    def test_resolve_model_uses_provider_specific_env(self):
        with mock.patch.dict("os.environ", {"NVIDIA_MODEL": "nvidia/test", "OPENAI_MODEL": "openai-test"}):
            self.assertEqual(sync.resolve_model("nvidia", None), "nvidia/test")
            self.assertEqual(sync.resolve_model("openai", None), "openai-test")
            self.assertEqual(sync.resolve_model("nvidia", "manual-model"), "manual-model")

    def test_enrich_entry_nvidia_uses_nvidia_api_shape(self):
        captured = {}

        def fake_post_json(url, payload, headers=None, timeout=60):
            captured["url"] = url
            captured["payload"] = payload
            captured["headers"] = headers
            return {
                "choices": [
                    {
                        "message": {
                            "content": (
                                '{"ai_translation":"suspicious","translation_note":"",'
                                '"examples":["Das ist verdächtig. - That is suspicious."]}'
                            )
                        }
                    }
                ]
            }

        args = sync.build_parser().parse_args(["--provider", "nvidia", "--model", "nvidia/test"])
        entry = sync.VocabEntry("verdächtig", "sus", 1)
        with mock.patch.dict("os.environ", {"NVIDIA_API_KEY": "test-key"}):
            with mock.patch.object(sync, "post_json", side_effect=fake_post_json):
                enrichment = sync.enrich_entry(entry, args)

        self.assertEqual(captured["url"], sync.DEFAULT_NVIDIA_URL)
        self.assertEqual(captured["payload"]["model"], "nvidia/test")
        self.assertIn("messages", captured["payload"])
        self.assertEqual(captured["headers"]["Authorization"], "Bearer test-key")
        self.assertEqual(enrichment["ai_translation"], "suspicious")

    def test_enrich_entry_openai_uses_openai_response_shape(self):
        captured = {}

        def fake_post_json(url, payload, headers=None, timeout=60):
            captured["url"] = url
            captured["payload"] = payload
            captured["headers"] = headers
            return {
                "output_text": (
                    '{"ai_translation":"suspicious","translation_note":"",'
                    '"examples":["Das ist verdächtig. - That is suspicious."]}'
                )
            }

        args = sync.build_parser().parse_args(["--provider", "openai", "--model", "openai-test"])
        entry = sync.VocabEntry("verdächtig", "sus", 1)
        with mock.patch.dict("os.environ", {"OPENAI_API_KEY": "test-key"}):
            with mock.patch.object(sync, "post_json", side_effect=fake_post_json):
                enrichment = sync.enrich_entry(entry, args)

        self.assertEqual(captured["url"], sync.DEFAULT_OPENAI_URL)
        self.assertEqual(captured["payload"]["model"], "openai-test")
        self.assertIn("input", captured["payload"])
        self.assertEqual(captured["headers"]["Authorization"], "Bearer test-key")
        self.assertEqual(enrichment["ai_translation"], "suspicious")

    def test_missing_nvidia_key_names_nvidia_env_var(self):
        args = sync.build_parser().parse_args(["--provider", "nvidia"])
        entry = sync.VocabEntry("gehen", "to go", 1)

        with mock.patch.dict("os.environ", {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "NVIDIA_API_KEY"):
                sync.enrich_entry(entry, args)

    def test_missing_openai_key_names_openai_env_var(self):
        args = sync.build_parser().parse_args(["--provider", "openai"])
        entry = sync.VocabEntry("gehen", "to go", 1)

        with mock.patch.dict("os.environ", {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "OPENAI_API_KEY"):
                sync.enrich_entry(entry, args)

    def test_parser_rejects_unknown_provider(self):
        parser = sync.build_parser()

        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                parser.parse_args(["--provider", "unknown"])


if __name__ == "__main__":
    unittest.main()
