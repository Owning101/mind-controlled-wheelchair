# Obsidian to Anki Sync

Purpose: keep the `DeuObsidian` Anki deck updated from the iCloud-synced
Obsidian note at:

```text
C:\Users\Admin\iCloudDrive\iCloud~md~obsidian\V1Deu\DeuVokab.md
```

Expected note format:

```text
gehen -> to go
Haus -> house
Blamieren (embarrass)
```

Run once:

```powershell
py -3.13 .\obsidian_anki_sync.py
```

Watch for iCloud/Obsidian changes:

```powershell
.\run_deu_obsidian_sync.ps1
```

Requirements:

- Anki Desktop is open.
- AnkiConnect is installed and reachable at `http://127.0.0.1:8765`.
- `OPENAI_API_KEY` is set in the environment.
- Optional: set `OPENAI_MODEL` to override the default model.

Useful checks:

```powershell
py -3.13 .\obsidian_anki_sync.py --dry-run
py -3.13 .\obsidian_anki_sync.py --limit 2
py -3.13 .\obsidian_anki_sync.py --watch --interval 30
```
