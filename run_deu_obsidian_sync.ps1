$ErrorActionPreference = "Stop"

Set-Location -Path $PSScriptRoot

py -3.13 .\obsidian_anki_sync.py --watch --provider nvidia --deck ObsidianNotesAI --state output\obsidian_anki_sync_ai_state.json
