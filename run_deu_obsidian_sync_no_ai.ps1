$ErrorActionPreference = "Stop"

Set-Location -Path $PSScriptRoot

py -3.13 .\obsidian_anki_sync.py --watch --no-ai
