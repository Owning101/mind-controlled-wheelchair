$ErrorActionPreference = "Stop"

$startup = [Environment]::GetFolderPath("Startup")
$repo = $PSScriptRoot
$scriptPath = Join-Path $repo "run_deu_obsidian_sync_no_ai.ps1"
$shell = New-Object -ComObject WScript.Shell

$shortcut = $shell.CreateShortcut((Join-Path $startup "Deu Obsidian Anki No AI.lnk"))
$shortcut.TargetPath = "powershell.exe"
$shortcut.Arguments = "-NoProfile -ExecutionPolicy Bypass -File `"$scriptPath`""
$shortcut.WorkingDirectory = $repo
$shortcut.IconLocation = "powershell.exe,0"
$shortcut.Save()

Get-Item (Join-Path $startup "Deu Obsidian Anki No AI.lnk") | Select-Object Name, FullName
