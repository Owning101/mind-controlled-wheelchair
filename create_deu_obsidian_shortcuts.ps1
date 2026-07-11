$ErrorActionPreference = "Stop"

$desktop = [Environment]::GetFolderPath("Desktop")
$repo = $PSScriptRoot
$shell = New-Object -ComObject WScript.Shell

$shortcuts = @(
    @{
        Name = "Deu Obsidian Anki Smart.lnk"
        Script = "run_deu_obsidian_sync.ps1"
    },
    @{
        Name = "Deu Obsidian Anki No AI.lnk"
        Script = "run_deu_obsidian_sync_no_ai.ps1"
    }
)

foreach ($shortcutSpec in $shortcuts) {
    $shortcut = $shell.CreateShortcut((Join-Path $desktop $shortcutSpec.Name))
    $scriptPath = Join-Path $repo $shortcutSpec.Script
    $shortcut.TargetPath = "powershell.exe"
    $shortcut.Arguments = "-NoProfile -ExecutionPolicy Bypass -File `"$scriptPath`""
    $shortcut.WorkingDirectory = $repo
    $shortcut.IconLocation = "powershell.exe,0"
    $shortcut.Save()
}

Get-ChildItem $desktop -Filter "Deu Obsidian Anki*.lnk" | Select-Object Name, FullName
