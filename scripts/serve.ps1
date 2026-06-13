$ErrorActionPreference = "Stop"

$machinePath = [System.Environment]::GetEnvironmentVariable("Path", "Machine")
$userPath = [System.Environment]::GetEnvironmentVariable("Path", "User")
$env:Path = "$machinePath;$userPath"

$hugo = Get-Command hugo -ErrorAction SilentlyContinue
if (-not $hugo) {
    $hugoExe = Get-ChildItem -Path "$env:LOCALAPPDATA\Microsoft\WinGet\Packages" -Recurse -Filter hugo.exe -ErrorAction SilentlyContinue |
        Select-Object -First 1

    if (-not $hugoExe) {
        throw "Cannot find hugo.exe. Install Hugo Extended with: winget install --id Hugo.Hugo.Extended --exact"
    }

    $hugoPath = $hugoExe.FullName
} else {
    $hugoPath = $hugo.Source
}

& $hugoPath server --bind 127.0.0.1 --port 1313 --baseURL "http://127.0.0.1:1313/" --disableFastRender
