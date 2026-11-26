@echo off
setlocal
cd /d "%~dp0"

set "PY_CMD="
where python >nul 2>nul && set "PY_CMD=python"
if not defined PY_CMD (
    where py >nul 2>nul && set "PY_CMD=py -3"
)
if not defined PY_CMD (
    if exist "C:\ncs\toolchains\b620d30767\opt\bin\python.exe" (
        set "PY_CMD=C:\ncs\toolchains\b620d30767\opt\bin\python.exe"
    )
)

if not defined PY_CMD (
    echo Erreur : Python introuvable. Lancez un serveur HTTP local sur ce dossier puis ouvrez stm32_ble_ota.html.
    pause
    exit /b 1
)

echo Ouverture de l'interface OTA BLE sur http://127.0.0.1:8090/stm32_ble_ota.html ...
start "" "http://127.0.0.1:8090/stm32_ble_ota.html"
%PY_CMD% -m http.server 8090 --bind 127.0.0.1
