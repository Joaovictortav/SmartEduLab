@echo off

:: Obter o diretório atual
set "current_dir=%cd%"

:: Iniciar o primeiro processo em um novo shell
start cmd /k "cd /d %current_dir%\BackFan-Plate\BackFan-Plate && dotnet run BackFan-Plate.cprojs"

:: Iniciar o segundo processo em um novo shell
start chrome.exe %current_dir%\monitor.html