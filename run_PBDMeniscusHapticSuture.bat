@echo off
set "ROOT=%~dp0"
set "PATH=%ROOT%install\bin;%PATH%"
cd /d "%ROOT%PBDMeniscusHapticSuture\build-clean\Release"
start "" "Example-PBDMeniscusHapticSuture.exe"
