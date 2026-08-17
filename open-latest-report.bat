@echo off
rem Opens the newest solver search-space report in the default browser.
rem Reports are written to Output\reports (one per msolvegate run,
rem timestamped).
for /f "delims=" %%f in ('dir /b /o-d "%~dp0Output\reports\*.html" 2^>nul') do (
  start "" "%~dp0Output\reports\%%f"
  exit /b
)
echo No reports found in Output\reports - run msolvegate first.
pause
