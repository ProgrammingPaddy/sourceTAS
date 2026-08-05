@echo off
rem Builds Injector.exe as a standalone x64 GUI app.
setlocal
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
cl /nologo /W3 /EHsc /O2 /MT /DUNICODE /D_UNICODE ^
   "%~dp0Injector.cpp" ^
   /Fe:"%~dp0Injector.exe" /Fo:"%~dp0Injector.obj" ^
   /link /SUBSYSTEM:WINDOWS
del "%~dp0Injector.obj" >nul 2>&1
endlocal
