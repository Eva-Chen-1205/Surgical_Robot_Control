@echo off
echo Compiling Kinematics Tool...
g++ -O3 kinematics_tool.cpp -o kinematics_tool.exe
if %ERRORLEVEL% EQU 0 (
    echo.
    echo Compilation Successful!
    echo Run 'kinematics_tool.exe' to start.
    echo.
) else (
    echo.
    echo Compilation Failed.
    echo.
)
pause
