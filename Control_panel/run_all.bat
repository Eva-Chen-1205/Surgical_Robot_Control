@echo off
echo.
echo ==============================================
echo Compiling C++ Programs
echo ==============================================
g++ ik_server.cpp -o ik_server.exe -lws2_32
if %errorlevel% neq 0 (
    echo [ERROR] ik_server compilation failed!
    pause
    exit /b %errorlevel%
)
echo [OK] ik_server.exe compiled successfully.

g++ fk_test.cpp -o fk_test.exe
if %errorlevel% neq 0 (
    echo [ERROR] fk_test compilation failed!
    pause
    exit /b %errorlevel%
)
echo [OK] fk_test.exe compiled successfully.

echo.
echo ==============================================
echo Starting Execution
echo ==============================================
echo Starting IK Server in a new window...
start "IK Server" cmd /k "ik_server.exe"

echo Waiting for IK Server to initialize the trajectory file...
timeout /t 2 /nobreak > nul

echo Starting FK Test in a new window...
start "FK Test" cmd /k "fk_test.exe"

echo Starting Bridge Server (Unity relay) in a new window...
start "Bridge Server" cmd /k "python bridge_server.py"

echo.
echo ==============================================
echo All programs are now running!
echo ==============================================
echo.
echo   [IK Server]     Port 8080 (browser <-> IK)
echo   [FK Test]        Reading trajectory file
echo   [Bridge Server]  Port 9090 (browser -> Unity)
echo.
echo   Next steps:
echo   1. Open index.html in your browser
echo   2. Enter the Bridge Server URL in the bottom bar
echo   3. Tell your classmate to connect Unity to ws://YOUR_IP:9090/unity
echo.
pause
