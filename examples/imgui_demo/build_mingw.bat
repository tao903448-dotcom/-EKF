@echo off
REM ImGui Demo Build Script for MinGW

set IMGUI_DIR=..\..\third_party\imgui
set SRC_DIR=..\..\src
set INC_DIR=..\..\include

REM 检查ImGui目录
if not exist "%IMGUI_DIR%\imgui.h" (
    echo ImGui not found at %IMGUI_DIR%
    echo Please download ImGui first.
    pause
    exit /b 1
)

echo ========================================
echo Building EKF ImGui Demo
echo ========================================

REM 编译ImGui源文件
echo.
echo [1/6] Compiling ImGui core...
g++ -c -std=c++11 -DIMGUI_IMPL_API="" %IMGUI_DIR%\imgui.cpp -I%IMGUI_DIR% -I%IMGUI_DIR%\backends -o imgui.o
if errorlevel 1 goto :error

g++ -c -std=c++11 -DIMGUI_IMPL_API="" %IMGUI_DIR%\imgui_draw.cpp -I%IMGUI_DIR% -I%IMGUI_DIR%\backends -o imgui_draw.o
if errorlevel 1 goto :error

g++ -c -std=c++11 -DIMGUI_IMPL_API="" %IMGUI_DIR%\imgui_tables.cpp -I%IMGUI_DIR% -I%IMGUI_DIR%\backends -o imgui_tables.o
if errorlevel 1 goto :error

g++ -c -std=c++11 -DIMGUI_IMPL_API="" %IMGUI_DIR%\imgui_widgets.cpp -I%IMGUI_DIR% -I%IMGUI_DIR%\backends -o imgui_widgets.o
if errorlevel 1 goto :error

g++ -c -std=c++11 -DIMGUI_IMPL_API="" %IMGUI_DIR%\imgui_demo.cpp -I%IMGUI_DIR% -I%IMGUI_DIR%\backends -o imgui_demo.o
if errorlevel 1 goto :error

echo [2/6] Compiling ImGui backends...
g++ -c -std=c++11 -DIMGUI_IMPL_API="" %IMGUI_DIR%\backends\imgui_impl_win32.cpp -I%IMGUI_DIR% -I%IMGUI_DIR%\backends -o imgui_impl_win32.o
if errorlevel 1 goto :error

g++ -c -std=c++11 -DIMGUI_IMPL_API="" %IMGUI_DIR%\backends\imgui_impl_dx11.cpp -I%IMGUI_DIR% -I%IMGUI_DIR%\backends -o imgui_impl_dx11.o
if errorlevel 1 goto :error

echo [3/6] Compiling EKF library...
g++ -c -std=c++11 %SRC_DIR%\matrix.c -I%INC_DIR% -o matrix.o
if errorlevel 1 goto :error

g++ -c -std=c++11 %SRC_DIR%\ekf.c -I%INC_DIR% -o ekf.o
if errorlevel 1 goto :error

echo [4/6] Compiling main program...
g++ -c -std=c++11 main.cpp -I%IMGUI_DIR% -I%IMGUI_DIR%\backends -I%INC_DIR% -o main.o
if errorlevel 1 goto :error

echo [5/6] Linking...
g++ -std=c++11 main.o imgui.o imgui_draw.o imgui_tables.o imgui_widgets.o imgui_demo.o imgui_impl_win32.o imgui_impl_dx11.o matrix.o ekf.o -ld3d11 -ldxgi -ld3dcompiler -lole32 -luuid -o ekf_imgui_demo.exe
if errorlevel 1 goto :error

echo [6/6] Cleaning up...
del *.o 2>nul

echo.
echo ========================================
echo Build successful!
echo Run: ekf_imgui_demo.exe
echo ========================================
pause
exit /b 0

:error
echo.
echo ========================================
echo Build failed!
echo ========================================
pause
exit /b 1
