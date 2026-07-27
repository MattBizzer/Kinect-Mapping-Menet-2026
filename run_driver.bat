@echo off
setlocal

set "PROJECT_DIR=%~dp0"
set "VENV_ACTIVATE=%PROJECT_DIR%venv\Scripts\activate.bat"

if not exist "%VENV_ACTIVATE%" (
    echo Error: Virtual environment not found. Run setup first:
    echo   python -m venv venv
    echo   venv\Scripts\activate
    echo   pip install -r requirements-windows.txt
    exit /b 1
)

call "%VENV_ACTIVATE%"
python -u "%PROJECT_DIR%kinect_osc_driver.py" %*
