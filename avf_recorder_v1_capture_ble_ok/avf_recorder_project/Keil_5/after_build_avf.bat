@echo off
setlocal

if "%1"=="" (
    echo Please input the app name!
    exit /b 1
)

if "%2"=="" (
    set "BIN_PATH=C:\Keil_v5\ARM\ARMCC\Bin"
) else (
    set "BIN_PATH=%2\ARM\ARMCC\Bin"
)

set "SDK_ROOT=..\..\GR551x_SDK_B6815"
set "SWD_DISENABLE=0"
set "ENCRY_ENABLE=1"
set "CONFIG=0X0016A286"
set "TOOL_PATH=%SDK_ROOT%\build\binaries\ble_tools\Keil"
set "FW_ENCRYPT_SIGN_TOOL_PATH=%SDK_ROOT%\build\binaries\encrypt_tools"
set "FW_KEY_SAVE_PATH=%FW_ENCRYPT_SIGN_TOOL_PATH%\encrypt_keys\encrypt_keys_example"
set "EFUSE_CONFIG_BIN=Efuse_SWD_Disable_%SWD_DISENABLE%_Encry_Mode_%ENCRY_ENABLE%_%CONFIG%"
set "TARGET=%1"
set "OBJ_DIR_PATH=.\Objects"
set "LISTING_DIR_PATH=.\Listings"
set "OUTPUT_DIR_PATH=.\build"
set "TARGET_APP=%TARGET%_fw"
set "TARGET_APP_ENCRY=%TARGET%_encrypt"

if not exist "%BIN_PATH%\fromelf.exe" (
    echo fromelf.exe not found: %BIN_PATH%\fromelf.exe
    exit /b 1
)
if not exist "%TOOL_PATH%\ble_tools.exe" (
    echo ble_tools.exe not found: %TOOL_PATH%\ble_tools.exe
    exit /b 1
)
if not exist "%FW_ENCRYPT_SIGN_TOOL_PATH%\generate_encrypt_bin.exe" (
    echo generate_encrypt_bin.exe not found: %FW_ENCRYPT_SIGN_TOOL_PATH%\generate_encrypt_bin.exe
    exit /b 1
)

if not exist "%OUTPUT_DIR_PATH%" (
    md "%OUTPUT_DIR_PATH%"
) else (
    del /q /s "%OUTPUT_DIR_PATH%\*" >nul 2>nul
)

"%BIN_PATH%\fromelf.exe" --bin --output "%OBJ_DIR_PATH%\%TARGET%.bin" "%OBJ_DIR_PATH%\%TARGET%.axf"
if errorlevel 1 exit /b 1

"%BIN_PATH%\fromelf.exe" --text -c --output "%OBJ_DIR_PATH%\%TARGET%.s" "%OBJ_DIR_PATH%\%TARGET%.axf"
if errorlevel 1 exit /b 1

copy "%OBJ_DIR_PATH%\%TARGET%.bin" "%OUTPUT_DIR_PATH%\%TARGET%.bin" >nul

"%TOOL_PATH%\ble_tools.exe" --mode=gen_fw --cfg="..\Src\config\custom_config.h" --bin="%OUTPUT_DIR_PATH%\%TARGET%.bin" --out_dir="%OUTPUT_DIR_PATH%" --app_name="%TARGET_APP%"
if errorlevel 1 exit /b 1

"%FW_ENCRYPT_SIGN_TOOL_PATH%\generate_encrypt_bin.exe" --operation encryptandsign --firmware_key "%FW_KEY_SAVE_PATH%\firmware.key" --signature_key "%FW_KEY_SAVE_PATH%\sign.key" --signature_pub_key "%FW_KEY_SAVE_PATH%\sign_pub.key" --rand_number "%FW_KEY_SAVE_PATH%\random.bin" --ori_firmware "%OUTPUT_DIR_PATH%\%TARGET_APP%.bin" --product_json_path "%FW_KEY_SAVE_PATH%\product.json" --output "%OUTPUT_DIR_PATH%\%TARGET_APP_ENCRY%.bin" >nul
if errorlevel 1 exit /b 1

"%TOOL_PATH%\ble_tools.exe" --mode=encrypt --cfg=%CONFIG% --keyfile="%FW_KEY_SAVE_PATH%\smt.bin" --swd=%SWD_DISENABLE% --enc=%ENCRY_ENABLE% --output="%OUTPUT_DIR_PATH%\%EFUSE_CONFIG_BIN%.bin" >nul
if errorlevel 1 exit /b 1

"%TOOL_PATH%\ble_tools.exe" --mode=bin2hex --cfg="..\Src\config\custom_config.h" --bin="%OUTPUT_DIR_PATH%\%TARGET_APP_ENCRY%.bin" --hex="%OUTPUT_DIR_PATH%\%TARGET_APP_ENCRY%.hex" >nul
if errorlevel 1 exit /b 1

copy "%OBJ_DIR_PATH%\%TARGET%.axf" "%OUTPUT_DIR_PATH%\%TARGET%.axf" >nul
copy "%LISTING_DIR_PATH%\%TARGET%.map" "%OUTPUT_DIR_PATH%\%TARGET%.map" >nul
copy "%OBJ_DIR_PATH%\%TARGET%.s" "%OUTPUT_DIR_PATH%\%TARGET%.s" >nul

if exist "%OUTPUT_DIR_PATH%\%TARGET_APP%.hex" ren "%OUTPUT_DIR_PATH%\%TARGET_APP%.hex" load_app.hex
if exist "%OUTPUT_DIR_PATH%\%TARGET_APP_ENCRY%.hex" ren "%OUTPUT_DIR_PATH%\%TARGET_APP_ENCRY%.hex" load_app_encrypt.hex

del /q /s "%OUTPUT_DIR_PATH%\*.tmp" >nul 2>nul
del /q /s "%OUTPUT_DIR_PATH%\info.*" >nul 2>nul
del /q /s "%OUTPUT_DIR_PATH%\header.bin" >nul 2>nul
del /q /s "%OUTPUT_DIR_PATH%\Efuse*" >nul 2>nul

echo Firmware has been successfully generated!
endlocal