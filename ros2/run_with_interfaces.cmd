@echo off
setlocal
set "ROS_INTERFACE_SETUP=%~dp0install\local_setup.bat"
if defined MUJOCO_DEMO_INTERFACES_PREFIX set "ROS_INTERFACE_SETUP=%MUJOCO_DEMO_INTERFACES_PREFIX%\local_setup.bat"
if not exist "%ROS_INTERFACE_SETUP%" (
  echo Build the custom action first: pixi run --manifest-path ros2/pixi.toml build-interfaces 1>&2
  exit /b 1
)
call "%ROS_INTERFACE_SETUP%" >nul
if errorlevel 1 exit /b 1
%*
exit /b %errorlevel%
