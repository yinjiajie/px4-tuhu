@echo off
REM 在 WSL (Ubuntu 22.04) 中配置 PX4 编译环境并编译 fmu-v5
REM 运行后会在 WSL 中执行安装与编译，遇到 [sudo] 提示时请输入你的 WSL 用户密码
REM 项目在 D: 盘时 WSL 路径为 /mnt/d/FCS/PX4-Autopilot-1.15.4

wsl -u niujinyu -- bash -c "cd /mnt/d/FCS/PX4-Autopilot-1.15.4 && bash Tools/setup/install_and_build_fmuv5.sh"
pause
