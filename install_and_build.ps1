# PX4 依赖安装与 fmu-v5 编译（WSL 或 Docker）
# 在 PowerShell 中于本项目根目录执行: .\install_and_build.ps1

$ErrorActionPreference = "Stop"
$ProjectRoot = $PSScriptRoot
$LogFile = Join-Path $ProjectRoot "build_output.log"
"" | Out-File -FilePath $LogFile -Encoding utf8

function Log { param($msg) Add-Content -Path $LogFile -Value $msg -Encoding utf8; Write-Host $msg }

Log "=== PX4 安装依赖并编译 fmu-v5 ==="
Log "项目目录: $ProjectRoot"

# 优先尝试 Docker（与 CI 一致，无需本机依赖）
$dockerImage = "px4io/px4-dev-nuttx-focal:2022-08-12"
try {
    $null = docker images -q $dockerImage 2>$null
    if ($LASTEXITCODE -eq 0) {
        Log "使用 Docker 编译: $dockerImage"
        docker run --rm -v "${ProjectRoot}:/src" -w /src $dockerImage make px4_fmu-v5_default 2>&1 | Tee-Object -FilePath $LogFile -Append
        if ($LASTEXITCODE -eq 0) {
            Log "编译完成. 固件: build\px4_fmu-v5_default\px4_fmu-v5_default.px4"
            exit 0
        }
    }
} catch {}

# 其次尝试 WSL
$wslPath = (wslpath -u $ProjectRoot 2>$null)
if ($wslPath) {
    Log "使用 WSL 编译. 路径: $wslPath"
    $env:PATH = "/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$env:PATH"
    wsl -e bash -c "cd '$wslPath' && bash Tools/setup/ubuntu.sh --no-sim-tools && make px4_fmu-v5_default" 2>&1 | Tee-Object -FilePath $LogFile -Append
    if ($LASTEXITCODE -eq 0) {
        Log "编译完成. 固件: build\px4_fmu-v5_default\px4_fmu-v5_default.px4"
        exit 0
    }
}

Log "未检测到 Docker 或 WSL 可用. 请安装 WSL2+Ubuntu 或 Docker 后重试."
Log "日志已写入: $LogFile"
exit 1
