# 在 WSL (Ubuntu 22.04) 下配置并编译 PX4 fmu-v5

你的 WSL 用户：**niujinyu**。安装依赖时会要求输入该用户的密码（不会保存到任何文件）。

---

## 方法一：在 WSL 终端里执行（推荐）

1. **打开 WSL**  
   - 在 Windows 开始菜单搜索并打开 **“Ubuntu 22.04”** 或 **“Ubuntu”**，或  
   - 在 CMD/PowerShell 里输入：`wsl`

2. **进入项目目录**（项目在 D 盘时）  
   ```bash
   cd /mnt/d/FCS/PX4-Autopilot-1.15.4
   ```

3. **一键安装依赖并编译**  
   ```bash
   bash Tools/setup/install_and_build_fmuv5.sh
   ```  
   - 出现 `[sudo] password for niujinyu:` 时，输入你的 WSL 用户密码后回车。  
   - 等待依赖安装和编译完成。

4. **编译完成后**，固件位置：  
   - `build/px4_fmu-v5_default/px4_fmu-v5_default.px4`  
   - `build/px4_fmu-v5_default/px4_fmu-v5_default.elf`

---

## 方法二：从 Windows 双击运行

- 双击项目根目录下的 **`wsl_setup_and_build.bat`**。  
- 会打开 WSL 并自动执行上述安装与编译；若出现密码提示，在**同一窗口**输入 WSL 用户密码。  
- 若没有弹出可输入密码的窗口，请改用**方法一**，在 WSL 里手动执行命令。

---

## 若项目不在 D 盘

- 在 WSL 里，C 盘为 `/mnt/c/`，D 盘为 `/mnt/d/`，以此类推。  
- 例如项目在 `C:\FCS\PX4-Autopilot-1.15.4`，则先执行：  
  ```bash
  cd /mnt/c/FCS/PX4-Autopilot-1.15.4
  ```  
  再执行：  
  ```bash
  bash Tools/setup/install_and_build_fmuv5.sh
  ```

---

## 以后只编译（依赖已装好）

在 WSL 里进入项目目录后执行：

```bash
export PATH=/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH
make px4_fmu-v5_default
```

可选：多核加速 `make px4_fmu-v5_default -j8`。

---

## 子模块与编译失败排查

**.gitmodules** 中：**NuttX** 与 **NuttX-apps** 使用 **GitHub**（保证分支与内容完整），其余使用 Gitee 等镜像。

若 **`git submodule sync --recursive` 和 `git submodule update --init --recursive` 没有效果、不下载依赖**，或编译报 **pathspec did not match**、**No rule to make target 'olddefconfig'**，请改用「直接 clone」脚本（不依赖 git submodule 命令）：

```bash
bash Tools/setup/clone_submodules_standalone.sh
```

该脚本会按 **.gitmodules** 里的 path/url/branch 逐个执行 **git clone**，缺失或空目录会重新拉取。

也可先尝试强制重新初始化（仍用 git submodule）：

```bash
bash Tools/setup/submodule_init_gitee.sh
```

若 GitHub 访问很慢，可只拉 NuttX 两个：

```bash
bash Tools/setup/submodule_fallback_nuttx_github.sh
```

然后执行：

```bash
export PATH="/opt/gcc-arm-none-eabi-9-2020-q2-update/bin:$PATH"
make px4_fmu-v5_default
```
