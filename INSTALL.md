# 安装与运行

## Linux / macOS

建议使用 Python 3.11：

```bash
cd e1pro
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r requirements.txt
python app.py --smoke-test
python teach_pendant.py --open
```

示教器脚本会自动启动 Viser 仿真、等待通信接口就绪，并打开 zerithRobot 页面。
默认 Viser 监听 `8080`，示教器监听 `8090`。

如果只需要单独运行三维仿真或执行无界面检查，才使用：

```bash
python app.py --robot e1pro --smoke-test
python app.py --host 127.0.0.1 --port 8080
```

## Windows PowerShell

```powershell
cd e1pro
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip setuptools wheel
python -m pip install -r requirements.txt
python app.py --smoke-test
python teach_pendant.py --open
```

如果 PowerShell 禁止激活脚本，可仅为当前窗口执行：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
```

## 局域网访问

如需让局域网设备访问示教器，可使用
`python teach_pendant.py --host 0.0.0.0 --open`，并在主机防火墙中允许 `8090`
端口。Viser 地址可以通过 `--viser-url` 指定。

项目必须保留 `e1_pro_full/urdf` 与 `e1_pro_full/meshes` 的相对目录结构，
否则 URDF 中的模型网格无法加载。
