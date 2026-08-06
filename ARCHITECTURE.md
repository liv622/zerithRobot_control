# 通用机器人仿真框架架构

本项目采用根目录平级分层。`application`、`communication`、`trajectory` 等目录
与 `robots`、`robot_framework` 平级，不再放在某个具体机器人型号的软件包内。
E1PRO 只是当前已接入的一个机器人插件，不是框架名称。

## 目录职责

```text
app.py                       # 仿真薄启动器
teach_pendant.py             # 示教器薄启动器
entrypoints/                 # CLI 参数、机器人选择和应用装配
application/                 # 交互用例、连续点动、示教程序编排
communication/               # 仿真与示教器之间的 HTTP/JSON 适配器
domain/                      # 示教点等领域实体
infrastructure/              # JSON 等可替换的持久化实现
interfaces/
├── simulator/               # Viser 场景及控件绑定
└── pendant/                 # 模块化协作机器人示教器
    ├── template.py          # 页面结构
    └── assets/              # 示教器样式与交互脚本
trajectory/                  # MOVL/MOVJ 轨迹插补算法
robot_framework/             # 通用模型协议、控制器、IK 和插件协议
robots/
├── registry.py              # 可用机器人型号注册表
└── e1pro/                   # E1PRO 专属模型、校验和插件定义
tests/
```

## 依赖方向

```text
entrypoints
    ├── robots/registry
    └── interfaces / communication
                 ↓
             application
                 ↓
domain / trajectory / robot_framework

robots/<型号> ──实现──> robot_framework 的模型与插件协议
infrastructure ──实现──> application/ports
```

- `trajectory/` 只实现轨迹数据和插补算法，不依赖 UI、HTTP 或具体机器人。
- `application/` 编排机器人操作，不创建 Viser 控件或网络服务。
- `application/null_space_motion.py` 计算 7 轴零空间方向，并通过当前 IK
  后端锁定 TCP 六维位姿；安装 PyRoki 时由 PyRoki 约束优化执行。
- `communication/` 只通过状态读取和命令处理接口与应用交互。
- `interfaces/` 接收 `RobotPlugin`，不直接导入 `robots.e1pro`。
- `interfaces/simulator/` 是只读 Viser 场景适配器；它不创建 GUI 控件，所有
  运动命令和参数设置只从 Robot 示教器发起。
- 示教器页面、样式和交互脚本分离；业务规则仍由 `application/` 执行。
- `robots/<型号>/` 保存该型号的 URDF 约定、运动学模型和专属自检。

## 增加机器人型号

例如增加 `r6`：

1. 创建 `robots/r6/model.py`，实现
   `robot_framework.RobotModelProtocol` 所需字段和方法。
2. 创建 `robots/r6/plugin.py`，声明型号键、显示名称、URDF 路径、模型加载器和
   自检函数。
3. 在 `robots/registry.py` 注册插件。
4. 使用 `python app.py --robot r6` 启动。

通用界面、通信、轨迹和应用代码不应因增加型号而修改。若一个型号需要特殊界面，
应在其插件中增加能力描述，再由通用界面按能力显示，而不是写型号判断。

## 后续功能放置

- MOVC 等新轨迹：在 `trajectory/` 新建实现，在应用层增加编排。
- WebSocket、ROS 2 等协议：在 `communication/` 增加适配器。
- 新示教器或桌面 UI：在 `interfaces/` 增加适配器。
- SQLite 或远程点位存储：在 `infrastructure/` 实现应用仓储协议。
- 新机器人型号：只放在 `robots/<型号>/` 并注册插件。

架构测试会检查通用算法与应用层不能反向依赖 Viser、HTTP 或具体机器人型号。
