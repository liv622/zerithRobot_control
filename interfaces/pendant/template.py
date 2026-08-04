"""Shell document for the modular collaborative-robot teach pendant."""

PENDANT_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>zerithRobot</title>
  <link rel="stylesheet" href="/assets/pendant.css">
</head>
<body>
<div class="screen">
  <header class="topbar">
    <div class="brand"><b>zerithRobot</b><span>协作机器人示教器</span></div>
    <div class="top-status">
      <span class="chip"><i id="lamp" class="lamp"></i><b id="connection">仿真离线</b></span>
      <span class="chip">手动 · 仿真</span>
      <span class="chip">TCP <b id="reach">--</b></span>
      <span class="chip">速度 <b id="speedBadge">30%</b></span>
    </div>
    <div class="clock"><span id="backend">IK --</span><span id="clock"></span></div>
  </header>

  <div class="workspace">
    <nav class="sidebar">
      <button class="nav active" data-page="home"><span>⌂</span>首页</button>
      <button class="nav" data-page="move"><span>✥</span>运动</button>
      <button class="nav" data-page="program"><span>▶</span>示教程序</button>
      <button class="nav" data-page="config"><span>⚙</span>配置</button>
      <button class="nav" data-page="diagnostics"><span>◉</span>诊断</button>
    </nav>

    <div class="content-stage">
      <main class="pages">
      <section id="page-home" class="page active">
        <div class="page-head"><div><h1>首页</h1><p>机器人状态和常用操作</p></div><span id="homeStatus" class="state-tag">待机</span></div>
        <div class="dashboard">
          <article class="panel hero" data-submodule="home-overview">
            <div class="panel-title">机器人概览 <span>当前 TCP / 关节状态</span></div>
            <div class="pose-overview">
              <div><small>X</small><b id="hx">--</b><em>m</em></div>
              <div><small>Y</small><b id="hy">--</b><em>m</em></div>
              <div><small>Z</small><b id="hz">--</b><em>m</em></div>
              <div><small>Rx</small><b id="hrx">--</b><em>deg</em></div>
              <div><small>Ry</small><b id="hry">--</b><em>deg</em></div>
              <div><small>Rz</small><b id="hrz">--</b><em>deg</em></div>
            </div>
            <div id="homeJoints" class="joint-bars"></div>
          </article>
          <article class="panel" data-submodule="home-quick">
            <div class="panel-title">快速操作 <span>常用任务</span></div>
            <div class="quick-grid">
              <button onclick="showPage('move')"><b>运动示教</b><small>笛卡尔 / 关节 / 附加轴</small></button>
              <button onclick="showPage('program')"><b>示教程序</b><small>MOVL / MOVJ 点位程序</small></button>
              <button onclick="cmd('target_current')"><b>读取当前 TCP</b><small>目标同步至实际位姿</small></button>
              <button class="warn" onclick="cmd('reset')"><b>机器人复位</b><small>恢复初始关节构型</small></button>
            </div>
          </article>
          <article class="panel" data-submodule="home-runtime">
            <div class="panel-title">运行参数 <span id="activeProfileHome">未调用配置</span></div>
            <div class="metric-grid">
              <div class="metric"><b id="homeSpeed">--</b><span>速度百分比</span></div>
              <div class="metric"><b id="homeLinear">--</b><span>最大线速度</span></div>
              <div class="metric"><b id="homeJointSpeed">--</b><span>最大关节速度</span></div>
              <div class="metric"><b id="homeDelay">--</b><span>动作间延时</span></div>
            </div>
          </article>
          <article class="panel hardware-panel" data-submodule="home-hardware">
            <div class="panel-title">E1Pro初始化控制 <span id="hardwareState">右臂未连接</span></div>
            <div class="form-row"><label>控制柜 IP</label><input id="hardwareIp" value="192.168.1.190" inputmode="decimal"></div>
            <div class="action-row two"><button class="primary" onclick="connectHardware()">连接右臂</button><button onclick="cmd('disconnect_hardware')">断开</button></div>
            <div class="action-row two"><button class="run" onclick="cmd('enable_hardware')">启用 PD 前馈</button><button class="danger" onclick="cmd('disable_hardware')">右臂下使能</button></div>
            <div class="action-row two"><button class="warn" onclick="cmd('release_hardware_brake')">右臂松闸</button><button class="danger" onclick="cmd('apply_hardware_brake')">右臂抱闸</button></div>
            <p class="hint">仅控制右臂 B。使能将配置关节阻抗及与插补频率匹配的 PD 前馈；松闸前请确认机械臂已被可靠支撑。</p>
          </article>
        </div>
      </section>

      <section id="page-move" class="page">
        <div class="page-head"><div><h1>运动示教</h1><p>长按方向键连续运动，松开立即停止</p></div>
          <div class="segmented move-tabs">
            <button class="active" data-mode="cartesian">笛卡尔</button>
            <button data-mode="joint">关节</button>
            <button data-mode="nullspace">零空间</button>
            <button data-mode="auxiliary">附加轴</button>
          </div>
        </div>
        <div class="move-workspace">
          <div class="move-operation">
        <div id="move-cartesian" class="move-mode active">
          <article class="panel grow" data-submodule="move-motion">
            <div class="panel-title">笛卡尔运动 <span>参考坐标系：BASE　活动工具：TCP</span></div>
            <div id="targetFields" class="target-fields"></div>
            <div id="cartJog" class="cart-jog"></div>
            <div class="action-row">
              <button class="primary" onclick="applyTarget()">写入目标</button>
              <button onclick="cmd('target_current')">读取当前 TCP</button>
              <button onclick="cmd('solve')">执行 IK</button>
              <button onclick="cmd('recover')">多起点求解</button>
            </div>
            <section class="inline-settings" data-submodule="move-settings">
              <div class="inline-settings-title">运动选项 <span>Move</span></div>
              <div class="inline-settings-grid">
                <label class="switch-row">实时 IK<input id="live" type="checkbox"><i></i></label>
                <label class="switch-row">锁定 TCP 姿态<input id="orient" type="checkbox"><i></i></label>
                <label class="switch-row">不可达自动恢复<input id="recovery" type="checkbox"><i></i></label>
                <label class="switch-row">场景 TCP 拖拽<input id="dragToggle" type="checkbox" onchange="setDrag()"><i></i></label>
                <div class="step-box"><span>步进值 / 连续速度档</span><b><input id="cartStep" type="number" value="10" min=".1" max="30" step=".1"> mm / deg</b><label class="step-mode-toggle">步进点动<input id="cartStepMode" type="checkbox"><i></i></label></div>
                <button onclick="applySolverSettings()">应用运动选项</button>
              </div>
            </section>
          </article>
        </div>
        <div id="move-joint" class="move-mode">
          <article class="panel grow" data-submodule="move-motion"><div class="panel-title">关节运动 <span>J1—J7 · deg</span></div><div id="jointList" class="joint-list"></div>
          <section class="inline-settings arm-shape-settings" data-submodule="move-settings">
            <div class="inline-settings-title">同 TCP 臂型 <span>冗余机器人</span></div>
            <p class="hint">保持当前 TCP 位置和姿态不变，搜索另一组关节解。</p>
            <div class="inline-settings-grid compact-actions"><button class="shape-button" onclick="cmd('switch_arm_shape',{direction:-1})">‹ 上一个臂型</button><button class="shape-button primary" onclick="cmd('switch_arm_shape',{direction:1})">下一个臂型 ›</button><button onclick="cmd('guide_current')">当前臂型设为参考</button><label class="switch-row">启用臂型引导<input id="guideOn" type="checkbox"><i></i></label><div class="form-row"><label>引导强度</label><input id="strength" type="number" min="0" max=".5" step=".01"></div><div class="step-box"><span>步进值 / 连续速度档</span><b><input id="jointStep" type="number" value="5" min=".1" max="30" step=".1"> deg</b><label class="step-mode-toggle">步进点动<input id="jointStepMode" type="checkbox"><i></i></label></div><button onclick="applySolverSettings()">应用臂型参数</button></div>
          </section></article>
        </div>
        <div id="move-nullspace" class="move-mode">
          <article class="panel grow" data-submodule="move-motion">
            <div class="panel-title">零空间关节点动 <span>TCP 位置与姿态硬约束 · PyRoki</span></div>
            <div class="null-lock">
              <div><small>锁定对象</small><b>当前实际 TCP</b></div>
              <div><small>位置约束</small><b>XYZ 保持不动</b></div>
              <div><small>姿态约束</small><b>Rx/Ry/Rz 保持不动</b></div>
            </div>
            <div class="null-controls">
              <button onpointerdown="startHold(event,'nullspace',0,-1)" onpointerup="endHold(event)" onpointerleave="endHold(event)" onpointercancel="endHold(event)">− 零空间负方向</button>
              <div><small>冗余自由度</small><b>φ</b><span>1 DOF</span></div>
              <button class="primary" onpointerdown="startHold(event,'nullspace',0,1)" onpointerup="endHold(event)" onpointerleave="endHold(event)" onpointercancel="endHold(event)">零空间正方向 ＋</button>
            </div>
            <div id="nullJointList" class="joint-list null-list"></div>
            <section class="inline-settings null-settings" data-submodule="move-settings">
            <div class="inline-settings-title">零空间状态 <span>7-DOF redundancy</span></div>
            <div class="null-state"><i id="nullLamp"></i><b id="nullState">等待点动</b></div>
            <p class="hint">长按零空间负方向或正方向。系统先锁定当前 TCP，再沿 7 轴机械臂的零空间方向连续改变关节值。</p>
            <p class="hint">7 轴关节会联动变化。一般构型下固定 6 维 TCP 后只剩 1 个冗余自由度，因此不能把 7 个关节当成 7 个独立自由度操作。</p>
            <div class="inline-settings-grid"><div class="step-box"><span>步进值 / 连续速度档</span><b><input id="nullStep" type="number" value="5" min=".1" max="30" step=".1"> deg</b><label class="step-mode-toggle">步进点动<input id="nullStepMode" type="checkbox"><i></i></label></div><button onclick="cmd('guide_current')">当前关节值设为参考</button></div>
            </section></article>
        </div>
        <div id="move-auxiliary" class="move-mode">
          <article class="panel grow" data-submodule="move-motion"><div class="panel-title">附加机构运动 <span>独立轴 · m / rad</span></div><div id="auxList" class="aux-list"></div><section class="inline-settings" data-submodule="move-settings"><div class="inline-settings-title">说明 <span>External axes</span></div><p class="hint">附加轴不参与机械臂 IK，运动时保持机械臂关节不被优化器修改。</p></section></article>
        </div>
          </div>
        </div>
      </section>

      <section id="page-program" class="page">
        <div class="page-head"><div><h1>示教程序</h1><p>MOVL 笛卡尔插补 / MOVJ 关节空间插补</p></div><span id="programStatus" class="state-tag">待机</span></div>
        <div class="program-layout">
          <article class="panel point-list-panel" data-submodule="program-points">
            <div class="panel-title">示教点列表 <span>勾选后按点表顺序执行</span></div>
            <div class="save-line">
              <input id="newPointName" maxlength="24" placeholder="示教点名称（可选）">
              <select id="newPointType"><option>MOVL</option><option>MOVJ</option></select>
              <button class="primary" onclick="saveCurrentPoint()">保存当前点位</button>
            </div>
            <div id="teachTable" class="teach-table"></div>
            <div class="pager"><button onclick="changePage(-1)">上一页</button><span id="pageInfo">0 / 0</span><button onclick="changePage(1)">下一页</button></div>
          </article>
          <aside class="panel program-side" data-submodule="program-editor">
            <div class="panel-title">点位编辑 <span id="editorHint">--</span></div>
            <input id="editName" placeholder="选择点位" disabled>
            <div id="pointEditor" class="editor-grid"></div>
            <div class="action-row two"><button onclick="updatePoint()">保存修改</button><button class="danger" onclick="deletePoint()">删除</button></div>
            <div class="divider"></div>
            <div class="form-row"><label>基础单点时长</label><input id="duration" type="number" min=".2" step=".1"></div>
            <div class="form-row"><label>插补频率</label><input id="frequency" type="number" min="50" max="1000" step="1"></div>
            <label class="switch-row">循环执行<input id="loopProgram" type="checkbox"><i></i></label>
            <div class="action-row two"><button class="run" onclick="runPoints()">▶ 运行勾选点</button><button class="danger" onclick="cmd('stop_teach_points')">■ 停止</button></div>
          </aside>
        </div>
      </section>

      <section id="page-config" class="page">
        <div class="page-head"><div><h1>配置</h1><p>运动限制、执行节拍与配置文件</p></div><span id="activeProfile" class="state-tag">未调用配置</span></div>
        <div class="config-layout">
          <article class="panel" data-submodule="config-motion">
            <div class="panel-title">全局速度 <span>实际速度 = 上限 × 百分比</span></div>
            <div class="big-speed"><button onclick="speedPreset(10)">10%</button><button onclick="speedPreset(30)">30%</button><button onclick="speedPreset(50)">50%</button><button onclick="speedPreset(100)">100%</button></div>
            <div class="config-grid">
              <div class="config-field"><label>速度百分比</label><div><input id="speedPercent" type="number" min="1" max="100" step="1"><em>%</em></div></div>
              <div class="config-field"><label>最大线速度</label><div><input id="maxLinear" type="number" min="1" max="2000"><em>mm/s</em></div></div>
              <div class="config-field"><label>最大角速度</label><div><input id="maxAngular" type="number" min="1" max="360"><em>deg/s</em></div></div>
              <div class="config-field"><label>最大关节速度</label><div><input id="maxJoint" type="number" min="1" max="360"><em>deg/s</em></div></div>
              <div class="config-field"><label>动作完成后延时</label><div><input id="commandDelay" type="number" min="0" max="60" step=".1"><em>s</em></div></div>
            </div>
            <button class="primary wide" onclick="applyMotionSettings()">应用运动配置</button>
          </article>
          <article class="panel" data-submodule="config-trajectory">
            <div class="panel-title">IK 与轨迹 <span>高级参数</span></div>
            <div class="config-grid compact-config">
              <div class="config-field"><label>恢复搜索种子数</label><div><input id="seeds" type="number" min="4" max="24"><em>个</em></div></div>
              <div class="config-field"><label>默认单点时长</label><div><input id="configDuration" type="number" min=".2" step=".1"><em>s</em></div></div>
              <div class="config-field"><label>轨迹频率</label><div><input id="configFrequency" type="number" min="50" max="1000"><em>Hz</em></div></div>
            </div>
            <p class="hint">速度百分比同时作用于连续点动、MOVL 和 MOVJ；最大速度作为轨迹硬上限。</p>
          </article>
          <aside class="panel profile-panel" data-submodule="config-profiles">
            <div class="panel-title">配置文件 <span>保存 / 调用</span></div>
            <input id="profileName" maxlength="32" placeholder="新配置名称">
            <button class="primary" onclick="saveProfile()">保存当前配置</button>
            <div id="profileList" class="profile-list"></div>
            <p class="hint">配置包含速度、延时、求解参数、轨迹参数和当前臂型参考。</p>
          </aside>
        </div>
      </section>

      <section id="page-diagnostics" class="page">
        <div class="page-head"><div><h1>诊断</h1><p>IK 误差、通信及机器人状态</p></div></div>
        <div class="diagnostic-layout">
          <article class="panel"><div class="panel-title">IK 求解质量 <span>实时</span></div><div class="metric-grid diagnostic">
            <div class="metric"><b id="poserr">--</b><span>位置误差 mm</span></div>
            <div class="metric"><b id="orierr">--</b><span>姿态误差 deg</span></div>
            <div class="metric"><b id="attempts">--</b><span>求解尝试</span></div>
            <div class="metric"><b id="dragState">锁定</b><span>场景拖拽</span></div>
          </div></article>
          <article class="panel log-panel"><div class="panel-title">运行信息 <span>最近消息</span></div><div id="diagnosticMessage">等待通信……</div></article>
          <article class="panel"><div class="panel-title">系统操作 <span>仿真</span></div><div class="quick-grid diagnostic-actions">
            <button onclick="cmd('solve')"><b>执行 IK</b><small>当前目标单次求解</small></button>
            <button onclick="cmd('recover')"><b>恢复求解</b><small>强制多起点搜索</small></button>
            <button onclick="cmd('toggle_drag')"><b>场景拖拽</b><small>切换 TCP 拖拽状态</small></button>
            <button class="warn" onclick="cmd('reset')"><b>整机复位</b><small>恢复初始状态</small></button>
          </div></article>
        </div>
      </section>
      </main>
      <aside id="viserDock" class="viser-dock" aria-label="机械臂实时仿真">
        <div class="viser-head"><b>机械臂实时仿真</b><span>Viser · 实时</span></div>
        <iframe id="viserFrame" title="机器人三维仿真" src="about:blank" data-src="__VISER_URL__"></iframe>
      </aside>
    </div>
  </div>

  <footer class="footer">
    <div id="message" class="message">正在连接仿真……</div>
    <div class="footer-speed"><span>全局速度</span><button onclick="adjustSpeed(-5)">−</button><b id="footerSpeed">30%</b><button onclick="adjustSpeed(5)">＋</button></div>
    <button class="stop" onclick="stopAll()">■ 停止运动</button>
  </footer>
</div>
<script src="/assets/pendant.js"></script>
</body>
</html>
"""
