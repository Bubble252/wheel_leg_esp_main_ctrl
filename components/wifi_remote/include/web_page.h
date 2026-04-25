/**
 * @file web_page.h
 * @brief WiFi 遥控网页 (HTML + JavaScript)
 * @author Bubble
 * @date 2026-01-16
 * @note 移植自 shibo_wheel_leg 项目的 basic_web.h
 */

#ifndef WEB_PAGE_H
#define WEB_PAGE_H

// HTML 网页内容
static const char web_page_html[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WL-PRO WiFi Remote</title>
  <style>
    body {
        font-family: Arial, sans-serif;
        background-color: #f5f5f5;
        margin: 0;
        padding: 10px;
    }
    h2 {
        width: auto;
        height: 50px;
        line-height: 50px;
        text-align: center;
        color: white;
        background-color: cornflowerblue;
        border-radius: 12px;
        margin: 10px 0;
    }
    .container {
        max-width: 400px;
        margin: 0 auto;
    }
    .status {
        text-align: center;
        padding: 10px;
        margin: 10px 0;
        border-radius: 8px;
        font-weight: bold;
    }
    .connected { background-color: #4CAF50; color: white; }
    .disconnected { background-color: #f44336; color: white; }
    
    .switch-container {
        text-align: center;
        margin: 15px 0;
    }
    input[type='checkbox'].switch {
        outline: none;
        appearance: none;
        -webkit-appearance: none;
        position: relative;
        width: 50px;
        height: 26px;
        background: #ccc;
        border-radius: 13px;
        transition: 0.3s;
        cursor: pointer;
        vertical-align: middle;
    }
    input[type='checkbox'].switch::after {
        content: '';
        display: inline-block;
        width: 22px;
        height: 22px;
        border-radius: 50%;
        background: #fff;
        position: absolute;
        top: 2px;
        left: 2px;
        transition: 0.3s;
    }
    input[type='checkbox'].switch:checked {
        background: #4CAF50;
    }
    input[type='checkbox'].switch.car-switch:checked {
        background: #FF9800;
    }
    input[type='checkbox'].switch:checked::after {
        left: 26px;
    }
    
    .joystick-container {
        width: 200px;
        height: 200px;
        margin: 20px auto;
    }
    
    .sliders {
        padding: 10px 20px;
    }
    .slider-row {
        margin: 15px 0;
    }
    .slider-row label {
        display: block;
        margin-bottom: 5px;
        font-size: 14px;
    }
    .slider-row input[type="range"] {
        width: 100%;
    }
    
    .buttons {
        display: grid;
        grid-template-columns: repeat(3, 1fr);
        gap: 10px;
        padding: 10px 20px;
        max-width: 320px;
        margin: 0 auto;
    }
    .dir-btn {
        height: 50px;
        font-size: 14px;
        border-radius: 10px;
        background-color: white;
        color: cornflowerblue;
        border: 2px solid cornflowerblue;
        cursor: pointer;
        transition: 0.2s;
    }
    .dir-btn:active {
        background-color: cornflowerblue;
        color: white;
    }
    .dir-btn.center { grid-column: 2; }
    .dir-btn.left { grid-column: 1; grid-row: 2; }
    .dir-btn.right { grid-column: 3; grid-row: 2; }
    .dir-btn.forward { grid-column: 2; grid-row: 1; }
    .dir-btn.back { grid-column: 2; grid-row: 3; }
    .dir-btn.jump { grid-column: 2; grid-row: 2; }
    
    .debug-info {
        background: #333;
        color: #0f0;
        font-family: monospace;
        font-size: 12px;
        padding: 10px;
        border-radius: 8px;
        margin: 10px 20px;
        max-height: 150px;
        overflow-y: auto;
    }
    
    /* 扩展控制面板 */
    .ctrl-panel {
        background: rgba(255,255,255,0.08);
        border-radius: 12px;
        padding: 12px 16px;
        margin: 10px 20px;
    }
    .ctrl-panel h3 {
        margin: 0 0 10px 0;
        font-size: 16px;
        color: #aaa;
        border-bottom: 1px solid #444;
        padding-bottom: 6px;
    }
    .ctrl-row {
        display: flex;
        align-items: center;
        justify-content: space-between;
        margin: 8px 0;
    }
    .ctrl-row label {
        font-size: 14px;
        flex: 1;
    }
    .ctrl-row select {
        background: #333;
        color: white;
        border: 1px solid #555;
        border-radius: 6px;
        padding: 6px 10px;
        font-size: 14px;
        outline: none;
    }
    .switch.estop-switch:checked + label::before {
        content: '';
    }
    .switch.estop-switch + label {
        cursor: pointer;
    }
    /* E-Stop 红色样式 */
    .estop-container {
        margin: 8px 0;
    }
    .estop-btn {
        width: 100%;
        height: 50px;
        border-radius: 10px;
        font-size: 18px;
        font-weight: bold;
        border: 2px solid #f44336;
        cursor: pointer;
        transition: 0.2s;
        background: #333;
        color: #f44336;
    }
    .estop-btn.active {
        background: #f44336;
        color: white;
        animation: pulse-red 1s infinite;
    }
    @keyframes pulse-red {
        0%, 100% { box-shadow: 0 0 5px #f44336; }
        50% { box-shadow: 0 0 20px #f44336; }
    }
    
    /* 详细调控面板 */
    .detail-panel {
        background: rgba(255,255,255,0.08);
        border-radius: 12px;
        padding: 12px 16px;
        margin: 10px 20px;
        border: 2px solid #2196F3;
    }
    .detail-panel h3 {
        margin: 0 0 10px 0;
        font-size: 16px;
        color: #2196F3;
        border-bottom: 1px solid #2196F3;
        padding-bottom: 6px;
    }
    .detail-panel.hidden {
        display: none;
    }
    .detail-side {
        background: rgba(255,255,255,0.04);
        border-radius: 8px;
        padding: 8px 12px;
        margin: 8px 0;
    }
    .detail-side h4 {
        margin: 0 0 6px 0;
        font-size: 14px;
        color: #aaa;
    }
    .detail-side.left-side { border-left: 3px solid #4CAF50; }
    .detail-side.right-side { border-left: 3px solid #FF9800; }
    .detail-side.synced { border-left: 3px solid #2196F3; }
    .sync-indicator {
        text-align: center;
        font-size: 12px;
        color: #2196F3;
        padding: 4px;
        display: none;
    }
    .sync-indicator.active { display: block; }
    
    * {
        -webkit-touch-callout: none;
        -webkit-user-select: none;
        user-select: none;
    }
  </style>
</head>
<body>
    <div class="container">
        <h2>WL-PRO WiFi Remote</h2>
        
        <div id="status" class="status disconnected">Disconnected</div>
        
        <!-- 紧急停止 -->
        <div class="ctrl-panel">
            <div class="estop-container">
                <button class="estop-btn" id="estopBtn" onclick="toggleEstop()">🛑 Emergency Stop</button>
            </div>
        </div>
        
        <!-- 扩展控制面板 -->
        <div class="ctrl-panel">
            <h3>⚙️ Control Settings</h3>
            
            <div class="ctrl-row">
                <label>🎯 Balance Enable</label>
                <input type="checkbox" id="balanceSwitch" class="switch" onclick="toggleBalance()">
            </div>
            
            <div class="ctrl-row">
                <label>� Yaw Enable</label>
                <input type="checkbox" id="yawEnableSwitch" class="switch" onclick="toggleYawEnable()">
            </div>            
            <div class="ctrl-row">
                <label>↔️ Diff Steer (差速转向)</label>
                <input type="checkbox" id="diffSpeedSwitch" class="switch" onclick="toggleDiffSpeed()">
            </div>            
            <div class="ctrl-row">
                <label>�📐 Pitch Compensation</label>
                <input type="checkbox" id="pitchCompSwitch" class="switch" onclick="togglePitchComp()">
            </div>
            
            <div class="ctrl-row">
                <label>🔧 Control Mode</label>
                <select id="ctrlModeSelect" onchange="updateCtrlMode()">
                    <option value="0">LQR</option>
                    <option value="1">Dual PID</option>
                    <option value="2">Single PID</option>
                    <option value="4" selected>Triple PID</option>
                </select>
            </div>
            
            <div class="ctrl-row">
                <label>🚗 Car Mode (趴下跑)</label>
                <input type="checkbox" id="carModeSwitch" class="switch" onclick="toggleCarMode()">
            </div>
            
            <div style="text-align:center; margin:6px 0;">
                <button class="dir-btn" style="width:160px; background:#28a745; border-color:#1e7e34; color:white;"
                        ontouchstart="carStandupPress()" ontouchend="carStandupRelease()"
                        onmousedown="carStandupPress()" onmouseup="carStandupRelease()">🏃 Car Standup</button>
            </div>
            
            <div class="slider-row">
                <label id="angleZeroLabel">🎯 Angle Zero: 7.40°</label>
                <div style="display:flex; align-items:center; gap:4px;">
                    <button class="dir-btn" style="width:36px; height:32px; font-size:16px; padding:0;" onclick="nudgeAngleZero(-0.1)">◀</button>
                    <input type="range" id="angleZeroSlider" min="-30" max="30" value="7.4" step="0.1" style="flex:1;" oninput="updateAngleZero()">
                    <button class="dir-btn" style="width:36px; height:32px; font-size:16px; padding:0;" onclick="nudgeAngleZero(0.1)">▶</button>
                </div>
            </div>
            
            <div class="slider-row">
                <label id="joySpeedLabel">🏎️ Speed Gain: 3.0 ‰</label>
                <input type="range" id="joySpeedSlider" min="1" max="20" value="3" step="0.5" oninput="updateJoySpeed()">
            </div>
            
            <div class="slider-row">
                <label id="joyYawLabel">🔄 Yaw Gain: 30 ‰</label>
                <input type="range" id="joyYawSlider" min="5" max="200" value="30" step="1" oninput="updateJoyYaw()">
            </div>
            
            <div class="ctrl-row">
                <label>� Distance Loop (位移环)</label>
                <input type="checkbox" id="distEnableSwitch" class="switch" checked onclick="toggleDistEnable()">
            </div>            
            <div class="ctrl-row">
                <label>👁️ Observer Speed (观测器速度)</label>
                <input type="checkbox" id="obsvSpeedSwitch" class="switch" onclick="toggleObsvSpeed()">
            </div>
            
            <div class="ctrl-row">
                <label>↔️ X-Offset Enable</label>
                <input type="checkbox" id="xoffsetEnableSwitch" class="switch" onclick="toggleXoffsetEnable()">
            </div>
            
            <div class="slider-row">
                <label id="xoffsetKpLabel">↔️ X-Offset Kp: 0.100</label>
                <input type="range" id="xoffsetKpSlider" min="0" max="1000" value="100" step="1" oninput="updateXoffsetKp()">
            </div>        </div>
        
        <!-- 腿部控制面板 -->
        <div class="ctrl-panel">
            <h3>🦿 Leg Control</h3>
            
            <div class="ctrl-row">
                <label>🔌 Leg Enable</label>
                <input type="checkbox" id="legEnableSwitch" class="switch" onclick="toggleLegEnable()">
            </div>
            
            <div class="ctrl-row">
                <label>↔️ Roll Enable</label>
                <input type="checkbox" id="rollEnableSwitch" class="switch" onclick="toggleRollEnable()">
            </div>
            
            <div class="slider-row">
                <label id="legAngleLabel">Leg Angle: -90°</label>
                <input type="range" id="legAngleSlider" min="-160" max="10" value="-90" oninput="updateLegAngle()">
            </div>
            
            <div class="slider-row">
                <label id="legLengthLabel">Leg Length: 90 mm</label>
                <input type="range" id="legLengthSlider" min="45" max="110" value="90" oninput="updateLegLength()">
            </div>
        </div>
        
        <!-- 详细调控模式面板 -->
        <div class="ctrl-panel">
            <div class="ctrl-row">
                <label>🔬 Detail Control Mode</label>
                <input type="checkbox" id="detailModeSwitch" class="switch" onclick="toggleDetailMode()">
            </div>
        </div>
        
        <div class="detail-panel hidden" id="detailPanel">
            <h3>🔬 Detailed Control</h3>
            
            <div class="ctrl-row">
                <label>🔗 Sync L/R (协同控制)</label>
                <input type="checkbox" id="detailSyncSwitch" class="switch" checked onclick="toggleDetailSync()">
            </div>
            <div class="sync-indicator active" id="syncIndicator">🔗 协同模式: 左侧滑条同时控制双腿双轮</div>
            
            <!-- 左侧控制 -->
            <div class="detail-side left-side" id="leftSide">
                <h4>🟢 Left / 左侧</h4>
                <div class="slider-row">
                    <label id="dlLengthLabel">Leg Length: 90 mm</label>
                    <input type="range" id="dlLengthSlider" min="45" max="110" value="90" oninput="updateDetailLeft()">
                </div>
                <div class="slider-row">
                    <label id="dlAngleLabel">Body Angle: -90°</label>
                    <input type="range" id="dlAngleSlider" min="-160" max="10" value="-90" oninput="updateDetailLeft()">
                </div>
                <div class="slider-row">
                    <label id="dlSpeedLabel">Wheel Speed: 0 rpm</label>
                    <input type="range" id="dlSpeedSlider" min="-200" max="200" value="0" oninput="updateDetailLeft()">
                </div>
                <div style="text-align:center; margin-top:4px;">
                    <button class="dir-btn" style="width:120px; height:32px; font-size:12px;" onclick="resetDetailLeftSpeed()">Reset Speed</button>
                </div>
            </div>
            
            <!-- 右侧控制 -->
            <div class="detail-side right-side" id="rightSide">
                <h4>🟠 Right / 右侧</h4>
                <div class="slider-row">
                    <label id="drLengthLabel">Leg Length: 90 mm</label>
                    <input type="range" id="drLengthSlider" min="45" max="110" value="90" oninput="updateDetailRight()">
                </div>
                <div class="slider-row">
                    <label id="drAngleLabel">Body Angle: -90°</label>
                    <input type="range" id="drAngleSlider" min="-160" max="10" value="-90" oninput="updateDetailRight()">
                </div>
                <div class="slider-row">
                    <label id="drSpeedLabel">Wheel Speed: 0 rpm</label>
                    <input type="range" id="drSpeedSlider" min="-200" max="200" value="0" oninput="updateDetailRight()">
                </div>
                <div style="text-align:center; margin-top:4px;">
                    <button class="dir-btn" style="width:120px; height:32px; font-size:12px;" onclick="resetDetailRightSpeed()">Reset Speed</button>
                </div>
            </div>
        </div>
        
        <div class="joystick-container" id="joystickDiv"></div>
        
        <div class="sliders">
            <div class="slider-row">
                <label id="heightLabel">Height: 38 mm</label>
                <input type="range" id="heightSlider" min="32" max="85" value="38" oninput="updateHeight()">
            </div>
            <div class="slider-row">
                <label id="rollLabel">Roll: 0°</label>
                <input type="range" id="rollSlider" min="-30" max="30" value="0" oninput="updateRoll()">
            </div>
        </div>
        
        <div class="buttons">
            <div></div>
            <button class="dir-btn forward" ontouchstart="dirPress('forward')" ontouchend="dirRelease()" 
                    onmousedown="dirPress('forward')" onmouseup="dirRelease()">Forward</button>
            <div></div>
            <button class="dir-btn left" ontouchstart="dirPress('left')" ontouchend="dirRelease()"
                    onmousedown="dirPress('left')" onmouseup="dirRelease()">Left</button>
            <button class="dir-btn jump" ontouchstart="dirPress('jump')" ontouchend="dirRelease()"
                    onmousedown="dirPress('jump')" onmouseup="dirRelease()">Jump</button>
            <button class="dir-btn right" ontouchstart="dirPress('right')" ontouchend="dirRelease()"
                    onmousedown="dirPress('right')" onmouseup="dirRelease()">Right</button>
            <div></div>
            <button class="dir-btn back" ontouchstart="dirPress('back')" ontouchend="dirRelease()"
                    onmousedown="dirPress('back')" onmouseup="dirRelease()">Back</button>
            <div></div>
        </div>
        
        <div style="text-align:center; margin:10px 0;">
            <button class="dir-btn" style="width:120px; background:#ff8c00; border-color:#ff8c00; color:white;"
                    ontouchstart="standupPress()" ontouchend="standupRelease()"
                    onmousedown="standupPress()" onmouseup="standupRelease()">Standup</button>
        </div>
        
        <div class="debug-info" id="debugInfo">Waiting for connection...</div>
    </div>

<script>
    var socket;
    var g_go = 0;
    var g_carMode = 0;
    var g_yawEnable = 0;
    var g_height = 38;
    var g_roll = 0;
    var g_joyX = 0;
    var g_joyY = 0;
    var g_dir = "stop";
    var g_balanceEnable = 0;
    var g_estop = 0;
    var g_ctrlMode = 4;
    var g_pitchComp = 0;
    var g_legEnable = 0;
    var g_rollEnable = 0;
    var g_legAngle = -90;
    var g_legLength = 90;
    var g_joySpeedGain = 3.0;
    var g_joyYawGain = 30;
    var g_distEnable = 1;
    var g_detailMode = 0;
    var g_detailSync = 1;
    var g_dlLength = 90;
    var g_dlAngle = -90;
    var g_dlSpeed = 0;
    var g_drLength = 90;
    var g_drAngle = -90;
    var g_drSpeed = 0;
    var msgCount = 0;
    var heartbeatTimer = null;
    var g_standup = 0;
    var g_carStandup = 0;
    var g_angleZero = 7.4;
    var g_obsvSpeed = 0;
    var g_xoffsetEnable = 0;
    var g_xoffsetKp = 100;
    var g_diffSpeed = 0;
    
    // WebSocket 初始化
    function initWebSocket() {
        var wsUrl = 'ws://' + window.location.hostname + '/ws';
        socket = new WebSocket(wsUrl);
        
        socket.onopen = function() {
            document.getElementById('status').className = 'status connected';
            document.getElementById('status').innerText = 'Connected';
            log('WebSocket connected');
            // 启动心跳: 每500ms发送一次数据，防止看门狗超时
            if (heartbeatTimer) clearInterval(heartbeatTimer);
            heartbeatTimer = setInterval(function(){ sendData(true); }, 500);
        };
        
        socket.onclose = function() {
            document.getElementById('status').className = 'status disconnected';
            document.getElementById('status').innerText = 'Disconnected';
            log('WebSocket disconnected, reconnecting...');
            if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
            setTimeout(initWebSocket, 2000);
        };
        
        socket.onerror = function(e) {
            log('WebSocket error');
        };
        
        socket.onmessage = function(e) {
            log('Received: ' + e.data);
        };
    }
    
    function log(msg) {
        var debugDiv = document.getElementById('debugInfo');
        var time = new Date().toLocaleTimeString();
        debugDiv.innerHTML = '[' + time + '] ' + msg + '<br>' + debugDiv.innerHTML;
        if (debugDiv.innerHTML.length > 2000) {
            debugDiv.innerHTML = debugDiv.innerHTML.substring(0, 2000);
        }
    }
    
    function sendData(silent) {
        if (socket && socket.readyState === WebSocket.OPEN) {
            var data = {
                mode: 'basic',
                dir: g_dir,
                joy_x: parseInt(g_joyX),
                joy_y: parseInt(g_joyY),
                height: g_height,
                roll: g_roll,
                stable: g_go,
                car_mode: g_carMode,
                linear: 0,
                angular: 0,
                balance_enable: g_balanceEnable,
                estop: g_estop,
                control_mode: g_ctrlMode,
                pitch_comp: g_pitchComp,
                leg_enable: g_legEnable,
                leg_angle: g_legAngle,
                leg_length: g_legLength / 1000.0,
                detail_mode: g_detailMode,
                detail_sync: g_detailSync,
                detail_left_length: g_dlLength / 1000.0,
                detail_left_angle: g_dlAngle,
                detail_left_speed: g_dlSpeed,
                detail_right_length: g_drLength / 1000.0,
                detail_right_angle: g_drAngle,
                detail_right_speed: g_drSpeed,
                joy_speed_gain: g_joySpeedGain / 1000.0,
                joy_yaw_gain: g_joyYawGain / 1000.0,
                dist_enable: g_distEnable,
                yaw_enable: g_yawEnable,
                roll_enable: g_rollEnable,
                standup: g_standup,
                car_standup: g_carStandup,
                angle_zero: parseFloat(g_angleZero.toFixed(2)),
                obsv_speed: g_obsvSpeed,
                xoffset_enable: g_xoffsetEnable,
                xoffset_kp: g_xoffsetKp / 1000.0,
                diff_speed_enable: g_diffSpeed
            };
            socket.send(JSON.stringify(data));
            msgCount++;
            if (!silent) {
                log('TX[' + msgCount + ']: dir=' + g_dir + ' joy=(' + g_joyX + ',' + g_joyY + ') go=' + g_go);
            }
        }
    }
    
    function toggleYawEnable() {
        g_yawEnable = document.getElementById('yawEnableSwitch').checked ? 1 : 0;
        sendData();
        log('Yaw enable: ' + (g_yawEnable ? 'ON' : 'OFF'));
    }
    
    function toggleDiffSpeed() {
        g_diffSpeed = document.getElementById('diffSpeedSwitch').checked ? 1 : 0;
        sendData();
        log('Diff steer: ' + (g_diffSpeed ? 'ON' : 'OFF'));
    }
    
    function updateHeight() {
        g_height = parseInt(document.getElementById('heightSlider').value);
        document.getElementById('heightLabel').innerText = 'Height: ' + g_height + ' mm';
        sendData();
    }
    
    function updateRoll() {
        g_roll = parseInt(document.getElementById('rollSlider').value);
        document.getElementById('rollLabel').innerText = 'Roll: ' + g_roll + '°';
        sendData();
    }
    
    function dirPress(dir) {
        g_dir = dir;
        sendData();
    }
    
    function dirRelease() {
        g_dir = 'stop';
        sendData();
    }
    
    function standupPress() {
        g_standup = 1;
        sendData();
    }
    
    function standupRelease() {
        g_standup = 0;
        sendData();
    }
    
    function carStandupPress() {
        g_carStandup = 1;
        sendData();
    }
    
    function carStandupRelease() {
        g_carStandup = 0;
        sendData();
    }
    
    function toggleBalance() {
        g_balanceEnable = document.getElementById('balanceSwitch').checked ? 1 : 0;
        sendData();
        log('Balance: ' + (g_balanceEnable ? 'ON' : 'OFF'));
    }
    
    function toggleEstop() {
        g_estop = g_estop ? 0 : 1;
        var btn = document.getElementById('estopBtn');
        if (g_estop) {
            btn.className = 'estop-btn active';
            btn.innerText = '🛑 E-STOP ACTIVE!';
            // 急停时自动关闭平衡
            g_balanceEnable = 0;
            document.getElementById('balanceSwitch').checked = false;
        } else {
            btn.className = 'estop-btn';
            btn.innerText = '🛑 Emergency Stop';
        }
        sendData();
        log('E-Stop: ' + (g_estop ? 'ACTIVE' : 'Released'));
    }
    
    function toggleCarMode() {
        g_carMode = document.getElementById('carModeSwitch').checked ? 1 : 0;
        sendData();
        log('Car Mode: ' + (g_carMode ? 'ON' : 'OFF'));
    }
    
    function updateCtrlMode() {
        g_ctrlMode = parseInt(document.getElementById('ctrlModeSelect').value);
        var names = {0:'LQR', 1:'Dual PID', 2:'Single PID', 4:'Triple PID'};
        sendData();
        log('Control mode: ' + (names[g_ctrlMode] || 'Unknown'));
    }
    
    function togglePitchComp() {
        g_pitchComp = document.getElementById('pitchCompSwitch').checked ? 1 : 0;
        sendData();
        log('Pitch Comp: ' + (g_pitchComp ? 'ON' : 'OFF'));
    }
    
    function toggleLegEnable() {
        g_legEnable = document.getElementById('legEnableSwitch').checked ? 1 : 0;
        sendData();
        log('Leg Enable: ' + (g_legEnable ? 'ON' : 'OFF'));
    }
    
    function toggleRollEnable() {
        g_rollEnable = document.getElementById('rollEnableSwitch').checked ? 1 : 0;
        sendData();
        log('Roll Enable: ' + (g_rollEnable ? 'ON' : 'OFF'));
    }
    
    function updateLegAngle() {
        g_legAngle = parseInt(document.getElementById('legAngleSlider').value);
        document.getElementById('legAngleLabel').innerText = 'Leg Angle: ' + g_legAngle + '°';
        sendData();
    }
    
    function updateLegLength() {
        g_legLength = parseInt(document.getElementById('legLengthSlider').value);
        document.getElementById('legLengthLabel').innerText = 'Leg Length: ' + g_legLength + ' mm';
        sendData();
    }
    
    function updateAngleZero() {
        g_angleZero = parseFloat(document.getElementById('angleZeroSlider').value);
        document.getElementById('angleZeroLabel').innerText = '🎯 Angle Zero: ' + g_angleZero.toFixed(1) + '°';
        sendData();
        log('Angle zeropoint: ' + g_angleZero.toFixed(2) + '°');
    }
    
    function nudgeAngleZero(delta) {
        g_angleZero = Math.max(-30, Math.min(30, parseFloat((g_angleZero + delta).toFixed(2))));
        document.getElementById('angleZeroSlider').value = g_angleZero;
        document.getElementById('angleZeroLabel').innerText = '🎯 Angle Zero: ' + g_angleZero.toFixed(1) + '°';
        sendData();
        log('Angle zeropoint nudge: ' + g_angleZero.toFixed(2) + '°');
    }
    
    function toggleObsvSpeed() {
        g_obsvSpeed = document.getElementById('obsvSpeedSwitch').checked ? 1 : 0;
        sendData();
        log('Observer speed: ' + (g_obsvSpeed ? 'ON' : 'OFF'));
    }
    
    function toggleXoffsetEnable() {
        g_xoffsetEnable = document.getElementById('xoffsetEnableSwitch').checked ? 1 : 0;
        sendData();
        log('X-Offset: ' + (g_xoffsetEnable ? 'ON' : 'OFF'));
    }
    
    function updateXoffsetKp() {
        g_xoffsetKp = parseFloat(document.getElementById('xoffsetKpSlider').value);
        document.getElementById('xoffsetKpLabel').innerText = '↔️ X-Offset Kp: ' + (g_xoffsetKp / 1000).toFixed(3);
        sendData();
        log('X-Offset Kp: ' + (g_xoffsetKp / 1000).toFixed(3));
    }
    
    function updateJoySpeed() {
        g_joySpeedGain = parseFloat(document.getElementById('joySpeedSlider').value);
        document.getElementById('joySpeedLabel').innerText = '🏎️ Speed Gain: ' + g_joySpeedGain.toFixed(1) + ' ‰';
        sendData();
        log('Joy speed gain: ' + g_joySpeedGain + '‰ (scale=' + (g_joySpeedGain/1000).toFixed(4) + ')');
    }
    
    function updateJoyYaw() {
        g_joyYawGain = parseInt(document.getElementById('joyYawSlider').value);
        document.getElementById('joyYawLabel').innerText = '🔄 Yaw Gain: ' + g_joyYawGain + ' ‰';
        sendData();
        log('Joy yaw gain: ' + g_joyYawGain + '‰ (scale=' + (g_joyYawGain/1000).toFixed(4) + ')');
    }
    
    function toggleDistEnable() {
        g_distEnable = document.getElementById('distEnableSwitch').checked ? 1 : 0;
        sendData();
        log('Distance loop: ' + (g_distEnable ? 'ON' : 'OFF'));
    }
    
    // ======== 详细调控模式 ========
    function toggleDetailMode() {
        g_detailMode = document.getElementById('detailModeSwitch').checked ? 1 : 0;
        var panel = document.getElementById('detailPanel');
        if (g_detailMode) {
            panel.className = 'detail-panel';
        } else {
            panel.className = 'detail-panel hidden';
        }
        sendData();
        log('Detail mode: ' + (g_detailMode ? 'ON' : 'OFF'));
    }
    
    function toggleDetailSync() {
        g_detailSync = document.getElementById('detailSyncSwitch').checked ? 1 : 0;
        var rightSide = document.getElementById('rightSide');
        var syncInd = document.getElementById('syncIndicator');
        var leftSide = document.getElementById('leftSide');
        if (g_detailSync) {
            rightSide.style.opacity = '0.4';
            rightSide.style.pointerEvents = 'none';
            syncInd.className = 'sync-indicator active';
            leftSide.className = 'detail-side left-side synced';
            // 同步右侧显示为左侧值
            syncRightToLeft();
        } else {
            rightSide.style.opacity = '1';
            rightSide.style.pointerEvents = 'auto';
            syncInd.className = 'sync-indicator';
            leftSide.className = 'detail-side left-side';
        }
        sendData();
        log('Detail sync: ' + (g_detailSync ? 'SYNC' : 'INDEPENDENT'));
    }
    
    function syncRightToLeft() {
        g_drLength = g_dlLength;
        g_drAngle = g_dlAngle;
        g_drSpeed = g_dlSpeed;
        document.getElementById('drLengthSlider').value = g_drLength;
        document.getElementById('drAngleSlider').value = g_drAngle;
        document.getElementById('drSpeedSlider').value = g_drSpeed;
        document.getElementById('drLengthLabel').innerText = 'Leg Length: ' + g_drLength + ' mm';
        document.getElementById('drAngleLabel').innerText = 'Body Angle: ' + g_drAngle + '°';
        document.getElementById('drSpeedLabel').innerText = 'Wheel Speed: ' + g_drSpeed + ' rpm';
    }
    
    function updateDetailLeft() {
        g_dlLength = parseInt(document.getElementById('dlLengthSlider').value);
        g_dlAngle = parseInt(document.getElementById('dlAngleSlider').value);
        g_dlSpeed = parseInt(document.getElementById('dlSpeedSlider').value);
        document.getElementById('dlLengthLabel').innerText = 'Leg Length: ' + g_dlLength + ' mm';
        document.getElementById('dlAngleLabel').innerText = 'Body Angle: ' + g_dlAngle + '°';
        document.getElementById('dlSpeedLabel').innerText = 'Wheel Speed: ' + g_dlSpeed + ' rpm';
        if (g_detailSync) { syncRightToLeft(); }
        sendData();
    }
    
    function updateDetailRight() {
        g_drLength = parseInt(document.getElementById('drLengthSlider').value);
        g_drAngle = parseInt(document.getElementById('drAngleSlider').value);
        g_drSpeed = parseInt(document.getElementById('drSpeedSlider').value);
        document.getElementById('drLengthLabel').innerText = 'Leg Length: ' + g_drLength + ' mm';
        document.getElementById('drAngleLabel').innerText = 'Body Angle: ' + g_drAngle + '°';
        document.getElementById('drSpeedLabel').innerText = 'Wheel Speed: ' + g_drSpeed + ' rpm';
        sendData();
    }
    
    function resetDetailLeftSpeed() {
        g_dlSpeed = 0;
        document.getElementById('dlSpeedSlider').value = 0;
        document.getElementById('dlSpeedLabel').innerText = 'Wheel Speed: 0 rpm';
        if (g_detailSync) { syncRightToLeft(); }
        sendData();
    }
    
    function resetDetailRightSpeed() {
        g_drSpeed = 0;
        document.getElementById('drSpeedSlider').value = 0;
        document.getElementById('drSpeedLabel').innerText = 'Wheel Speed: 0 rpm';
        sendData();
    }
    
    // 简易摇杆实现
    var JoyStick = function(containerId) {
        var container = document.getElementById(containerId);
        var canvas = document.createElement('canvas');
        canvas.width = 200;
        canvas.height = 200;
        container.appendChild(canvas);
        var ctx = canvas.getContext('2d');
        
        var centerX = 100, centerY = 100;
        var stickX = centerX, stickY = centerY;
        var maxRadius = 60;
        var stickRadius = 30;
        var isDragging = false;
        
        function draw() {
            ctx.clearRect(0, 0, 200, 200);
            // 外圈
            ctx.beginPath();
            ctx.arc(centerX, centerY, maxRadius + 10, 0, Math.PI * 2);
            ctx.strokeStyle = '#0097BC';
            ctx.lineWidth = 2;
            ctx.stroke();
            // 摇杆
            ctx.beginPath();
            ctx.arc(stickX, stickY, stickRadius, 0, Math.PI * 2);
            var grd = ctx.createRadialGradient(stickX, stickY, 5, stickX, stickY, stickRadius);
            grd.addColorStop(0, '#00979C');
            grd.addColorStop(1, '#006060');
            ctx.fillStyle = grd;
            ctx.fill();
        }
        
        function updateStick(x, y) {
            var dx = x - centerX;
            var dy = y - centerY;
            var dist = Math.sqrt(dx*dx + dy*dy);
            if (dist > maxRadius) {
                dx = dx / dist * maxRadius;
                dy = dy / dist * maxRadius;
            }
            stickX = centerX + dx;
            stickY = centerY + dy;
            g_joyX = Math.round(dx / maxRadius * 100);
            g_joyY = Math.round(-dy / maxRadius * 100);
            draw();
            sendData();
        }
        
        function resetStick() {
            stickX = centerX;
            stickY = centerY;
            g_joyX = 0;
            g_joyY = 0;
            draw();
            sendData();
        }
        
        function getPos(e) {
            var rect = canvas.getBoundingClientRect();
            if (e.touches) {
                return { x: e.touches[0].clientX - rect.left, y: e.touches[0].clientY - rect.top };
            }
            return { x: e.clientX - rect.left, y: e.clientY - rect.top };
        }
        
        canvas.addEventListener('mousedown', function(e) { isDragging = true; updateStick(getPos(e).x, getPos(e).y); });
        canvas.addEventListener('mousemove', function(e) { if (isDragging) updateStick(getPos(e).x, getPos(e).y); });
        document.addEventListener('mouseup', function() { if (isDragging) { isDragging = false; resetStick(); } });
        
        canvas.addEventListener('touchstart', function(e) { e.preventDefault(); isDragging = true; updateStick(getPos(e).x, getPos(e).y); });
        canvas.addEventListener('touchmove', function(e) { e.preventDefault(); if (isDragging) updateStick(getPos(e).x, getPos(e).y); });
        document.addEventListener('touchend', function() { if (isDragging) { isDragging = false; resetStick(); } });
        
        draw();
    };
    
    // 页面加载完成后初始化
    window.onload = function() {
        initWebSocket();
        new JoyStick('joystickDiv');
        // 初始化详细调控面板: 协同模式下禁用右侧
        var rightSide = document.getElementById('rightSide');
        rightSide.style.opacity = '0.4';
        rightSide.style.pointerEvents = 'none';
    };
</script>
</body>
</html>
)rawliteral";

#endif // WEB_PAGE_H
