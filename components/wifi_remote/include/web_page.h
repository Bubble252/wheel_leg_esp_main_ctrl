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
        
        <div class="switch-container">
            <input type="checkbox" id="goSwitch" class="switch" onclick="toggleGo()">
            <label for="goSwitch" style="font-size:18px; font-weight:bold;"> Robot Go!</label>
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
        
        <div class="debug-info" id="debugInfo">Waiting for connection...</div>
    </div>

<script>
    var socket;
    var g_go = 0;
    var g_height = 38;
    var g_roll = 0;
    var g_joyX = 0;
    var g_joyY = 0;
    var g_dir = "stop";
    var msgCount = 0;
    
    // WebSocket 初始化
    function initWebSocket() {
        var wsUrl = 'ws://' + window.location.hostname + '/ws';
        socket = new WebSocket(wsUrl);
        
        socket.onopen = function() {
            document.getElementById('status').className = 'status connected';
            document.getElementById('status').innerText = 'Connected';
            log('WebSocket connected');
        };
        
        socket.onclose = function() {
            document.getElementById('status').className = 'status disconnected';
            document.getElementById('status').innerText = 'Disconnected';
            log('WebSocket disconnected, reconnecting...');
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
    
    function sendData() {
        if (socket && socket.readyState === WebSocket.OPEN) {
            var data = {
                mode: 'basic',
                dir: g_dir,
                joy_x: parseInt(g_joyX),
                joy_y: parseInt(g_joyY),
                height: g_height,
                roll: g_roll,
                stable: g_go,
                linear: 0,
                angular: 0
            };
            socket.send(JSON.stringify(data));
            msgCount++;
            log('TX[' + msgCount + ']: dir=' + g_dir + ' joy=(' + g_joyX + ',' + g_joyY + ') go=' + g_go);
        }
    }
    
    function toggleGo() {
        g_go = document.getElementById('goSwitch').checked ? 1 : 0;
        sendData();
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
    };
</script>
</body>
</html>
)rawliteral";

#endif // WEB_PAGE_H
