#ifndef PIDANDTOF_WEB_H
#define PIDANDTOF_WEB_H

const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: sans-serif; text-align: center; padding: 10px; touch-action: manipulation; }
        .control-row { display: flex; align-items: flex-start; justify-content: center; gap: 20px; flex-wrap: wrap; margin-bottom: 20px; }
        .side-col { display: flex; flex-direction: column; align-items: center; }
        .slider-group { text-align: left; width: 220px; margin-bottom: 8px; font-size: 14px; }
        input[type=range] { width: 100%; height: 20px; cursor: pointer; }
        button { font-size: 16px; padding: 10px 15px; margin: 5px; cursor: pointer; width: 150px; }
        .dir-btn { width: 70px; height: 50px; margin: 2px; }
        .active { background-color: #4CAF50; color: white; }
        h3 { margin: 15px 0 10px 0; color: #555; }
        hr { margin: 20px 0; border: none; border-top: 1px solid #ccc; }
        .d-pad { display: inline-block; width: 220px; }
        .attack-btn { background-color: #ff4444; color: white; height: 60px; width: 100%; font-weight: bold; margin-top: 10px; }
    </style>
</head>
<body>
    <h1>Team 8 Car</h1>
    
    <h3>Manual Drive & Auto Tuning</h3>
    <div class="control-row">
        <div class="side-col">
            <div class="d-pad">
                <button class="dir-btn" onmousedown="sendDir('F')" onmouseup="sendDir('S')" ontouchstart="sendDir('F')" ontouchend="sendDir('S')">Fwd</button><br>
                <button class="dir-btn" onmousedown="sendDir('R')" onmouseup="sendDir('S')" ontouchstart="sendDir('R')" ontouchend="sendDir('S')">Left</button>
                <button class="dir-btn" onmousedown="sendDir('L')" onmouseup="sendDir('S')" ontouchstart="sendDir('L')" ontouchend="sendDir('S')">Right</button><br>
                <button class="dir-btn" onmousedown="sendDir('B')" onmouseup="sendDir('S')" ontouchstart="sendDir('B')" ontouchend="sendDir('S')">Back</button>
            </div>
            <button onclick="sendDir('S')" style="background-color: #f44336; color: white;">Force Stop</button>
            <button class="attack-btn" onclick="sendGET('/attack')">ATTACK ARM</button>
        </div>
        <div class="side-col">
            <div class="slider-group">RPM: <input type="range" id="rpmSlider" min="0" max="130" value="85"><span id="rpmOut">85</span></div>
            <div class="slider-group">Kp: <input type="range" id="kpSlider" min="0" max="5" step="0.1" value="1.4"><span id="kpOut">1.4</span></div>
            <div class="slider-group">Ki: <input type="range" id="kiSlider" min="0" max="5" step="0.1" value="1.0"><span id="kiOut">1.0</span></div>
            <div class="slider-group">Kd: <input type="range" id="kdSlider" min="0" max="5" step="0.1" value="0"><span id="kdOut">0</span></div>
        </div>
    </div>

    <hr>
    <h3>Modes & Wall Tuning</h3>
    <div class="control-row">
        <div class="side-col">
            <button id="m0" onclick="setMode(0)">Stop / Manual</button>
            <button id="m1" onclick="setMode(1)">Wall Follow</button>
            <button id="m2" onclick="setMode(2)">Centering</button>
            <button id="m3" onclick="setMode(3)" style="background-color:#9c27b0; color:white;">Low Tower</button>
            <button id="m4" onclick="setMode(4)" style="background-color:#e91e63; color:white;">Attack Nexus</button>
            <button id="m5" onclick="setMode(5)" style="background-color:#ff9800; color:white;">Attack Top Tower</button>
            <button id="straightButton" onclick="sendGET('/straight=12')" style="background-color:#2196F3; color:white;">Straight Movement</button>
        </div>
        <div class="side-col">
            <div class="slider-group">wf_Kp: <input type="range" id="wfkpslider" min="0" max="5" step="0.05" value="0.7"><span id="wfkpout">0.7</span></div>
            <div class="slider-group">wf_Kd: <input type="range" id="wfkdslider" min="0" max="10" step="0.05" value="1.2"><span id="wfkdout">1.2</span></div>
            <div class="slider-group">Sharp Offset: <input type="range" id="stoSlider" min="0" max="255" value="90"><span id="stoOut">90</span></div>
        </div>
    </div>

    <div class="side-col">
    <h3>Vive Navigation</h3>
    <div class="slider-group">
        Vive X: <input type="number" id="vxInput" placeholder="e.g. 4500" style="width: 80px;">
    </div>
    <div class="slider-group">
        Vive Y: <input type="number" id="vyInput" placeholder="e.g. 3200" style="width: 80px;">
    </div>
    <button id="m10" onclick="sendVive()">Navigate to Vive</button>
</div>

<script>
    function sendGET(url) { var xhr = new XMLHttpRequest(); xhr.open("GET", url, true); xhr.send(); }
    
    function setMode(m) {
        document.querySelectorAll('button').forEach(btn => btn.classList.remove('active'));
        document.getElementById('m' + m).classList.add('active');
        sendGET("/mode=" + m);
    }
    
    function sendDir(dir) { sendGET("/dir=" + dir); }
    
    function setupInput(id, outId, endpoint) {
        var el = document.getElementById(id);
        el.oninput = function() { document.getElementById(outId).innerHTML = this.value; sendGET(endpoint + this.value); };
    }

    function sendVive() {
    var x = document.getElementById('vxInput').value;
    var y = document.getElementById('vyInput').value;
    if(x && y) {
        // Sends /goto_vive?x=XXXX&y=YYYY
        sendGET("/goto_vive?x=" + x + "&y=" + y);
        // Also switch the mode visually to your new Nav mode (e.g. Mode 10)
        setMode(10); 
    } else {
        alert("Please enter both X and Y Vive coordinates");
        }
    }
    
    // Aligned with backend handlers
    setupInput("rpmSlider", "rpmOut", "/motor_speed=");
    setupInput("kpSlider",  "kpOut",  "/Kp=");
    setupInput("kiSlider",  "kiOut",  "/Ki=");
    setupInput("kdSlider",  "kdOut",  "/Kd=");
    setupInput("wfkpslider", "wfkpout", "/wf_Kp=");
    setupInput("wfkdslider", "wfkdout", "/wf_Kd=");
    setupInput("stoSlider",  "stoOut",  "/sharpTurn=");
</script>
</body>
</html>
)===";

#endif