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

        /* Telemetry panel pinned to the top-right, offset from the manual
           controls. Displays AttackTopTower bridge-gating counters and a
           live XY plot of the Vive MID position. */
        #statusPanel {
            position: fixed;
            top: 10px;
            right: 10px;
            width: 320px;
            background: #fafafa;
            border: 1px solid #ccc;
            border-radius: 6px;
            padding: 10px;
            font-size: 13px;
            text-align: left;
            z-index: 100;
            box-shadow: 0 2px 6px rgba(0,0,0,0.08);
        }
        #statusPanel h4 { margin: 0 0 6px 0; color: #333; font-size: 14px; }
        .stateRow { display: flex; justify-content: space-between; padding: 2px 0; font-family: monospace; }
        .stateRow .k { color: #555; }
        .stateRow .v { font-weight: bold; color: #111; }
        #statusPanel .graphWrap { margin-top: 8px; }
        #statusPanel svg { width: 100%; height: 240px; background: #fff; border: 1px solid #ddd; }
        .axisLabel { font-size: 10px; fill: #666; font-family: monospace; }
        .gridLine  { stroke: #eee; stroke-width: 1; }
        .axisLine  { stroke: #999; stroke-width: 1; }
        .robotDot  { fill: #2196F3; stroke: #0d47a1; stroke-width: 1; }
        .robotTrail { fill: none; stroke: #2196F3; stroke-width: 1; opacity: 0.4; }
    </style>
</head>
<body>
    <!-- Telemetry panel: AttackTopTower counters + live Vive XY plot.
         Position is fixed top-right so it doesn't overlap the controls. -->
    <div id="statusPanel">
        <h4>Telemetry</h4>
        <div class="stateRow"><span class="k">entryHits</span><span class="v" id="sEntryHits">--</span></div>
        <div class="stateRow"><span class="k">entryFlag</span><span class="v" id="sEntryFlag">--</span></div>
        <div class="stateRow"><span class="k">trigHits</span><span class="v" id="sTrigHits">--</span></div>
        <div class="stateRow"><span class="k">exitHits</span><span class="v" id="sExitHits">--</span></div>
        <div class="stateRow"><span class="k">MID x,y</span><span class="v" id="sXY">--, --</span></div>
        <div class="graphWrap">
            <!-- Plot: x vertical (1000 top → 7000 bottom),
                       y horizontal (6000 left → 2000 right). -->
            <svg id="posGraph" viewBox="0 0 300 240" preserveAspectRatio="none">
                <!-- background grid -->
                <g id="gridX"></g>
                <g id="gridY"></g>
                <!-- axis labels -->
                <text class="axisLabel" x="2"   y="12">x=1000</text>
                <text class="axisLabel" x="2"   y="236">x=7000</text>
                <text class="axisLabel" x="4"   y="232" transform="rotate(-90 4 232)">y=6000</text>
                <text class="axisLabel" x="266" y="232" transform="rotate(-90 266 232)">y=2000</text>
                <!-- trail of recent positions -->
                <polyline id="posTrail" class="robotTrail" points=""/>
                <!-- current robot position -->
                <circle id="posDot" class="robotDot" cx="-10" cy="-10" r="5"/>
            </svg>
        </div>
    </div>

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

    // ---- Telemetry polling ------------------------------------------
    // Plot dimensions match the SVG viewBox (300 x 240).
    var GW = 300, GH = 240;
    // X axis is vertical: vx in [1000, 7000], vx=1000 -> top, vx=7000 -> bottom.
    function vxToScreenY(vx) {
        var v = Math.max(1000, Math.min(7000, vx));
        return ((v - 1000) / (7000 - 1000)) * GH;
    }
    // Y axis is horizontal: vy in [2000, 6000], vy=6000 -> left, vy=2000 -> right.
    function vyToScreenX(vy) {
        var v = Math.max(2000, Math.min(6000, vy));
        return ((6000 - v) / (6000 - 2000)) * GW;
    }

    // Pre-render grid lines (every 1000 mm in robot coords).
    (function drawGrid() {
        var gx = document.getElementById('gridX');
        var gy = document.getElementById('gridY');
        // horizontal lines = constant robot-x
        for (var x = 1000; x <= 7000; x += 1000) {
            var sy = vxToScreenY(x);
            var ln = document.createElementNS('http://www.w3.org/2000/svg', 'line');
            ln.setAttribute('class', 'gridLine');
            ln.setAttribute('x1', 0); ln.setAttribute('x2', GW);
            ln.setAttribute('y1', sy); ln.setAttribute('y2', sy);
            gx.appendChild(ln);
        }
        // vertical lines = constant robot-y
        for (var y = 2000; y <= 6000; y += 1000) {
            var sx = vyToScreenX(y);
            var ln2 = document.createElementNS('http://www.w3.org/2000/svg', 'line');
            ln2.setAttribute('class', 'gridLine');
            ln2.setAttribute('x1', sx); ln2.setAttribute('x2', sx);
            ln2.setAttribute('y1', 0); ln2.setAttribute('y2', GH);
            gy.appendChild(ln2);
        }
    })();

    var trailPts = [];
    var TRAIL_MAX = 60;

    function pollState() {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/state", true);
        xhr.timeout = 800;
        xhr.onload = function() {
            if (xhr.status !== 200) return;
            try {
                var s = JSON.parse(xhr.responseText);
                document.getElementById('sEntryHits').textContent = s.entryHits;
                document.getElementById('sEntryFlag').textContent = s.entryFlag ? 'TRUE' : 'false';
                document.getElementById('sEntryFlag').style.color = s.entryFlag ? '#2e7d32' : '#111';
                document.getElementById('sTrigHits').textContent  = s.trigHits;
                document.getElementById('sExitHits').textContent  = s.exitHits;
                document.getElementById('sXY').textContent =
                    s.x.toFixed(0) + ', ' + s.y.toFixed(0);

                var sx = vyToScreenX(s.y);
                var sy = vxToScreenY(s.x);
                var dot = document.getElementById('posDot');
                dot.setAttribute('cx', sx);
                dot.setAttribute('cy', sy);

                trailPts.push(sx + ',' + sy);
                if (trailPts.length > TRAIL_MAX) trailPts.shift();
                document.getElementById('posTrail')
                        .setAttribute('points', trailPts.join(' '));
            } catch (e) { /* ignore parse errors */ }
        };
        xhr.send();
    }
    setInterval(pollState, 250);
    pollState();
</script>
</body>
</html>
)===";

#endif