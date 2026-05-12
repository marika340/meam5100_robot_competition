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
<!--
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
 -->
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
    <div id="viveRealCoords" style="font-family:monospace; font-size:12px; color:#555; margin-top:6px; min-height:1.4em;">
        Real X: --, Real Y: --
    </div>
</div>

    <hr>
    <h3>Vive Calibration</h3>
    <p style="font-size:12px; color:#666; margin:0 0 8px 0;">
        Position the robot at each labeled field point, read Vive coords (e.g. from Serial or telemetry), enter below, then click Calibrate.
        Real coords (inches from center of low tower) are fixed.
    </p>
    <table style="font-size:13px; border-collapse:collapse; margin:0 auto;">
        <tr style="background:#eee;">
            <th style="padding:4px 10px;">Point</th>
            <th style="padding:4px 10px;">Real X (in)</th>
            <th style="padding:4px 10px;">Real Y (in)</th>
            <th style="padding:4px 10px;">Vive X</th>
            <th style="padding:4px 10px;">Vive Y</th>
        </tr>
        <tr>
            <td style="padding:3px 10px;text-align:center;font-weight:bold;">2</td>
            <td style="padding:3px 10px;text-align:center;">-24</td>
            <td style="padding:3px 10px;text-align:center;">0</td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p2_x" step="0.5" style="width:72px;" placeholder="e.g. 4745"></td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p2_y" step="0.5" style="width:72px;" placeholder="e.g. 4526"></td>
        </tr>
        <tr style="background:#f9f9f9;">
            <td style="padding:3px 10px;text-align:center;font-weight:bold;">3</td>
            <td style="padding:3px 10px;text-align:center;">-5.75</td>
            <td style="padding:3px 10px;text-align:center;">0</td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p3_x" step="0.5" style="width:72px;" placeholder="e.g. 4058"></td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p3_y" step="0.5" style="width:72px;" placeholder="e.g. 4511"></td>
        </tr>
        <tr>
            <td style="padding:3px 10px;text-align:center;font-weight:bold;">4</td>
            <td style="padding:3px 10px;text-align:center;">5.75</td>
            <td style="padding:3px 10px;text-align:center;">0</td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p4_x" step="0.5" style="width:72px;" placeholder="e.g. 3484"></td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p4_y" step="0.5" style="width:72px;" placeholder="e.g. 4534"></td>
        </tr>
        <tr style="background:#f9f9f9;">
            <td style="padding:3px 10px;text-align:center;font-weight:bold;">5</td>
            <td style="padding:3px 10px;text-align:center;">24</td>
            <td style="padding:3px 10px;text-align:center;">0</td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p5_x" step="0.5" style="width:72px;" placeholder="e.g. 2724"></td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p5_y" step="0.5" style="width:72px;" placeholder="e.g. 4641"></td>
        </tr>
        <tr>
            <td style="padding:3px 10px;text-align:center;font-weight:bold;">7</td>
            <td style="padding:3px 10px;text-align:center;">0</td>
            <td style="padding:3px 10px;text-align:center;">17.75</td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p7_x" step="0.5" style="width:72px;" placeholder="e.g. 3727"></td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p7_y" step="0.5" style="width:72px;" placeholder="e.g. 5216"></td>
        </tr>
        <tr style="background:#f9f9f9;">
            <td style="padding:3px 10px;text-align:center;font-weight:bold;">8</td>
            <td style="padding:3px 10px;text-align:center;">0</td>
            <td style="padding:3px 10px;text-align:center;">-15.75</td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p8_x" step="0.5" style="width:72px;" placeholder="e.g. 3622"></td>
            <td style="padding:3px 4px;"><input type="number" id="cal_p8_y" step="0.5" style="width:72px;" placeholder="e.g. 3856"></td>
        </tr>
    </table>
    <div style="margin-top:10px;">
        <button onclick="runCalibration()" style="background-color:#4CAF50;color:white;width:200px;font-weight:bold;">Run Calibration</button>
        <button onclick="clearCalib()" style="background-color:#9e9e9e;color:white;width:100px;">Clear</button>
    </div>
    <div id="calibResult" style="font-family:monospace;font-size:12px;margin-top:10px;padding:8px;border-radius:4px;display:none;text-align:left;max-width:500px;margin-left:auto;margin-right:auto;"></div>

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

    // ---- Calibration coefficient store (mirrors NavigationTools defaults) --
    var coeffs = {
        mx_xr: -0.02326407517, my_xr:  0.001761464417, c_xr:   78.568437,
        mx_yr:  0.00132269917, my_yr:  0.0243494819,   c_yr: -115.5075201
    };

    // Recompute and display real X/Y from whatever is in the Vive Nav inputs.
    function updateViveRealDisplay() {
        var xv = parseFloat(document.getElementById('vxInput').value);
        var yv = parseFloat(document.getElementById('vyInput').value);
        var el = document.getElementById('viveRealCoords');
        if (isNaN(xv) || isNaN(yv) || document.getElementById('vxInput').value === '' || document.getElementById('vyInput').value === '') {
            el.textContent = 'Real X: --, Real Y: --';
            return;
        }
        var rx = coeffs.mx_xr * xv + coeffs.my_xr * yv + coeffs.c_xr;
        var ry = coeffs.mx_yr * xv + coeffs.my_yr * yv + coeffs.c_yr;
        el.textContent = 'Real X: ' + rx.toFixed(2) + ' in,  Real Y: ' + ry.toFixed(2) + ' in';
    }

    // Fetch current coefficients from the robot (runs on page load).
    function fetchCoeffs() {
        var xhr = new XMLHttpRequest();
        xhr.open('GET', '/coeff', true);
        xhr.timeout = 1500;
        xhr.onload = function() {
            if (xhr.status === 200) {
                try { coeffs = JSON.parse(xhr.responseText); updateViveRealDisplay(); } catch(e) {}
            }
        };
        xhr.send();
    }

    function sendVive() {
        var x = document.getElementById('vxInput').value;
        var y = document.getElementById('vyInput').value;
        if (x && y) {
            sendGET("/goto_vive?x=" + x + "&y=" + y);
            setMode(10);
        } else {
            alert("Please enter both X and Y Vive coordinates");
        }
    }

    // Update real-coord display whenever either Vive Nav input changes.
    document.addEventListener('DOMContentLoaded', function() {
        document.getElementById('vxInput').addEventListener('input', updateViveRealDisplay);
        document.getElementById('vyInput').addEventListener('input', updateViveRealDisplay);
        fetchCoeffs();
    });

    // ---- Vive Calibration -------------------------------------------
    // Known real coords (in) for each point, ordered 2,3,4,5,7,8.
    var CAL_REAL_X = [-24, -5.75, 5.75, 24, 0, 0];
    var CAL_REAL_Y = [0, 0, 0, 0, 17.75, -15.75];
    var CAL_IDS    = ['p2','p3','p4','p5','p7','p8'];

    function coeffRow(label, val) {
        return '<tr><td style="padding:1px 8px;color:#555;">' + label + '</td>' +
               '<td style="padding:1px 8px;font-weight:bold;">' + val.toFixed(9) + '</td></tr>';
    }

    function showCoeffTable(r) {
        return '<table style="border-collapse:collapse;font-size:12px;font-family:monospace;margin:6px auto;">' +
            '<tr><th colspan="2" style="text-align:left;padding:2px 8px;border-bottom:1px solid #aaa;">X_real = mx_xr·Xv + my_xr·Yv + c_xr</th></tr>' +
            coeffRow('mx_xr', r.mx_xr) + coeffRow('my_xr', r.my_xr) + coeffRow('c_xr',  r.c_xr) +
            '<tr><th colspan="2" style="text-align:left;padding:4px 8px 2px;border-bottom:1px solid #aaa;border-top:1px solid #aaa;">Y_real = mx_yr·Xv + my_yr·Yv + c_yr</th></tr>' +
            coeffRow('mx_yr', r.mx_yr) + coeffRow('my_yr', r.my_yr) + coeffRow('c_yr',  r.c_yr) +
            '</table>';
    }

    function runCalibration() {
        var vals = [];
        for (var i = 0; i < CAL_IDS.length; i++) {
            var xv = document.getElementById('cal_' + CAL_IDS[i] + '_x').value;
            var yv = document.getElementById('cal_' + CAL_IDS[i] + '_y').value;
            if (xv === '' || yv === '') {
                alert('Please fill in Vive X and Y for all 6 points (2,3,4,5,7,8).');
                return;
            }
            vals.push(parseFloat(xv));
            vals.push(parseFloat(yv));
        }
        var url = '/calibrate=' + vals.join(',');
        var res = document.getElementById('calibResult');
        res.style.display = 'block';
        res.style.background = '#fffde7';
        res.style.color = '#333';
        res.innerHTML = 'Running calibration...';
        var xhr = new XMLHttpRequest();
        xhr.open('GET', url, true);
        xhr.timeout = 4000;
        xhr.onload = function() {
            if (xhr.status === 200) {
                try {
                    var r = JSON.parse(xhr.responseText);
                    if (r.ok) {
                        // Store new coefficients and refresh the Vive Nav display
                        coeffs = {mx_xr: r.mx_xr, my_xr: r.my_xr, c_xr: r.c_xr,
                                  mx_yr: r.mx_yr, my_yr: r.my_yr, c_yr: r.c_yr};
                        updateViveRealDisplay();
                        res.style.background = '#e8f5e9';
                        res.style.color = '#1b5e20';
                        res.innerHTML = '<strong>Calibration OK — coefficients updated on robot:</strong>' +
                                        showCoeffTable(r);
                    } else {
                        res.style.background = '#ffebee';
                        res.style.color = '#b71c1c';
                        res.innerHTML = 'Calibration FAILED: ' + (r.err || 'unknown error') +
                                        '. Check that Vive values are spread across all 6 distinct points.';
                    }
                } catch(e) {
                    res.style.background = '#ffebee';
                    res.style.color = '#b71c1c';
                    res.innerHTML = 'Could not parse response. Is the robot connected?';
                }
            } else {
                res.style.background = '#ffebee';
                res.style.color = '#b71c1c';
                res.innerHTML = 'HTTP error ' + xhr.status;
            }
        };
        xhr.ontimeout = function() {
            res.style.background = '#ffebee';
            res.style.color = '#b71c1c';
            res.innerHTML = 'Request timed out. Is the robot connected?';
        };
        xhr.onerror = function() {
            res.style.background = '#ffebee';
            res.style.color = '#b71c1c';
            res.innerHTML = 'Network error. Is the robot connected?';
        };
        xhr.send();
    }

    function clearCalib() {
        for (var i = 0; i < CAL_IDS.length; i++) {
            document.getElementById('cal_' + CAL_IDS[i] + '_x').value = '';
            document.getElementById('cal_' + CAL_IDS[i] + '_y').value = '';
        }
        var res = document.getElementById('calibResult');
        res.style.display = 'none';
        res.innerHTML = '';
    }
    // -----------------------------------------------------------------

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