#ifndef PIDANDTOF_WEB_H
#define PIDANDTOF_WEB_H

const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: sans-serif; text-align: center; padding: 10px; touch-action: manipulation; }
        input[type=range] { width: 80%; height: 30px; cursor: pointer; }
        button { font-size: 18px; padding: 15px 25px; margin: 5px; cursor: pointer; }
        .dir-btn { width: 100px; height: 50px; }
        .active { background-color: #4CAF50; color: white; }
        h3 { margin: 10px 0 4px 0; color: #555; }
        hr { margin: 12px 0; border: none; border-top: 1px solid #ccc; }
    </style>
</head>
<body>
    <h1>Team 8 Car</h1>
    <button id="automode" onclick="toggleAuto()">Auto Mode: OFF</button>
    <br><br>

    <hr>
    <h3>Drive</h3>
    Speed: <input type="range" id="speedslider" min="0" max="130" value="0">
    <span id="dout">0</span><br><br>

    <div>
        <button class="dir-btn" onmousedown="sendDir('F')" onmouseup="sendDir('S')" ontouchstart="sendDir('F')" ontouchend="sendDir('S')">Fwd</button><br>
        <button class="dir-btn" onmousedown="sendDir('R')" onmouseup="sendDir('S')" ontouchstart="sendDir('R')" ontouchend="sendDir('S')">Left</button>
        <button class="dir-btn" onmousedown="sendDir('L')" onmouseup="sendDir('S')" ontouchstart="sendDir('L')" ontouchend="sendDir('S')">Right</button><br>
        <button class="dir-btn" onmousedown="sendDir('B')" onmouseup="sendDir('S')" ontouchstart="sendDir('B')" ontouchend="sendDir('S')">Back</button>
    </div>
    <br>
    <button onclick="sendDir('S')">&#9646; Force Stop</button>

    <hr>
    <h3>PID Tuning (Drive)</h3>
    Kp: <input type="range" id="kpslider" min="0" max="20" step="0.1" value="0">
    <span id="kpout">0</span><br><br>

    Ki: <input type="range" id="kislider" min="0" max="15" step="0.1" value="0">
    <span id="kiout">0</span><br><br>

    Kd: <input type="range" id="kdslider" min="0" max="20" step="0.1" value="0">
    <span id="kdout">0</span><br><br>

    <hr>
    <h3>Wall Following Tuning</h3>
    wf_Kp: <input type="range" id="wfkpslider" min="0" max="5" step="0.05" value="0.7">
    <span id="wfkpout">0.7</span><br><br>

    wf_Kd: <input type="range" id="wfkdslider" min="0" max="10" step="0.05" value="1.2">
    <span id="wfkdout">1.2</span><br><br>

    sharpTurnOffset: <input type="range" id="stoSlider" min="0" max="255" step="1" value="90">
    <span id="stoOut">90</span><br><br>

<script>
    function sendGET(url) {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", url, true);
        xhr.send();
    }

    let auto = false;
    function toggleAuto() {
        auto = !auto;
        document.getElementById("automode").innerHTML = "Auto Mode: " + (auto ? "ON" : "OFF");
        document.getElementById("automode").className = auto ? "active" : "";
        sendGET("/Auto=" + (auto ? "1" : "0"));
    }

    function sendDir(dir) { sendGET("/dir=" + dir); }

    function setupInput(id, outId, endpoint) {
        var el = document.getElementById(id);
        el.oninput = function() {
            document.getElementById(outId).innerHTML = this.value;
            sendGET(endpoint + this.value);
        };
    }

    setupInput("speedslider",  "dout",    "/motor_speed=");
    setupInput("kpslider",     "kpout",   "/Kp=");
    setupInput("kislider",     "kiout",   "/Ki=");
    setupInput("kdslider",     "kdout",   "/Kd=");
    setupInput("wfkpslider",   "wfkpout", "/wf_Kp=");
    setupInput("wfkdslider",   "wfkdout", "/wf_Kd=");
    setupInput("stoSlider",    "stoOut",  "/sharpTurn=");
</script>
</body>
</html>
)===";

#endif