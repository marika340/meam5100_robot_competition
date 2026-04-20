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
    </style>
</head>
<body>
    <h1>Team 8 Car</h1>
    <button id="automode" onclick="toggleAuto()">Auto Mode: OFF</button>
    <br><br>

    Speed: <input type="range" id="speedslider" min="0" max="130" value="0">
    <span id="dout">0</span><br><br>
    
    Kp: <input type="range" id="kpslider" min="0" max="20" step="0.1" value="0">
    <span id="kpout">0</span><br><br>
    
    Ki: <input type="range" id="kislider" min="0" max="15" step="0.1" value="0">
    <span id="kiout">0</span><br><br>
    
    Kd: <input type="range" id="kdslider" min="0" max="20" step="0.1" value="0">
    <span id="kdout">0</span><br><br>

    <div>
        <button class="dir-btn" onmousedown="sendDir('F')" onmouseup="sendDir('S')" ontouchstart="sendDir('F')" ontouchend="sendDir('S')">Fwd</button><br>
        <button class="dir-btn" onmousedown="sendDir('R')" onmouseup="sendDir('S')" ontouchstart="sendDir('R')" ontouchend="sendDir('S')">Left</button>
        <button class="dir-btn" onmousedown="sendDir('L')" onmouseup="sendDir('S')" ontouchstart="sendDir('L')" ontouchend="sendDir('S')">Right</button><br>
        <button class="dir-btn" onmousedown="sendDir('B')" onmouseup="sendDir('S')" ontouchstart="sendDir('B')" ontouchend="sendDir('S')">Back</button>
    </div>
    <br>
    <button onclick="sendDir('S')">&#9646; Force Stop</button>

<script>
    // FIXED: Correct XHR implementation
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
        // Using oninput for immediate updates while dragging
        el.oninput = function() { 
            document.getElementById(outId).innerHTML = this.value;
            sendGET(endpoint + this.value); 
        };
    }
    
    setupInput("speedslider", "dout", "/motor_speed=");
    setupInput("kpslider", "kpout", "/Kp=");
    setupInput("kislider", "kiout", "/Ki=");
    setupInput("kdslider", "kdout", "/Kd=");
</script>
</body>
</html>
)===";