const char body[] PROGMEM = R"===(
<!DOCTYPE html>
<html>
<head>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body { font-family: sans-serif; text-align: center; padding: 10px; }
        input[type=range] { width: 120px; height: 4px; cursor: pointer; }
        button { font-size: 20px; padding: 18px 24px; margin: 4px; min-width: 90px; cursor: pointer; }
        #joystick { border: 2px solid #333; background: #eee; border-radius: 50%; touch-action: none; margin: 20px; }
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

    <canvas id="joystick" width="150" height="150"></canvas><br>
    <button onclick="sendDir('S')">&#9646; Force Stop</button>

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
    setupInput("speedslider", "dout", "/motor_speed=");
    setupInput("kpslider", "kpout", "/Kp=");
    setupInput("kislider", "kiout", "/Ki=");
    setupInput("kdslider", "kdout", "/Kd=");

    const canvas = document.getElementById('joystick');
    const ctx = canvas.getContext('2d');
    
    function draw(x, y) {
        ctx.clearRect(0, 0, 150, 150);
        ctx.beginPath();
        ctx.arc(x, y, 20, 0, Math.PI * 2);
        ctx.fillStyle = "#333";
        ctx.fill();
    }
    draw(75, 75);

    function handle(e) {
        const rect = canvas.getBoundingClientRect();
        const x = (e.clientX || e.touches[0].clientX) - rect.left;
        const y = (e.clientY || e.touches[0].clientY) - rect.top;
        draw(x, y);
        const dx = x - 75, dy = y - 75;
        if (Math.abs(dx) > Math.abs(dy)) sendDir(dx > 0 ? 'L' : 'R');
        else sendDir(dy > 0 ? 'B' : 'F');
    }

    canvas.addEventListener('mousedown', (e) => { canvas.onmousemove = handle; });
    window.addEventListener('mouseup', () => { canvas.onmousemove = null; draw(75, 75); sendDir('S'); });
    canvas.addEventListener('touchstart', (e) => { canvas.ontouchmove = handle; });
    canvas.addEventListener('touchend', () => { canvas.ontouchmove = null; draw(75, 75); sendDir('S'); });
</script>
</body>
</html>
)===";