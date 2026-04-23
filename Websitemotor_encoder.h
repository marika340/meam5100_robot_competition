const char body[] PROGMEM = R"===(
  <!DOCTYPE html>
  <html>
  <head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    input[type=range] {
      width: 120px;
      height: 4px;
      cursor: pointer;
    }
    button {
      font-size: 20px;
      padding: 18px 24px;
      margin: 4px;
      min-width: 90px;
      cursor: pointer;
    }
  </style>
  </head>
  <body>
  <h1> Manual or PID Mode <br>
  Mode: <input type ="range" id="modeslider" min="0" max="1" step = "1" value="0">
  <span id="mout">Manual</span>
  <br><br>
  Direction Left: <input type="range" id="dirL" min="-1" max="1" value="1">
  <span id="lout">1</span>
  <br><br>
  Direction Right: <input type="range" id="dirR" min="-1" max="1" value="1">
  <span id="rout">1</span>
  <br><br>
  Speed: <input type ="range" id="speedslider" min="0" max="130" value="0">
  <span id="dout">0</span>
  <br><br>
  Kp: <input type ="range" id="kpslider" min="0" max="20" step="0.1" value="0">
  <span id="kpout">0</span>
  <br><br>
  Ki: <input type ="range" id="kislider" min="0" max="15" step="0.1" value="0">
  <span id="kiout">0</span>
  <br><br>
  Kd: <input type ="range" id="kdslider" min="0" max="20" step="0.1" value="0">
  <span id="kdout">0</span>

  <br><br>
  <button onmousedown="sendDir('F')" onmouseup="sendDir('S')" onmouseleave="sendDir('S')"
          ontouchstart="sendDir('F')" ontouchend="sendDir('S')">&#9650; Forward</button>
  <button onmousedown="sendDir('B')" onmouseup="sendDir('S')" onmouseleave="sendDir('S')"
          ontouchstart="sendDir('B')" ontouchend="sendDir('S')">&#9660; Backward</button>
  
  <button onmousedown="sendDir('R')" onmouseup="sendDir('S')" onmouseleave="sendDir('S')"
          ontouchstart="sendDir('R')" ontouchend="sendDir('S')">&#9668; Left</button>
  
  <button onmousedown="sendDir('L')" onmouseup="sendDir('S')" onmouseleave="sendDir('S')"
          ontouchstart="sendDir('L')" ontouchend="sendDir('S')">&#9658; Right</button>
          
  <button onclick="sendDir('S')">&#9646; Stop</button>

  <button onclick="attack()">&#9646; Attack</button>
  <br>

  <script>
  var modeslider = document.getElementById("modeslider");
  var dirL = document.getElementById("dirL");
  var dirR = document.getElementById("dirR");
  var speedslider = document.getElementById("speedslider");
  var kpslider = document.getElementById("kpslider");
  var kislider = document.getElementById("kislider");
  var kdslider = document.getElementById("kdslider");

  function sendGET(url, retries) {
    retries = retries || 3;
    var xhttp = new XMLHttpRequest();
    xhttp.open("GET", url, true);
    xhttp.timeout = 2000;
    xhttp.onerror = xhttp.ontimeout = function() {
      if (retries > 1) setTimeout(function(){ sendGET(url, retries - 1); }, 300);
    };
    xhttp.send();
  }

  function attack(dir) {
    sendGET("/attack");
  }

  function sendDir(dir) {
    sendGET("/dir=" + dir);
  }

  modeslider.onchange = function() {
    sendGET("/Mode=" + this.value);
    document.getElementById("mout").innerHTML = (this.value == 0) ? "Manual" : "PID";
  }
  dirL.onchange = function() {
    sendGET("/dirLeft=" + this.value);
    document.getElementById("lout").innerHTML = this.value;
  }
  dirR.onchange = function() {
    sendGET("/dirRight=" + this.value);
    document.getElementById("rout").innerHTML = this.value;
  }
  speedslider.onchange = function() {
    sendGET("/motor_speed=" + this.value);
    document.getElementById("dout").innerHTML = this.value;
  }
  kpslider.onchange = function() {
    sendGET("/Kp=" + this.value);
    document.getElementById("kpout").innerHTML = this.value;
  }
  kislider.onchange = function() {
    sendGET("/Ki=" + this.value);
    document.getElementById("kiout").innerHTML = this.value;
  }
  kdslider.onchange = function() {
    sendGET("/Kd=" + this.value);
    document.getElementById("kdout").innerHTML = this.value;
  }
  </script>
  </body>
  </html>
)===";