const char body[] PROGMEM = R"===(
  <!DOCTYPE html>
  <html>
  <body>
  <h1> Manual or PID Mode <br>
  Mode: <input type ="range" id="modeslider" min="0" max="1" step = "1" value="0">
  <span id="mout">Manual</span>
  <br><br>
  Direction: <input type ="range" id="directionslider" min="-1" max="1" value="1">
  <span id="fout">1</span>
  <br><br>
  Speed: <input type ="range" id="speedslider" min="0" max="110" value="0">
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
  <script>
  //SET UP SLIDER
  var modeslider = document.getElementById("modeslider");
  var directionslider = document.getElementById("directionslider");
  var speedslider = document.getElementById("speedslider");
  var kpslider = document.getElementById("kpslider");
  var kislider = document.getElementById("kislider");
  var kdslider = document.getElementById("kdslider");
  //MODE SLIDER
    modeslider.onchange = function() {
      var xhttp = new XMLHttpRequest();
      var res = "/Mode=" + this.value;
      xhttp.open("GET",res,true);
      xhttp.send();
      document.getElementById("mout").innerHTML = (this.value == 0) ? "Manual" : "PID";
    }
  //DIRECTION SLIDER
    directionslider.onchange = function() {
      var xhttp = new XMLHttpRequest();
      var res = "/motor_direction=" + this.value;
      xhttp.open("GET",res,true);
      xhttp.send();
      document.getElementById("fout").innerHTML = this.value;
    }
  //SPEED SLIDER
    speedslider.onchange = function() {
      var xhttp = new XMLHttpRequest();
      var res = "/motor_speed=" + this.value;
      xhttp.open("GET",res,true);
      xhttp.send();
      document.getElementById("dout").innerHTML = this.value;
    }
  //KP SLIDER
    kpslider.onchange = function() {
      var xhttp = new XMLHttpRequest();
      var res = "/Kp=" + this.value;
      xhttp.open("GET",res,true);
      xhttp.send();
      document.getElementById("kpout").innerHTML = this.value;
    }
  //KI SLIDER
    kislider.onchange = function() {
      var xhttp = new XMLHttpRequest();
      var res = "/Ki=" + this.value;
      xhttp.open("GET",res,true);
      xhttp.send();
      document.getElementById("kiout").innerHTML = this.value;
    }
  //KD SLIDER
    kdslider.onchange = function() {
      var xhttp = new XMLHttpRequest();
      var res = "/Kd=" + this.value;
      xhttp.open("GET",res,true);
      xhttp.send();
      document.getElementById("kdout").innerHTML = this.value;
    }
  </script>
  </body>
  </html>
)===";