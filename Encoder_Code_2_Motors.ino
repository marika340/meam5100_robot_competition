#include <WiFi.h>
#include <WiFiUdp.h>
#include "html510.h"
#include "Websitemotor_encoder.h"
#include <Encoder.h>

//SET UP WIFI STUFF
HTML510Server h(80);
//SET UP SSID AT GM LAB
//const char* ssid = "TP-Link_8A8C";
//const char* password = "12488674";
//SET UP SSID AT ANH'S HOME
//const char* ssid = "TheSpot";
//const char* password = "D0esntSp0tLiveHere?";
//SET UP SSID AT ANH'S PHONE
const char* ssid = "iPhone";
const char* password = "anhduong";
//WIFI IP ADDRESSES
// IPAddress myIP (172,20,10,5); //Anh's IP FOR PHONE ONLY!!!
// IPAddress gateway(172,20,10,1); //Router IP
// IPAddress subnet(255,255,255,240); //Subnet IP
IPAddress myIP (192,168,1,105); //Anh's IP
IPAddress gateway(192,168,1,1); //Router IP
IPAddress subnet(255,255,255,0); //Subnet IP
//DECLARE UDP (USED SAMPLE FROM LECTURE)
//WiFiUDP UDPTestServer;
//int port = 8888; // any UDP port# up to 65535 // but higher is safer > 1023

//DEFINING ALL PINS
//Encoder myEnc[] = { {4, 7}, {18, 19} }; //Encoder object
Encoder myEnc[] = { {35,36}, {34,33} }; //Encoder object SWAP LOGIC TO MATCH DIRECTION
int Hpin[] = {1, 2};             // PWM Pins
int Hpin_dir1[] = {42, 40};       // Direction Pin 1
int Hpin_dir2[] = {41, 39};       // Direction Pin 2
int enc_pin_a[] = {34, 36};       // Encoder A
int enc_pin_b[] = {33, 35};       // Encoder B

//Encoder myEnc[] = {{enc_pin_a[0],enc_pin_a[1]},{enc_pin_b[0],enc_pin_b[1]}};

// PID & State Arrays
float motor_speed[] = {0, 0};
int motor_direction[] = {1, 1};
float integral[] = {0, 0};
float previous_error[] = {0, 0};
long previous_count[] = {0, 0};
long prev_manual_count[] = {0, 0};
unsigned long last_pid_time = 0;
unsigned long last_time = 0;
float rpm_new = 0.0;

int encoder_slot = 12; //COUNT 12 SLOTS FOR THE NEW MOTOR
int frequency = 500; //PICK A FREQUENCY FOR A MOTOR, WANT TO PICK A FREQUENCY THAT DOES NOT PRODUCE NOISE
int duty_cycle = 50; //PLACE HOLDER VALUE
int period = 1000.0/(2*frequency); //PERIOD in ms, want blinking at half cycle
int resolution_bit = 14; //DEPENDING ON THE NUMBER OF BITS, USE BIT = 14 TO MEET THE REQUIREMENT OF > 39 kHz
int resolution = ((1<<resolution_bit)-1); //GET MAX RESOLUTION BASED ON BITS
int pwm_channels[] = {0, 1};     // ESP32 PWM channels

//SET UP PID CONTROL VARIABLES - TUNING CONSTANT
float Kp = 1, Ki = 0, Kd = 0; //CHANGE FOR TUNING
float rpm_desired = 0.0; //CHANGE IF WANT
int gear_ratio = 34; //NEW MOTORS
float count_per_revolution = (12.0*4*gear_ratio); //12 CPR * 4 EDGES
bool pid_enable = false;

//LOOP FOR ANALOG WRITE
void ledcAnalogWrite(uint8_t pin, uint32_t value, uint32_t value_max) {
  uint32_t duty_cycle = map(value,0,value_max,0,resolution); //SET UP DUTY CYCLE
  // Set the PWM duty cycle of pin as a proportion of resolution bits
  ledcWrite(pin,duty_cycle); //DUTY CYCLE TO PWM
}

//MOTOR FUNCTION
void motor(int i) {
  uint32_t pwm_value = (uint32_t)constrain(motor_speed[i],0,resolution);
  if (motor_direction[i] == -1) { //REVERSE DIRECTION
    digitalWrite(Hpin_dir1[i],HIGH);
    digitalWrite(Hpin_dir2[i],LOW);
    ledcWrite(Hpin[i],pwm_value); //WRITE PWM TO PIN
  } else if (motor_direction[i] == 1) { //FORWARD DIRECTION
    digitalWrite(Hpin_dir1[i],LOW);
    digitalWrite(Hpin_dir2[i],HIGH);
    ledcWrite(Hpin[i],pwm_value); //WRITE PWM TO PIN
  } else {//BRAKING
    digitalWrite(Hpin_dir1[i],LOW); //CHANGE HIGH HIGH LOGIC FOR OUR PART
    digitalWrite(Hpin_dir2[i],LOW);
    ledcWrite(Hpin[i],0); //WRITE PWM TO PIN
  }
  //Serial.printf("Motor speed: %d Direction: %d\n",motor_speed, motor_direction);
}

//PID FUNCTION
void PIDcontrol() {
  //TIME DERIVATIVE DT
  unsigned long current = millis(); //SET UP CURRENT TIME 
  float dt = (current - last_pid_time)/1000.0; //FIND DELTA TIME IN MILLISECONDS
  float current_rpms[2] = {0, 0};
  //ESTABLISH CURRENT COUNT
  //INSTALL ENCODER COUNT LIBRARY BY Paul Stoffregen (v 1.4.4)
  if (dt >= 0.100) {//RUN PID AT 500 Ms
    //GET VALUE FROM CURRENT RPM
    for (int i = 0; i < 2; i++) { 
      long current_count = myEnc[i].read(); //USING ENCODER COUNTING LIBRARY 
      float current_rpm = abs(((current_count-previous_count[i])/(float)count_per_revolution)*(60.0/dt));
      current_rpms[i] = current_rpm;
      if (motor_direction[i] == 0) {
       motor_speed[i] = 0;
       integral[i] = 0;
       motor(i);
      continue;
      }
      float individual_target = abs(rpm_desired);
      float error = individual_target - current_rpm; //CALCULATE ERROR
      float output = Kp*error; // P OUTPUT
      integral[i] = constrain(integral[i]+(error * dt), -50, 50); //Integral tuning
      float output_Ki = Ki*integral[i]; //I OUTPUT
      float derivative = (error - previous_error[i]) / dt; //derivative
      float output_Kd = Kd*derivative; //D OUTPUT
      float control_output = output + output_Ki + output_Kd;
      motor_speed[i] = constrain(control_output * (resolution / 130.0), 0, resolution);
      Serial.printf("M%d | Target:%.1f | Curr:%.1f | Error:%.1f | Out:%.1f | Dir:%d | PWM:%.0f \n",
      i, individual_target, current_rpm, error, control_output, motor_direction[i], motor_speed[i]);
      // if (control_output > 0) {
      //   motor_direction[i] = 1;
      //   } else if (control_output < 0) {
      //   motor_direction[i] = -1;
      //   //motor_speed[i] = constrain(abs(control_output) * (resolution / 110.0), 0, resolution);
      //   } else {
      //   motor_direction[i] = 0;
      //   motor_speed[i] = 0;
      //   }
      //motor(i);
      Serial.printf("MOTOR %d | Target RPM: %.1f | Current RPM: %.1f | Dir: %d | PWM: %.1f\n", i, (rpm_desired * motor_direction[i]), current_rpm, motor_direction[i], motor_speed[i]);
      previous_count[i] = current_count;
      previous_error[i] = error;
    }
    //SYNC THE MOTORS
    if (motor_direction[0] != 0 && motor_direction[1] != 0) {
      // Use the values we just stored in current_rpms
      float sync_error = current_rpms[0] - current_rpms[1]; 
      float sync_gain = 0.005; 
      motor_speed[0] = constrain(motor_speed[0] - sync_gain * (resolution / 130.0) * sync_error, 0, resolution);
      motor_speed[1] = constrain(motor_speed[1] + sync_gain * (resolution / 130.0) * sync_error, 0, resolution);
      // FIX INTEGRAL
      float integral_diff = integral[0] - integral[1];
      integral[0] -= 0.1 * integral_diff;
      integral[1] += 0.1 * integral_diff;
    }
    motor(0); 
    motor(1);
    last_pid_time = current; //UPDATE TIME
  }
}

//ALL HANDLERS
void handleDir() {
  String dir = h.getText();  // returns "F", "B", "L", "R", or "S"
  if (dir == "F") {
    motor_direction[0] = 1;  motor_direction[1] = 1;
  } else if (dir == "B") {
    motor_direction[0] = -1; motor_direction[1] = -1;
  } else if (dir == "L") {
    motor_direction[0] = -1; motor_direction[1] = 1;
  } else if (dir == "R") {
    motor_direction[0] = 1;  motor_direction[1] = -1;
  } else if (dir == "S") {
    motor_direction[0] = 0;  motor_direction[1] = 0;
  }
  motor(0); motor(1);
  h.sendhtml(body);
}

void handleSpeed() {
  rpm_new = h.getVal();
  if (pid_enable) {
  rpm_desired = constrain(rpm_new,-130,130);
  for(int i=0; i<2; i++) { integral[i] = 0; previous_error[i] = 0; }
  Serial.printf("RPM set to: %.1f, PID enabled\n", rpm_desired);
  } else {
    for(int i=0; i<2; i++) {
      motor_speed[i] = map(constrain(abs(rpm_new), 0, 130), 0, 130, 0, resolution);
      motor(i);
      }
    }
  h.sendhtml(body);
}

void handleRoot() {
  h.sendhtml(body); //HTML CODE SEPARATE
}

// HANDLER FOR LEFT MOTOR (Index 0)
void handleDirectionLeft() {
  int dir = h.getVal();
  motor_direction[0] = constrain(dir, -1, 1);
  motor(0); // Update hardware for Motor 0
  h.sendhtml(body);
}

// HANDLER FOR RIGHT MOTOR (Index 1)
void handleDirectionRight() {
  int dir = h.getVal();
  motor_direction[1] = constrain(dir, -1, 1);
  motor(1); // Update hardware for Motor 1
  h.sendhtml(body);
}

void handleKp() {
  Kp = h.getVal();
  for(int i = 0; i < 2; i++) {
    integral[i] = 0;
    previous_error[i] = 0;
  }
  Serial.printf("Kp: %.2f\n",Kp);
  h.sendhtml(body);
}

void handleKi() {
  Ki = h.getVal();
  for(int i = 0; i < 2; i++) {
    integral[i] = 0;
    previous_error[i] = 0;
  }
  Serial.printf("Ki: %.2f\n",Ki);
  h.sendhtml(body);
}

void handleKd() {
  Kd = h.getVal();
  for(int i = 0; i < 2; i++) {
    integral[i] = 0;
    previous_error[i] = 0;
  }
  Serial.printf("Kd: %.2f\n",Kd);
  h.sendhtml(body);
}

void handleMode() {
  int mode = h.getVal();
  pid_enable = (mode == 1);
  rpm_desired = 0;
  //RESET ENCODER
  for(int i = 0; i < 2; i++) {
    myEnc[i].write(0);        // Reset each encoder individually
    prev_manual_count[i] = 0; 
    previous_count[i] = 0;
    integral[i] = 0;
    previous_error[i] = 0;
    motor_speed[i] = 0;
    motor(i);                 // Tell the specific motor to stop
  }
  Serial.printf("Mode: %d\n",mode);
  h.sendhtml(body);
}

void setup() {
  for (int i = 0; i < 2; i++) {
    pinMode(Hpin_dir1[i], OUTPUT);
    pinMode(Hpin_dir2[i], OUTPUT);
    pinMode(Hpin[i], OUTPUT);
    pinMode(enc_pin_a[i], INPUT_PULLUP);
    pinMode(enc_pin_b[i], INPUT_PULLUP);
    
    // Attach separate PWM channels
    ledcAttachChannel(Hpin[i], frequency, resolution_bit, pwm_channels[i]);
  }

  //SET UP WIFI
  Serial.begin(115200); //FOR SERIAL MONITOR WINDOW TO DEBUG
  //WiFi.config(myIP, gateway, subnet);
  WiFi.setSleep(false); 
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) {//CHECK WIFI STATUS
    delay(500); //WAIT
    Serial.print("."); //PRINT
  }
  Serial.println("WiFi connected"); //PRINT
  Serial.println("IP address:");
  Serial.println(WiFi.localIP());
  //UDPTestServer.begin(port); 

  //SET UP THE HANDLERS
  h.begin();
  h.attachHandler("/motor_speed=", handleSpeed);
  h.attachHandler("/dirLeft=", handleDirectionLeft);
  h.attachHandler("/dirRight=", handleDirectionRight);
  //h.attachHandler("/RPM=", handleRPM);
  h.attachHandler("/Mode=", handleMode);
  h.attachHandler("/Kp=", handleKp);
  h.attachHandler("/Ki=", handleKi);
  h.attachHandler("/Kd=", handleKd);
  h.attachHandler("/dir=", handleDir);
  h.attachHandler("/",handleRoot);

  last_pid_time = millis();
  last_time = millis();
}

void manualMode() {
  //UPDATE ENCODER VALUE
  if(millis() - last_time >= 100) { //START A TIMER IN MS
    long now = millis();
    float dt_manual = (now - last_time)/1000.0;
    for (int i = 0; i < 2; i++) {
      long current_manual_count = myEnc[i].read();
      float count = current_manual_count - prev_manual_count[i]; // Use the array!
      float rpm = ((count/count_per_revolution)*(60.0/dt_manual));
      // IMPROVED PRINT STATEMENT:
      Serial.printf("MOTOR %d | Manual RPM: %.1f | Dir: %d  ", i, rpm, motor_direction[i]);
      prev_manual_count[i] = current_manual_count;
    }
    Serial.println();
    last_time = now;
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  h.serve(); //WEB REQUEST
  //IF PID IS ON
  if (pid_enable) {
    PIDcontrol();
  } else {
    manualMode();
  }
}

