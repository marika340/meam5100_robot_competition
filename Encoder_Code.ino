//ENC04C QUADRATURE ENCODER READER CODE
//VCC - 5v
//GND - SAME AS MICROCONTROLLER
//A - INTERRUPT PIN A
//B - INTERRUPT PIN B
//M1 - FOR MOTOR POWER
//M2 - FOR MOTOR POWER

//SET UP WIFI STUFF
#include <WiFi.h>
#include <WiFiUdp.h>
//INCLUDE MEAM RESOURCES FOR WIFI
#include "html510.h"
#include "Websitemotor_encoder.h"
#include <Encoder.h>
HTML510Server h(80);
//SET UP SSID AT GM LAB
//const char* ssid = "TP-Link_8A8C";
//const char* password = "12488674";
//SET UP SSID AT ANH'S HOME
const char* ssid = "TheSpot";
const char* password = "D0esntSp0tLiveHere?";
//SET UP SSID AT ANH'S PHONE
//const char* ssid = "iPhone";
//const char* password = "anhduong";
//WIFI IP ADDRESSES
IPAddress myIP (192,168,1,105); //Anh's IP
IPAddress gateway(192,168,1,1); //Router IP
IPAddress subnet(255,255,255,0); //Subnet IP
//DECLARE UDP (USED SAMPLE FROM LECTURE)
WiFiUDP UDPTestServer;
int port = 8888; // any UDP port# up to 65535 // but higher is safer > 1023

//SET UP H-BRIDGE PINS
int Hpin = 1; //OUTPUT PIN 1 TO H-BRIDGE //SPEED THROUGH PWM
int Hpin_direction1 = 5;//PIN 5 TO DRIVE H-BRIDGE
int Hpin_direction2 = 6;//PIN 6 TO DRIVE H-BRIDGE

//SET UP FOR MOTOR VARIABLES
float motor_speed = 0; //MAP MOTOR SPEED TO PWM
int motor_direction = 1; // 1 = FORWARD, -1 = REVERSE, 0 = BRAKE

//SET UP ENCODER
volatile int encoder_count = 0;
//portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //ADD THIS TO MINIMIZE ERROR FROM LECTURE
unsigned long last_time = 0; 
int encoder_pin_a = 4; //INPUT PIN
int encoder_pin_b = 7; //INPUT PIN
int encoder_slot = 20; //COUNT 20 SLOTS
Encoder myEnc(encoder_pin_a, encoder_pin_b);

//SET UP FOR ESP 32
int frequency = 500; //PICK A FREQUENCY FOR A MOTOR, WANT TO PICK A FREQUENCY THAT DOES NOT PRODUCE NOISE
int duty_cycle = 50; //PLACE HOLDER VALUE
int period = 1000.0/(2*frequency); //PERIOD in ms, want blinking at half cycle
int resolution_bit = 14; //DEPENDING ON THE NUMBER OF BITS, USE BIT = 14 TO MEET THE REQUIREMENT OF > 39 kHz
int resolution = ((1<<resolution_bit)-1); //GET MAX RESOLUTION BASED ON BITS
int pwm_channel = 0; //THERE ARE 6 CHANNELS

//SET UP PID CONTROL VARIABLES - TUNING CONSTANT
float Kp = 1; //CHANGE FOR TUNING
float Ki = 0; //CHANGE FOR TUNING
float Kd = 0; //CHANGE FOR TUNING

//PID STATE
float integral = 0; //CHANGE FOR TUNING
float previous_error = 0; //CHANGE FOR TUNING
unsigned long last_pid_time = 0; //TIMER FOR PID
float previous_rpm = 0; //STORE PREVIOUS RPM VALUE
float previous_count = 0; //STORE PREVIOUS COUNT VALUE
float previous_manual_count = 0; //STORE PREVIOUS COUNT VALUE

//TARGET RPM
float rpm_desired = 0.0; //CHANGE IF WANT

//MOTOR SPECS CALCULATION WITH ENCODER
int gear_ratio = 125; //TEST YELLOW MOTOR = 120 AND 2 EDGE, OUR MOTOR IS 125 AND 4 EDGES
//ENCODER SPECS = 20 CPR, 4 COMES FROM 4 EDGES AKA QUADRATURE DECODING
float count_per_revolution = (20.0*gear_ratio); //20 CPR * 4 EDGES
//float count_per_revolution = 20; //FOR YELLOW MOTOR

//SET TO TRUE FOR PID, SET TO FALSE FOR MANUAL
bool pid_enable = false;

//LOOP FOR ANALOG WRITE
void ledcAnalogWrite(uint8_t pin, uint32_t value, uint32_t value_max) {
  uint32_t duty_cycle = map(value,0,value_max,0,resolution); //SET UP DUTY CYCLE
  // Set the PWM duty cycle of pin as a proportion of resolution bits
  ledcWrite(pin,duty_cycle); //DUTY CYCLE TO PWM
}

//SET UP MOTOR FUNCTION
void motor() {
  uint32_t pwm_value = (uint32_t)constrain(motor_speed,0,resolution);
  if (motor_direction == -1) { //REVERSE DIRECTION
    digitalWrite(Hpin_direction1,HIGH);
    digitalWrite(Hpin_direction2,LOW);
    ledcWrite(Hpin,pwm_value); //WRITE PWM TO PIN
  } else if (motor_direction == 1) { //FORWARD DIRECTION
    digitalWrite(Hpin_direction1,LOW);
    digitalWrite(Hpin_direction2,HIGH);
    ledcWrite(Hpin,pwm_value); //WRITE PWM TO PIN
  } else {//BRAKING
    digitalWrite(Hpin_direction1,LOW); //CHANGE HIGH HIGH LOGIC FOR OUR PART
    digitalWrite(Hpin_direction2,LOW);
    ledcWrite(Hpin,0); //WRITE PWM TO PIN
  }
  //Serial.printf("Motor speed: %d Direction: %d\n",motor_speed, motor_direction);
}

// //SET UP SPEED HANDLER
// void handleSpeed() {
//   if (pid_enable) {h.sendhtml(body); return; }
//   int motor_speed_new = h.getVal();
//   if (motor_speed_new < 0) motor_speed_new = 0;
//   if (motor_speed_new > resolution) motor_speed_new = resolution;
//   motor_speed = motor_speed_new;
//   motor(); //UPDATE H-BRIDGE
//   h.sendhtml(body);
// }

//SET UP MOTOR DESIRED RPM
void handleSpeed() {
  int rpm_new = h.getVal();
  if (pid_enable) {
  rpm_desired = constrain(rpm_new,0,110);
  integral = 0; //RESET
  previous_error = 0;
  Serial.printf("RPM set to: %.1f, PID enabled\n", rpm_desired);
  } else {
    motor_speed = map(constrain(rpm_new,0,110),0,110,0,resolution);
    motor();
    Serial.printf("Manual speed set to: %.1f, PID enabled\n", motor_speed);
    }
  h.sendhtml(body);
}

// //SET UP ENCODER COUNT
// void IRAM_ATTR encoderA() {
//   portENTER_CRITICAL_ISR(&timerMux);
//   int a = digitalRead(encoder_pin_a);
//   int b = digitalRead(encoder_pin_b);
//   if (a == b) {
//     encoder_count++; //COUNT +1
//   } else {
//     encoder_count--; //COUNT -1
//   }
//   portEXIT_CRITICAL_ISR(&timerMux);
// }
// void IRAM_ATTR encoderB() {
//   portENTER_CRITICAL_ISR(&timerMux);
//   int a = digitalRead(encoder_pin_a);
//   int b = digitalRead(encoder_pin_b);
//   if (a != b) {
//     encoder_count++; //COUNT +1
//   } else {
//     encoder_count--; //COUNT -1
//   }
//   portEXIT_CRITICAL_ISR(&timerMux);
// }
//SET UP WEBSITE
void handleRoot() {
  h.sendhtml(body); //HTML CODE SEPARATE
}

//SET UP DIRECTION HANDLER
void handleDirection() {
  int motor_direction_new = h.getVal();
  if (motor_direction_new < -1) motor_direction_new = -1;
  if (motor_direction_new > 1) motor_direction_new = 1;
  motor_direction = motor_direction_new;
  motor();
  Serial.printf("Direction of motor: %d\n",motor_direction);
  h.sendhtml(body);
}

//SET UP PID FUNCTION
void PIDcontrol() {
  //TIME DERIVATIVE DT
  unsigned long current = millis(); //SET UP CURRENT TIME 
  float dt = (current - last_pid_time)/1000.0; //FIND DELTA TIME IN MILLISECONDS
  //ESTABLISH CURRENT COUNT
  //INSTALL ENCODER COUNT LIBRARY BY Paul Stoffregen (v 1.4.4)
  if (dt >= 0.100) {//RUN PID AT 500 MS, BUT CAN CHANGE
    //GET VALUE FROM CURRENT RPM 
    long current_count = myEnc.read(); //USING ENCODER COUNTING LIBRARY 
    //Serial.printf("Count: %.1f  Encoder count: %.1f", count, encoder_count);
    float current_rpm = ((current_count-previous_count)/(float)count_per_revolution)*(60.0/dt);
    //encoder_count = 0; //RESET
    //CALCULATE ERROR
    float error = rpm_desired - current_rpm;
    //P - PROPORTIONAL TUNING
    float output = Kp*error; // P OUTPUT
    //I - INTEGRAL TUNING
    integral += error*dt;
    //MAKE SURE DOES NOT GO ABOVE 150 RPM
    integral = constrain(integral,-150,150);
    //output += Ki*integral; // I OUTPUT
    float output_Ki = Ki*integral; //I OUTPUT
    //D - DERIVATE TUNING
    //float derivative = (current_rpm-previous_rpm)/dt;
    float derivative = (error - previous_error)/dt;
    //output += Kd*derivative; // D OUTPUT
    float output_Kd = Kd*derivative; //D OUTPUT
    //ADD ALL OUTPUT
    float control_output = output + output_Ki + output_Kd;
    //APPLY TO MOTOR
    //motor_speed = constrain(control_output,-1001,1001);
    if (control_output < 0) {
      motor_direction = -1;
      motor_speed = constrain(abs(control_output)*(resolution/110.0),0,resolution);
    } else {
      motor_direction = 1;
      motor_speed = constrain((control_output)*(resolution/110.0),0,resolution);
    }
    motor();
    //PRINT VALUE
    Serial.printf("Current RPM: %.1f Error %.1f Kp_out:%.1f Ki_out:%.1f Kd_out:%.1f\n",current_rpm,error,Kp*error,Ki*integral,Kd*derivative);
    Serial.printf("Delta_count %.1f", current_count-previous_count);
    previous_count = current_count; //UPDATE COUNT
    // if (motor_speed == 0) {
    //   myEnc.write(0);  // reset encoder when motor is stopped
    //   previous_count = 0;
    //   previous_manual_count = 0;
    // }
    //UPDATE ERROR AND TIME AND RPM AND COUNT
    previous_rpm = current_rpm;
    previous_error = error;
    last_pid_time = current; //UPDATE TIME
  }
}
//SET UP KP
void handleKp() {
  Kp = h.getVal();
  integral = 0;
  previous_error = 0;
  previous_rpm = 0;
  Serial.printf("Kp: %.2f\n",Kp);
  h.sendhtml(body);
}
//SET UP KI
void handleKi() {
  Ki = h.getVal();
  integral = 0;
  previous_error = 0;
  Serial.printf("Ki: %.2f\n",Ki);
  h.sendhtml(body);
}
//SET UP KD
void handleKd() {
  Kd = h.getVal();
  integral = 0;
  previous_error = 0;
  Serial.printf("Kd: %.2f\n",Kd);
  h.sendhtml(body);
}
//MANUAL OR PID MODE
void handleMode() {
  int mode = h.getVal();
  myEnc.write(0); // Clear the internal library state
  previous_manual_count = 0;
  previous_count = 0;
  if (mode == 0) {
    pid_enable = false;
    motor_speed = 0;
    previous_manual_count = myEnc.read(); // reset manual count
    motor();
  } else {
    pid_enable = true;
    integral = 0;
    previous_error = 0;
    previous_count = myEnc.read(); // reset manual count
  }
  Serial.printf("Mode: %d\n",mode);
  h.sendhtml(body);
}
void setup() {
  // put your setup code here, to run once:
  //SET UP OUTPUT PIN FOR MOTOR
  pinMode(Hpin_direction1,OUTPUT);
  pinMode(Hpin_direction2,OUTPUT);
  pinMode(Hpin,OUTPUT);

  //SET UP INPUT PIN FOR ENCODER
  pinMode(encoder_pin_a,INPUT_PULLUP);
  pinMode(encoder_pin_b,INPUT_PULLUP);

  //SET DIRECTION PINS AS HIGH (FOR OUR MOTOR DRIVER) FOR BRAKING AS DEFAULT
  digitalWrite(Hpin_direction1,LOW);
  digitalWrite(Hpin_direction2,LOW);

  //SET UP WIFI
  Serial.begin(115200); //FOR SERIAL MONITOR WINDOW TO DEBUG
  WiFi.begin(ssid,password);
  while (WiFi.status() != WL_CONNECTED) { //CHECK WIFI STATUS
  delay(500); //WAIT
  Serial.print("."); //PRINT
  }
  Serial.println("WiFi connected"); //PRINT
  Serial.println("IP address:");
  Serial.println(WiFi.localIP());
  UDPTestServer.begin(port); 

  //SET UP THE HANDLERS
  h.begin();
  h.attachHandler("/motor_speed=", handleSpeed);
  h.attachHandler("/motor_direction=", handleDirection);
  //h.attachHandler("/RPM=", handleRPM);
  h.attachHandler("/Mode=", handleMode);
  h.attachHandler("/Kp=", handleKp);
  h.attachHandler("/Ki=", handleKi);
  h.attachHandler("/Kd=", handleKd);
  h.attachHandler("/",handleRoot);

  //SET UP ANALOG WRITE
  // Setup a channel to output a square wave at a specific pin, frequency, and resolution_bit for an assigned channel, return true if successful
  analogReadResolution(resolution_bit); //ADC BITS READ
  ledcAttachChannel(Hpin,frequency,resolution_bit,pwm_channel); //ATTACH PIN TO THE CHANNEL
  ledcWrite(Hpin,0); //MOTOR STARTS AT 0
  //attachInterrupt(digitalPinToInterrupt(encoder_pin_a),encoderA,CHANGE); //CHANGE = RISING AND FALLING
  //attachInterrupt(digitalPinToInterrupt(encoder_pin_b),encoderB,CHANGE);
  //UPDATE TIME
  last_pid_time = millis();
  last_time = millis();
}
void manualMode() {
  //UPDATE ENCODER VALUE
  if(millis() - last_time >= 100) { //START A TIMER IN MS
    long now = millis();
    float dt_manual = (now - last_time)/1000.0;
    long current_manual_count = myEnc.read();
    float count = current_manual_count - previous_manual_count;
    float rpm = ((count/count_per_revolution)*(60.0/dt_manual)); //CONVERT PULSE TO RPM AND SCALE TO SECOND DUE TO DELAY
    Serial.printf("RPM: %.1f Total count: %ld\n",rpm,current_manual_count);
    previous_manual_count = current_manual_count; //RESET FOR NEXT CALCULATION
    last_time = now; //UPDATE TIMER
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
