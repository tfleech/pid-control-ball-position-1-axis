#include <Servo.h>
#define sensor A0
#define monitorPin 7
#define thetaMax 89 //93 for not extended sensor
#define thetaMin 72//69
#define maxsensor 420
#define debug 0

Servo myservo;

float desiredDistance = 200;
int thetaOffset = 82;
float Kp = 0.08;
float Kd = 0.06;
float Kdd = 0;//3;
float dt = 0.03;

float prev_sensor_reading = -1000;
float prev_deriv = -99999;
float prev_distance = desiredDistance;
float prev_pos = thetaOffset;
const int window_size = 3;

float window[window_size];
int window_index = 0;
float window_total = 0;

unsigned long loopTime = micros();
unsigned long prevLoopTime = micros();
boolean switchFlag;

boolean switch_desired = true;
int switch_freq = 200; //units of time-steps
int switch_counter = 0;

void setup() {
  myservo.attach(9);
  Serial.begin(9600);
  pinMode(7, OUTPUT);

  //init window
  for (int i = 0; i<window_size; i++){
    window[i] = 0;
  }
}

float readSensor(){
  float volts = maxsensor - analogRead(sensor);
  if (prev_sensor_reading == -1000) {
    prev_sensor_reading = volts;
  }
  if (abs(volts - prev_sensor_reading) > 60) {
    volts = prev_sensor_reading;
  } else {
    prev_sensor_reading = volts;
  }
  //update window
  window_total = window_total - window[window_index];
  window[window_index] = volts;
  window_total += window[window_index];
  window_index = (window_index+1) % window_size;

  volts = window_total/float(window_size);
  //Serial.println(volts);
  //Serial.println(volts);
  //float new_val = 20*pow(volts, -1);

  //calculate distance
  //float distance = window_total / float(window_size);
  return(volts);
}

void loop() {

  //set duty cycle
  loopTime = micros();
  prevLoopTime = micros();
  while( loopTime < prevLoopTime + (dt*1000000)){
    loopTime = micros();
  }
  prevLoopTime = loopTime;
  
  if (debug) {
    switchFlag = !switchFlag;
    digitalWrite(monitorPin, switchFlag);
  }

  if (switch_desired) {
    //Serial.println(desiredDistance);
    if (switch_counter == switch_freq) {
      if (desiredDistance == 95) {
        desiredDistance = 200;
      } else {
        desiredDistance = 95;
      }
      switch_counter = 0;
    } else {
      switch_counter++;
    }
  }

  
  float distance = readSensor();
  int new_pos;
  float deriv = (distance - prev_distance)/dt;
  if (prev_deriv == -99999) {
    prev_deriv = deriv;
  }
  //Serial.println(deriv);
  if (abs(deriv - prev_deriv) > 200) {
    deriv = 0;
  }
  prev_deriv = deriv;
  Serial.println(deriv);

  new_pos = Kp*(distance - desiredDistance) + Kd*deriv + Kdd*(thetaOffset - myservo.read()) + thetaOffset;
  new_pos = max(new_pos, thetaMin);
  new_pos = min(new_pos, thetaMax);
  //Serial.println(new_pos);
  prev_pos = new_pos;
  prev_distance = distance;
  myservo.write(new_pos);
  
  if (debug) {
    Serial.print("Kp contribution: ");
    Serial.print(Kp*(distance - desiredDistance));
    Serial.print("Kdd contribution: ");
    Serial.print(Kdd*(thetaOffset - myservo.read()));
    Serial.print(", Kd contribution: ");
    Serial.println(Kd*deriv);
  }
}
