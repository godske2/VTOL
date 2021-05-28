#include "arduino_secrets.h"
#include "BNO055_support.h"
#include <Wire.h>
#include <Servo.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#define serial 0 // 1 Serial on, 0 Serial off
#define bnoConnected 1 // 1 BNO connected, 0 BNO disconnected

#define flap1Pin 14 //A0
#define flap2Pin 15 //A1
#define flap3Pin 16 //A2
#define flap4Pin 17 //A3
#define edfPin 20 //A6
#define led1Pin 3
#define led2Pin 4
#define led3Pin 5


int status = WL_IDLE_STATUS;
char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
unsigned int localPort = 2390;
char packetBuffer[512];

//IPAddress stationIP(192, 168, 87, 108); // Home IP
IPAddress stationIP(192,168,1,34); // Vicon Lab IP  1.33
int stationPort = 2380;

// UDP string and indexes
int ind1, ind2, ind3, ind4, ind5, ind6, ind7, ind8, ind9, ind10;
int ind11, ind12, ind13, ind14, ind15, ind16, ind17, ind18, ind19, ind20;
int ind21, ind22, ind23, ind24, ind25, ind26, ind27, ind28, ind29, ind30;
int ind31, ind32, ind33, ind34, ind35;

String d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12, d13, d14, d15, d16, d17, d18, d19, d20;
String d21, d22, d23, d24, d25, d26, d27, d28, d29, d30, d31, d32, d33, d34, d35;

String str = "", sData = "", altitudeData = "", attitudeData = "";
char sendBuffer[140];

WiFiUDP Udp;

unsigned long timer = 0;
long loopTime = 20000;    // in micro seconds
float sampleRate = 1.0 / loopTime; //Hz
float sampleTime = loopTime * 0.000001;

struct bno055_t BNO;
struct bno055_euler euler;
struct bno055_gyro omega;


unsigned char myChar;
unsigned long lastTime = 0;
int eUnit = 0, gUnit = 0, gBand = 0, gRange = 0;


// ******************************************
// ************ Position Control ************
// ******************************************

// Measured Position
float r_e1 = 0.0;
float r_e2 = 0.0;
float r_e3 = 0.0;

// Desired Position
float des_r_e1 = 0.0;
float des_r_e2 = 0.0;
float des_r_e3 = 0.0;

// Previous position
float prev_r_e1 = 0.0;
float prev_r_e2 = 0.0;
float prev_r_e3 = 0.0;

// Measured Velocity
float d_r_e1 = 0.0;
float d_r_e2 = 0.0;
float d_r_e3 = 0.0;

// Desired Velocity
float des_d_r_e1 = 0.0;
float des_d_r_e2 = 0.0;
float des_d_r_e3 = 0.0;

// Desired Acceleration
float des_dd_r_e1 = 0.0;
float des_dd_r_e2 = 0.0;
float des_dd_r_e3 = 0.0;

// Error
float error_e1 = 0.0;
float error_e2 = 0.0;
float error_e3 = 0.0;

// Error Rate
float d_error_e1 = 0.0;
float d_error_e2 = 0.0;
float d_error_e3 = 0.0;

// Surface
float s_e1 = 0.0;
float s_e2 = 0.0;
float s_e3 = 0.0;

// Position Controller Gains
float lambda_e1 = 2.0;
float lambda_e2 = 2.0;
float lambda_e3 = 2.0;

float A_e1 = 1.0;
float A_e2 = 1.0;
float A_e3 = 1.0;

float B_e1 = 1.0;
float B_e2 = 1.0;
float B_e3 = 1.0;

float eps_e1 = 0.1;
float eps_e2 = 0.1;
float eps_e3 = 0.1;

float z_offset = 0.23;
float u_e1 = 0.0;
float u_e2 = 0.0;
float uz = 0.0;
float pwmRate = 0.0;
float pw_user = 0.0;
int edfPWM = 0;



// ******************************************
// ************ Attitude Control ************
// ******************************************

// Measured Euler Angles
float phi = 0.0;
float theta = 0.0;
float psi = 0.0;

// Desired Euler Angles
float des_phi = 0.0;
float des_theta = 0.0;
float des_psi = 0.0;

// Measured Angular Rate
float w_b1 = 0.0;
float w_b2 = 0.0;
float w_b3 = 0.0;

// Euler Rate
float d_phi = 0.0;
float d_theta = 0.0;
float d_psi = 0.0;

// Desired Euler Rate
float des_d_phi = 0.0;
float des_d_theta = 0.0;
float des_d_psi = 0.0;

// Desired Euler Acceleration
float des_dd_phi = 0.0;
float des_dd_theta = 0.0;
float des_dd_psi = 0.0;

// Attitude Error
float error_phi = 0.0;
float error_theta = 0.0;
float error_psi = 0.0;

// Attitude Error Rate
float d_error_phi = 0.0;
float d_error_theta = 0.0;
float d_error_psi = 0.0;

// Attitude Surface
float s_phi = 0.0;
float s_theta = 0.0;
float s_psi = 0.0;

// Attitude Controller Gains
float lambda_phi = 1.0; //10.0; //0.5;
float lambda_theta = 0.0; //10.0; //0.5;
float lambda_psi = 0.0;

float A_phi = 0.0; //0.5;
float A_theta = 0.0; //0.5;
float A_psi = 0.0;

float B_phi = 1.0; //3.0; //1.0;
float B_theta = 0.0; //3.0; //1.0;
float B_psi = 0.0;

float eps_phi = 0.001; //0.001;
float eps_theta = 0.1; //0.001;
float eps_psi = 0.1; //0.001;


// ******************************************
// *************** Constants ****************
// ******************************************

float gravity = 9.81;
float mass = 3.5;
float Ix = 0.05;
float Iy = 0.04;
float Iz = 0.03;
float L = 0.10161;
float d = 0.0308;
float rho = 1.2041;
float Ad = 0.0016201;

// ******************************************
// ************ Aux. Parameters *************
// ******************************************

float PWM_Attitude = 1.0;

Servo flap1;
Servo flap2;
Servo flap3;
Servo flap4;
Servo edf;

int flap1_eq = 1510;
int flap2_eq = 1570;
int flap3_eq = 1640;
int flap4_eq = 1530;


int flap1_angle = 0;
int flap2_angle = 0;
int flap3_angle = 0;
int flap4_angle = 0;

float rad_to_microSec = 600.0 / (54.0 * PI / 180.0);
int flapScaling = 1.0;

int vState = 0;
int connectedToStation = 0;

int calculateAltitude = 0;

template<int dim, class ElemT> struct Diagonal {
  mutable ElemT m[dim];

  // The only requirement on this class is that it implement the () operator like so:
  typedef ElemT elem_t;

  ElemT &operator()(int row, int col) const
  {
    static ElemT dummy;

    // If it's on the diagonal and it's not larger than the matrix dimensions then return the element
    if (row == col && row < dim)
      return m[row];
    else
      // Otherwise return a zero
      return (dummy = 0);
  }
};

BLA::Matrix<4> servo_angles;
BLA::Matrix<4> prev_servo_angles;
BLA::Matrix<3, 3, Diagonal<3, float> > I;
BLA::Matrix<3, 3> H;
BLA::Matrix<3, 3> d_H_inv;
BLA::Matrix<3> u_torque;

float u_phi = 0.0, u_theta = 0.0, u_psi = 0.0;


float prevW1 = 0.0, prevW2 = 0.0, prevW3 = 0.0;

void udpSend(String ReplyBuffer) {
  Udp.beginPacket(stationIP, stationPort);
  Udp.print(ReplyBuffer);
  Udp.endPacket();
  if (serial) {
    Serial.println(ReplyBuffer);
  }
}

void udpGet() {
  int packetSize = Udp.parsePacket();

  

  if (packetSize <= 42) { // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, packetSize);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    str = String(packetBuffer);
    ind1 = str.indexOf(',');
    d1 = str.substring(0, ind1);
    
    if (connectedToStation == 0 && d1.equals("StationON")) {
      udpSend("ArduinoON");
      delay(1);
      udpSend("ArduinoON");
      udpSend("ArduinoON");
      udpSend("ArduinoON");
      udpSend("ArduinoON");
      udpSend("ArduinoON");
      delay(1);
      connectedToStation = 1;
      vState = 0;
      digitalWrite(13, LOW);
      showLEDS(0);
    }
    if (connectedToStation == 1 && d1.equals("Disconnect")) {
      connectedToStation = 0;
      edf.writeMicroseconds(1000);
      vState = 0;
      digitalWrite(13, LOW);
      showLEDS(7);
    }
    if(packetSize == 3){
      vState = 0;
    }
  }

  if (packetSize >= 43 && packetSize < 50){
    int len = Udp.read(packetBuffer, packetSize);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    str = String(packetBuffer);

    ind1 = str.indexOf(',');
    d1 = str.substring(0, ind1);
    ind2 = str.indexOf(',', ind1 + 1 );
    d2 = str.substring(ind1 + 1);              // x pos
    ind3 = str.indexOf(',', ind2 + 1 );
    d3 = str.substring(ind2 + 1, ind3 + 1);    // y pos
    ind4 = str.indexOf(',', ind3 + 1 );
    d4 = str.substring(ind3 + 1, ind4 + 1);    // z pos
    ind5 = str.indexOf(',', ind4 + 1 );
    d5 = str.substring(ind4 + 1, ind5 + 1);    // x_des
    ind6 = str.indexOf(',', ind5 + 1 );
    d6 = str.substring(ind5 + 1, ind6 + 1);    // y_des
    ind7 = str.indexOf(',', ind6 + 1 );
    d7 = str.substring(ind6 + 1, ind7 + 1);  // z_des
    ind8 = str.indexOf(',', ind7 + 1 );
    d8 = str.substring(ind7 + 1); // pw_user

    
    r_e1 = d2.toFloat();
    r_e2 = d3.toFloat();
    r_e3 = d4.toFloat();
    des_r_e1 = d5.toFloat();
    des_r_e2 = d6.toFloat();
    des_r_e3 = -d7.toFloat();
    pw_user = d8.toFloat();

    calculateAltitude = 1;
    
  }
  
  if (packetSize >= 190) { // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, packetSize);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    str = String(packetBuffer);
    
    ind1 = str.indexOf(',');
    d1 = str.substring(0, ind1);
    ind2 = str.indexOf(',', ind1 + 1 );
    d2 = str.substring(ind1 + 1);              // x pos
    ind3 = str.indexOf(',', ind2 + 1 );
    d3 = str.substring(ind2 + 1, ind3 + 1);    // y pos
    ind4 = str.indexOf(',', ind3 + 1 );
    d4 = str.substring(ind3 + 1, ind4 + 1);    // z pos
    ind5 = str.indexOf(',', ind4 + 1 );
    d5 = str.substring(ind4 + 1, ind5 + 1);    // phi
    ind6 = str.indexOf(',', ind5 + 1 );
    d6 = str.substring(ind5 + 1, ind6 + 1);    // theta
    ind7 = str.indexOf(',', ind6 + 1 );
    d7 = str.substring(ind6 + 1, ind7 + 1);    // psi
    ind8 = str.indexOf(',', ind7 + 1 );
    d8 = str.substring(ind7 + 1, ind8 + 1);    // x_des
    ind9 = str.indexOf(',', ind8 + 1 );
    d9 = str.substring(ind8 + 1, ind9 + 1);    // y_des
    ind10 = str.indexOf(',', ind9 + 1 );
    d10 = str.substring(ind9 + 1, ind10 + 1);  // z_des
    ind11 = str.indexOf(',', ind10 + 1 );
    d11 = str.substring(ind10 + 1, ind11 + 1); // pw_user
    ind12 = str.indexOf(',', ind11 + 1 );
    d12 = str.substring(ind11 + 1, ind12 + 1); // lambda_phi
    ind13 = str.indexOf(',', ind12 + 1 );
    d13 = str.substring(ind12 + 1, ind13 + 1); // lambda_theta
    ind14 = str.indexOf(',', ind13 + 1 );
    d14 = str.substring(ind13 + 1, ind14 + 1); // lambda_psi
    ind15 = str.indexOf(',', ind14 + 1 );
    d15 = str.substring(ind14 + 1, ind15 + 1); // A_phi
    ind16 = str.indexOf(',', ind15 + 1 );
    d16 = str.substring(ind15 + 1, ind16 + 1); // A_theta
    ind17 = str.indexOf(',', ind16 + 1 );
    d17 = str.substring(ind16 + 1, ind17 + 1); // A_psi
    ind18 = str.indexOf(',', ind17 + 1 );
    d18 = str.substring(ind17 + 1, ind18 + 1); // B_phi
    ind19 = str.indexOf(',', ind18 + 1 );
    d19 = str.substring(ind18 + 1, ind19 + 1); // B_theta
    ind20 = str.indexOf(',', ind19 + 1 );
    d20 = str.substring(ind19 + 1, ind20 + 1); // B_psi
    ind21 = str.indexOf(',', ind20 + 1 );
    d21 = str.substring(ind20 + 1, ind21 + 1); //eps_phi
    ind22 = str.indexOf(',', ind21 + 1 );
    d22 = str.substring(ind21 + 1, ind22 + 1); //eps_theta
    ind23 = str.indexOf(',', ind22 + 1 );
    d23 = str.substring(ind22 + 1, ind23 + 1); //eps_psi
    ind24 = str.indexOf(',', ind23 + 1 );
    d24 = str.substring(ind23 + 1, ind24 + 1); // lambda_x
    ind25 = str.indexOf(',', ind24 + 1 );
    d25 = str.substring(ind24 + 1, ind25 + 1); // lambda_y
    ind26 = str.indexOf(',', ind25 + 1 );
    d26 = str.substring(ind25 + 1, ind26 + 1); // lambda_z
    ind27 = str.indexOf(',', ind26 + 1 );
    d27 = str.substring(ind26 + 1, ind27 + 1); // A_x
    ind28 = str.indexOf(',', ind27 + 1 );
    d28 = str.substring(ind27 + 1, ind28 + 1); // A_y
    ind29 = str.indexOf(',', ind28 + 1 );
    d29 = str.substring(ind28 + 1, ind29 + 1); // A_z
    ind30 = str.indexOf(',', ind29 + 1 );
    d30 = str.substring(ind29 + 1, ind30 + 1); // B_x
    ind31 = str.indexOf(',', ind30 + 1 );
    d31 = str.substring(ind30 + 1, ind31 + 1); // B_y
    ind32 = str.indexOf(',', ind31 + 1 );
    d32 = str.substring(ind31 + 1, ind32 + 1); // B_z
    ind33 = str.indexOf(',', ind32 + 1 );
    d33 = str.substring(ind32 + 1, ind33 + 1); //eps_x
    ind34 = str.indexOf(',', ind33 + 1 );
    d34 = str.substring(ind33 + 1, ind34 + 1); //eps_y
    ind35 = str.indexOf(',', ind34 + 1 );
    d35 = str.substring(ind34 + 1);            //eps_z

    vState = d1.toInt();
    r_e1 = d2.toFloat();
    r_e2 = d3.toFloat();
    r_e3 = d4.toFloat();
    phi = d5.toFloat();
    theta = d6.toFloat();
    psi = d7.toFloat();
    des_r_e1 = d8.toFloat();
    des_r_e2 = d9.toFloat();
    des_r_e3 = -d10.toFloat();
    pw_user = d11.toFloat();
    lambda_phi = d12.toFloat();
    lambda_theta = d13.toFloat();
    lambda_psi = d14.toFloat();
    A_phi = d15.toFloat();
    A_theta = d16.toFloat();
    A_psi = d17.toFloat();
    B_phi = d18.toFloat();
    B_theta = d19.toFloat();
    B_psi = d20.toFloat();
    eps_phi = d21.toFloat();
    eps_theta = d22.toFloat();
    eps_psi = d23.toFloat();
    lambda_e1 = d24.toFloat();
    lambda_e2 = d25.toFloat();
    lambda_e3 = d26.toFloat();
    A_e1 = d27.toFloat();
    A_e2 = d28.toFloat();
    A_e3 = d29.toFloat();
    B_e1 = d30.toFloat();
    B_e2 = d31.toFloat();
    B_e3 = d32.toFloat();
    eps_e1 = d33.toFloat();
    eps_e2 = d34.toFloat();
    eps_e3 = d35.toFloat();

  }
}

void sendTelemetry() {


  sprintf(sendBuffer, "%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%2.3f,%2.3f,%2.3f,%2.3f,%1.4f,%1.4f,%1.4f,%1.4f", phi, theta, psi, des_phi, des_theta, des_psi, w_b1, w_b2, w_b3, des_d_phi, des_d_theta, des_d_psi, d_r_e1, d_r_e2, d_r_e3, uz, servo_angles(0), servo_angles(1), servo_angles(2), servo_angles(3));


  Udp.beginPacket(stationIP, stationPort);
  Udp.write(sendBuffer);
  Udp.endPacket();
  //  Udp.beginPacket(stationIP, stationPort);
  //  Udp.write(sendBuffer);
  //  Udp.endPacket();
  //  Udp.beginPacket(stationIP, stationPort);
  //  Udp.write(sendBuffer);
  //  Udp.endPacket();
  //  Udp.beginPacket(stationIP, stationPort);
  //  Udp.write(sendBuffer);
  //  Udp.endPacket();

    if(serial){Serial.println(sendBuffer);}

}

void showLEDS(int num) {
  if (num == 0) {
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, LOW);
    digitalWrite(led3Pin, LOW);
  }
  else if (num == 1) {
    digitalWrite(led1Pin, HIGH);
    digitalWrite(led2Pin, LOW);
    digitalWrite(led3Pin, LOW);
  }
  else if (num == 2) {
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, HIGH);
    digitalWrite(led3Pin, LOW);
  }
  else if (num == 3) {
    digitalWrite(led1Pin, HIGH);
    digitalWrite(led2Pin, HIGH);
    digitalWrite(led3Pin, LOW);
  }
  else if (num == 4) {
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, LOW);
    digitalWrite(led3Pin, HIGH);
  }
  else if (num == 5) {
    digitalWrite(led1Pin, HIGH);
    digitalWrite(led2Pin, LOW);
    digitalWrite(led3Pin, HIGH);
  }
  else if (num == 6) {
    digitalWrite(led1Pin, LOW);
    digitalWrite(led2Pin, HIGH);
    digitalWrite(led3Pin, HIGH);
  }
  else if (num == 7) {
    digitalWrite(led1Pin, HIGH);
    digitalWrite(led2Pin, HIGH);
    digitalWrite(led3Pin, HIGH);
  }
  else {}
}

void state0() { // Everything Off
  showLEDS(vState);
  digitalWrite(13, 0);
  edf.writeMicroseconds(1000);;
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
}

void state1() { //  Telemetry
  showLEDS(vState);
  edf.writeMicroseconds(1000);;
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
  getIMUData();
  sendTelemetry();

}

void state2() { // Position Control
  showLEDS(vState);
  getIMUData();

  if (pw_user < 70.0) {
    PWM_Attitude = pw_user;
    pwmRate = pw_user * 0.01;
    uz = pwmRate * 47.93;
    edfPWM = int(interpolate(pwmRate, 0.0, 1.0, 1000.0, 2000.0));
    edf.writeMicroseconds(edfPWM);
  }
  else {
    altitudeControl();
    PWM_Attitude = pwmRate * 100.0;
  }
  
  positionControl();
  attitudeControl();
  sendTelemetry();
}

void state3() { // Attitude control, Altitude Control EDF OFF
  showLEDS(vState);
  getIMUData();
  PWM_Attitude = pwmRate * 100.0;
  attitudeControl();

  sendTelemetry();

}

void state4() { // Attitude control, Altitude Control EDF ON
  showLEDS(vState);
  getIMUData();

  if (pw_user < 60.0) {
    PWM_Attitude = pw_user;
    pwmRate = pw_user * 0.01;
    uz = pwmRate * 47.93;
    edfPWM = int(interpolate(pwmRate, 0.0, 1.0, 1000.0, 2000.0));
    edf.writeMicroseconds(edfPWM);
  }
  else {
    altitudeControl();
    PWM_Attitude = pwmRate * 100.0;
  }

  des_phi = 0.0;
  des_theta = 0.0;
  attitudeControl();
  sendTelemetry();
}

void state5() {
  // showLEDS(7);
  digitalWrite(13, HIGH);
}

void state6() { // EDF Manual PWM
  getIMUData();
  pwmRate = pw_user * 0.01;
  edfPWM = int(interpolate(pwmRate, 0.0, 1.0, 1000.0, 2000.0));
  edf.writeMicroseconds(edfPWM);
  sendTelemetry();
}

void servoStartup() {
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq + 300);
  flap2.writeMicroseconds(flap2_eq + 300);
  flap3.writeMicroseconds(flap3_eq + 300);
  flap4.writeMicroseconds(flap4_eq + 300);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq - 300);
  flap2.writeMicroseconds(flap2_eq - 300);
  flap3.writeMicroseconds(flap3_eq - 300);
  flap4.writeMicroseconds(flap4_eq - 300);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq + 300);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq - 300);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq - 300);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq + 300);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq + 300);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq - 300);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq - 300);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq + 300);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
}

void setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(2000);
  digitalWrite(13, LOW);
  if (serial) { // Enable serial
    Serial.begin(9600);
    while (!Serial) {}
    Serial.println("Connecting to WiFi");
  }
  Wire.begin();
  if (bnoConnected) {
    BNO_Init(&BNO);
  }
  while (status != WL_CONNECTED) { // Connect to WiFi
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }
  if (serial) {
    Serial.println("Connected to WiFi");
  }

  Udp.begin(localPort);


  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);

  digitalWrite(13, HIGH);
  delay(2000);
  digitalWrite(13, LOW);
  digitalWrite(led1Pin, LOW);
  digitalWrite(led2Pin, LOW);
  digitalWrite(led3Pin, LOW);

  if (bnoConnected) {
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(1);

    while (eUnit + gUnit + gBand + gRange != 4) { //Wait for correct settings on BNO
      bno055_set_euler_unit(1); //rad should be 1
      delay(1);
      bno055_set_gyro_unit(1); //rad should be 1
      delay(1);
      bno055_set_gyro_bandwidth(0x02); // 116 Hz should be 2
      delay(1);
      bno055_set_gyro_range(0x02); //500 rps should be 2
      delay(1);

      bno055_get_euler_unit(&myChar);
      if (serial) {
        Serial.println(myChar);
      }
      if (myChar == 1) {
        eUnit = 1;
      }
      else {
        eUnit = 0;
      }
      delay(2);

      bno055_get_gyro_unit(&myChar);
      if (serial) {
        Serial.println(myChar);
      }
      if (myChar == 1) {
        gUnit = 1;
      }
      else {
        gUnit = 0;
      }
      delay(2);

      bno055_get_gyro_bandwidth(&myChar);
      if (serial) {
        Serial.println(myChar);
      }
      if (myChar == 2) {
        gBand = 1;
      }
      else {
        gBand = 0;
      }
      delay(2);

      bno055_get_gyro_range(&myChar);
      if (serial) {
        Serial.println(myChar);
      }
      if (myChar == 2) {
        gRange = 1;
      }
      else {
        gRange = 0;
      }
      delay(2);

      digitalWrite(13, HIGH);
      delay(200);
      digitalWrite(13, LOW);
      delay(200);

      eUnit = 1;
      gUnit = 1;
      gBand = 1;
      gRange = 1;
    }
  }

  prev_servo_angles.Fill(0);

  I(0,0) = Ix;
  I(1,1) = Iy;
  I(2,2) = Iz;

  flap1.attach(flap1Pin);
  flap2.attach(flap2Pin);
  flap3.attach(flap3Pin);
  flap4.attach(flap4Pin);
  edf.attach(edfPin, 1000, 2000);
  delay(2);
  edf.writeMicroseconds(1000);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
  servoStartup();
  showLEDS(7);
}

void loop() {
  timeSync(loopTime);
  Udp.flush();
  udpGet();
//  if (serial){Serial.println(str);}

  switch (vState) {
    case 0: state0(); if (serial) {Serial.println("Case 0");} delay(10); break;
    case 1: state1(); delayMicroseconds(10); break;
    case 2: state2(); delayMicroseconds(10); break;
    case 3: state3(); delayMicroseconds(10); break;
    case 4: state4(); delayMicroseconds(10); break;
    case 5: state5(); delayMicroseconds(500); break;
    case 6: state6(); delayMicroseconds(10); break;
  }

}

void timeSync(unsigned long deltaT) {
  unsigned long currTime = micros();
  long timeToDelay = deltaT - (currTime - timer);
  if (timeToDelay > 5000)
  {
    delay(timeToDelay / 1000);
    delayMicroseconds(timeToDelay % 1000);
  }
  else if (timeToDelay > 0)
  {
    delayMicroseconds(timeToDelay);
  }
  else
  {
    // timeToDelay is negative so we start immediately
  }
  timer = currTime + timeToDelay;
}

void getIMUData() {
  if (bnoConnected) {
    bno055_read_gyro_xyz(&omega);
    bno055_read_euler_hrp(&euler);

    phi = -(float(euler.p) / 900);
    theta = float(euler.r) / 900;
    psi = ((float(euler.h) / 900) - PI) - 0;

    w_b1 = float(omega.x) / 900;
    w_b2 = -(float(omega.y) / 900);
    w_b3 = -(float(omega.z) / 900);

  }
  //  w_b1 = 0.3*prevW1 + 0.7*w_b1;
  //  w_b2 = 0.95*prevW2 + 0.05*w_b2;
  //  w_b3 = 0.95*prevW3 + 0.05*w_b3;
}

float sat(float s, float eps) {
  if (s > eps) {
    return 1.0;
  }
  else if (s < -eps) {
    return -1.0;
  }
  else {
    return s / eps;
  }
}

float interpolate(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void altitudeControl() {

if(calculateAltitude){
  calculateAltitude = 0;

  r_e3 = -(r_e3 - z_offset);
  d_r_e3 = (r_e3 - prev_r_e3) / (0.2);
  prev_r_e3 = r_e3;
  // Limit Vertival Velocity
  d_r_e3 = max(-7.0, min(d_r_e3, 7.0));

  error_e3 = r_e3 - des_r_e3;
  d_error_e3 = d_r_e3 - des_d_r_e3;

  s_e3 = d_error_e3 + lambda_e3 * error_e3;

  uz = ((mass) / (cos(phi) * cos(theta))) * (A_e3 * sat(s_e3, eps_e3) + B_e3 * s_e3 + lambda_e3 * d_error_e3 - des_dd_r_e3 + gravity);

  pwmRate = uz / 47.93;

  // PWM signal conditioning
  if (pwmRate > 0.9) {
    pwmRate = 0.9;
  }
  if (pwmRate < 0.0) {
    pwmRate = 0.0;
  }

  edfPWM = int(interpolate(pwmRate, 0.0, 1.0, 1000.0, 2000.0));
  edf.writeMicroseconds(edfPWM);

}



}


void attitudeControl() {

  BLA::Matrix<3, 3> w_skew;
  BLA::Matrix<3> w;
  BLA::Matrix<3> temp;
  temp.Fill(0.0);

  float sin_phi = sin(phi);
  float cos_phi = cos(phi);
  float sin_theta = sin(theta);
  float cos_theta = cos(theta);
  float cos_theta_sqr = cos_theta * cos_theta;
  float tan_theta = tan(theta);
  float tan_theta_sqr = tan_theta * tan_theta;

  // Angular Rate
  w(0) = w_b1;
  w(1) = w_b2;
  w(2) = w_b3;

  // Angular Rate skewed matrix
  w_skew(0, 0) = 0.0;
  w_skew(0, 1) = -w_b3;
  w_skew(0, 2) = w_b2;

  w_skew(1, 0) = w_b3;
  w_skew(1, 1) = 0.0;
  w_skew(1, 2) = -w_b1;

  w_skew(2, 0) = -w_b2;
  w_skew(2, 1) = w_b1;
  w_skew(2, 2) = 0.0;

  // H matrix
  H(0, 0) = 1.0;
  H(0, 1) = 0.0;
  H(0, 2) = -sin_theta;
  H(1, 0) = 0.0;
  H(1, 1) = cos_phi;
  H(1, 2) = sin_phi * cos_theta;
  H(2, 0) = 0.0;
  H(2, 1) = -sin_phi;
  H(2, 2) = cos_phi * cos_theta;


  // Euler velocities
  d_phi = w_b1 + w_b2 * sin_phi * tan_theta + w_b3 * cos_phi * tan_theta;
  d_theta = w_b2 * cos_phi - w_b3 * sin_phi;
  d_psi = (w_b2 * sin_phi) / cos_theta + (w_b3 * cos_phi) / cos_theta;

  // Time derivative of H_inv
  d_H_inv(0, 0) = 0.0;
  d_H_inv(0, 1) = d_phi * cos_phi * tan_theta + sin_phi * d_theta * (1 + tan_theta_sqr);
  d_H_inv(0, 2) = -d_phi * sin_phi * tan_theta + cos_phi * d_theta * (1 + tan_theta_sqr);
  d_H_inv(1, 0) = 0.0;
  d_H_inv(1, 1) = -d_phi * sin_phi;
  d_H_inv(1, 2) = -d_phi * cos_phi;
  d_H_inv(2, 0) = 0.0;
  d_H_inv(2, 1) = (d_phi * cos_phi / cos_theta) + (sin_phi * d_theta * sin_theta / cos_theta_sqr);
  d_H_inv(2, 2) = (-d_phi * sin_phi / cos_theta) + (cos_phi * d_theta * sin_theta / cos_theta_sqr);


  // Trajectory error
  error_phi = phi - des_phi;
  error_theta = theta - des_theta;
  error_psi = psi - des_psi;

  // Tajectory error rate
  d_error_phi = d_phi - des_d_phi;
  d_error_theta = d_theta - des_d_theta;
  d_error_psi = d_psi - des_d_psi;


  // Attitude surface
  s_phi = d_error_phi + lambda_phi * error_phi;
  s_theta = d_error_theta + lambda_theta * error_theta;
  s_psi = d_error_psi + lambda_psi * error_psi;


  temp = d_H_inv * w;
  temp(0) = -A_phi * sat(s_phi, eps_phi) - B_phi * s_phi - lambda_phi * d_error_phi - temp(0);
  temp(1) = -A_theta * sat(s_theta, eps_theta) - B_theta * s_theta - lambda_theta * d_error_theta - temp(1);
  temp(2) = -A_psi * sat(s_psi, eps_psi) - B_psi * s_psi - lambda_psi * d_error_psi - temp(2);

  // Needed Torque
  u_torque = I * H * temp + w_skew * I * w;

  float torque_phi = -u_torque(0);
  float torque_theta = -u_torque(1);
  float torque_psi = -u_torque(2);

  float force_deflector_1 = -(torque_phi / L) + (torque_psi / d);
  float force_deflector_2 = -(torque_theta / L) + (torque_psi / d);
  float force_deflector_3 = (torque_phi / L) + (torque_psi / d);
  float force_deflector_4 = (torque_theta / L) + (torque_psi / d);

  servo_angles(0) = (1 / 6.639) * (asin(min(max(((2 * force_deflector_1) / (1.143 * rho * Ad * (-(48.648 * PWM_Attitude - 131.08)))), -1), 1)));
  servo_angles(1) = (1 / 6.639) * (asin(min(max(((2 * force_deflector_2) / (1.143 * rho * Ad * (-(48.648 * PWM_Attitude - 131.08)))), -1), 1)));
  servo_angles(2) = (1 / 6.639) * (asin(min(max(((2 * force_deflector_3) / (1.143 * rho * Ad * (-(48.648 * PWM_Attitude - 131.08)))), -1), 1)));
  servo_angles(3) = (1 / 6.639) * (asin(min(max(((2 * force_deflector_4) / (1.143 * rho * Ad * (-(48.648 * PWM_Attitude - 131.08)))), -1), 1)));

  if (isnan(servo_angles(0))) {
    servo_angles(0) = prev_servo_angles(0);
  }
  if (isnan(servo_angles(1))) {
    servo_angles(1) = prev_servo_angles(1);
  }
  if (isnan(servo_angles(2))) {
    servo_angles(2) = prev_servo_angles(2);
  }
  if (isnan(servo_angles(3))) {
    servo_angles(3) = prev_servo_angles(3);
  }
  prev_servo_angles = servo_angles;

  flap1_angle = int(flapScaling * (servo_angles(0) * rad_to_microSec));
  flap2_angle = int(flapScaling * (servo_angles(1) * rad_to_microSec));
  flap3_angle = int(flapScaling * (servo_angles(2) * rad_to_microSec));
  flap4_angle = int(flapScaling * (servo_angles(3) * rad_to_microSec));

  flap1_angle = min(max(flap1_angle, -300), 300);
  flap2_angle = min(max(flap2_angle, -300), 300);
  flap3_angle = min(max(flap3_angle, -300), 300);
  flap4_angle = min(max(flap4_angle, -300), 300);

  flap1_angle = flap1_eq + flap1_angle;
  flap2_angle = flap2_eq + flap2_angle;
  flap3_angle = flap3_eq + flap3_angle;
  flap4_angle = flap4_eq + flap4_angle;

  flap1.writeMicroseconds(flap1_angle);
  flap2.writeMicroseconds(flap2_angle);
  flap3.writeMicroseconds(flap3_angle);
  flap4.writeMicroseconds(flap4_angle);
}


void positionControl() {

  error_e1 = r_e1 - des_r_e1;
  error_e2 = r_e2 - des_r_e2;

  d_r_e1 = (r_e1 - prev_r_e1) / sampleTime;
  d_r_e2 = (r_e2 - prev_r_e2) / sampleTime;

  d_error_e1 = d_r_e1 - des_d_r_e1;
  d_error_e2 = d_r_e2 - des_d_r_e2;

  s_e1 = lambda_e1 * error_e1 + d_error_e1;
  s_e2 = lambda_e2 * error_e2 + d_error_e2;

  if(uz <= 1){ // Prevent from getting NAN
    u_e1 = 0;
    u_e2 = 0;
  }
  else{
    u_e1 = -(mass / uz) * (A_e1 * sat(s_e1, eps_e1) + B_e1 * s_e1 + lambda_e1 * d_error_e1);
    u_e2 = -(mass / uz) * (A_e2 * sat(s_e2, eps_e2) + B_e2 * s_e2 + lambda_e2 * d_error_e2);
  }


  des_phi = sin(psi) * u_e1 - cos(psi) * u_e2;
  des_phi = min(max(des_phi,-1),1);
  des_phi = asin(des_phi);
  
  des_theta = (sin(psi) * u_e2 + cos(psi) * u_e1) / (cos(des_phi));
  des_theta = min(max(des_theta,-1),1);
  des_theta = asin(des_theta);
}
