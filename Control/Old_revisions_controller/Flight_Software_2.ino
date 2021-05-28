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

IPAddress stationIP(192,168,87,109); // Home IP
//IPAddress stationIP(192,168,1,35); // Vicon Lab IP  1.33
int stationPort = 2380;

// UDP string and indexes
int ind1,ind2,ind3,ind4,ind5,ind6,ind7,ind8,ind9,ind10;
int ind11,ind12,ind13,ind14,ind15,ind16,ind17,ind18,ind19,ind20;
int ind21,ind22,ind23,ind24,ind25,ind26,ind27,ind28,ind29,ind30;
int ind31,ind32,ind33,ind34,ind35;

String d1,d2,d3,d4,d5,d6,d7,d8,d9,d10,d11,d12,d13,d14,d15,d16,d17,d18,d19,d20;
String d21,d22,d23,d24,d25,d26,d27,d28,d29,d30,d31,d32,d33,d34,d35;

String str="",sData="",altitudeData="",attitudeData="";
char sendBuffer[217];

WiFiUDP Udp;

unsigned long timer = 0;
long loopTime = 20000;    // in micro seconds
float sampleRate = 1.0/loopTime; //Hz
float sampleTime = loopTime*0.000001;

struct bno055_t BNO;
struct bno055_euler euler;
struct bno055_gyro omega;


unsigned char myChar;
unsigned long lastTime = 0;
int eUnit=0, gUnit=0, gBand=0, gRange=0;

float v_x=0.0,v_y=0.0,v_z=0.0,v_phi=0.0,v_theta=0.0,v_psi=0.0;
float des_x=0.0,des_y=0.0,des_z=0.0,pw_user=0.0;


float z_offset = 0.23;
float prev_z = 0.0;
float des_dd_z = 0.0;
float error_z=0.0, d_error_z=0.0, v_d_z=0.0, s_z=0.0,uz=0.0, pwmRate=0.0;
int edfPWM=0;



// Position controller variables
float lambda_x = 1.0;
float lambda_y = 1.0;
float A_x = 1.0;
float A_y = 1.0;
float B_x = 1.0;
float B_y = 1.0;
float eps_x = 0.1;
float eps_y = 0.1;

// Altitude controller variables
float lambda_z = 2.0;
float A_z = 1.0;
float B_z = 1.0;
float eps_z = 0.1;

// Attitude controller variables
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

float gravity = 9.81;
float mass = 3.5;
float Ix = 0.05;
float Iy = 0.04;
float Iz = 0.03;
float L = 0.10161;
float d = 0.0308;
float rho = 1.2041;
float Ad = 0.0016201;

float PWM_Attitude = 1.0;

Servo flap1;
Servo flap2;
Servo flap3;
Servo flap4;
Servo edf;

//float rad_to_microSec = 1000.0/PI;
float rad_to_microSec = 600.0/(54.0*PI/180.0);
int flap1_eq = 1510;
int flap2_eq = 1570;
int flap3_eq = 1640;
int flap4_eq = 1530;
int flapScaling = 1.0;

int flap1_angle = 0;
int flap2_angle = 0;
int flap3_angle = 0;
int flap4_angle = 0;

int vState = 0;
int connectedToStation = 0;

template<int dim, class ElemT> struct Diagonal{
    mutable ElemT m[dim];

    // The only requirement on this class is that it implement the () operator like so:
    typedef ElemT elem_t;

    ElemT &operator()(int row, int col) const
    {
        static ElemT dummy;

        // If it's on the diagonal and it's not larger than the matrix dimensions then return the element
        if(row == col && row < dim)
            return m[row];
        else
            // Otherwise return a zero
            return (dummy = 0);
    }
};

BLA::Matrix<3> measured_angles;
BLA::Matrix<3> measured_omega;
BLA::Matrix<3> prev_angles;
BLA::Matrix<3> desired_angles;
BLA::Matrix<3> measured_d_angles;
BLA::Matrix<3> desired_d_angles;
BLA::Matrix<4> servo_angles;
BLA::Matrix<4> prev_servo_angles;
BLA::Matrix<4> deflector_area;
BLA::Matrix<3, 3, Diagonal<3, float> > I;
BLA::Matrix<3, 3, Diagonal<3, float> > lambda_angles;
BLA::Matrix<3, 3, Diagonal<3, float> > A_angles;
BLA::Matrix<3, 3, Diagonal<3, float> > B_angles;
BLA::Matrix<3> eps_angles;
BLA::Matrix<3> desired_d_r;

  // Attitude
BLA::Matrix<3> error;
BLA::Matrix<3> d_error;
BLA::Matrix<3> s;
BLA::Matrix<3,3> omega_skew;
BLA::Matrix<3,3> H;
BLA::Matrix<3,3> H_inv;
BLA::Matrix<3,3> d_H_inv;
BLA::Matrix<3> d_euler;
BLA::Matrix<3> u_torque;
BLA::Matrix<4> u_force;

float u_phi=0.0,u_theta=0.0,u_psi=0.0;
float s_phi=0.0,s_theta=0.0,s_psi=0.0;


float prevW1=0.0, prevW2=0.0, prevW3=0.0;

void udpSend(String ReplyBuffer){
  Udp.beginPacket(stationIP, stationPort);
  Udp.print(ReplyBuffer);
  Udp.endPacket();
}

void udpGet(){
  int packetSize = Udp.parsePacket();
  while(!packetSize){
    packetSize = Udp.parsePacket();
    //if(serial){Serial.println("Waiting for packet");}
    };
    //Serial.println("Got packet");
  if (packetSize) {
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, packetSize);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    str = String(packetBuffer);
  }
}

void sendTelemetry(){
//  sData = String(micros())+','+String(vState)+','+String(v_x)+','+String(v_y)+','+String(v_z)+','+String(measured_angles(0))+','+String(measured_angles(1))+','+String(measured_angles(2))+','+String(measured_omega(0))+','+String(measured_omega(1))+','+String(measured_omega(2))+','+String(pwmRate)+','+String(uz)+','+String(u_phi)+','+String(u_theta)+','+String(u_psi)+','+String(servo_angles(0))+','+String(servo_angles(1))+','+String(servo_angles(2))+','+String(servo_angles(3));
  
//  sData = String(micros())+','+String(vState)+','+String(v_x,4)+','+String(v_y,4)+','+String(v_z,4)+','+String(measured_angles(0),8)+','+String(measured_angles(1),8)+','+String(measured_angles(2),8)+','+String(measured_omega(0),4)+','+String(measured_omega(1),4)+','+String(measured_omega(2),4);
//  sData = sData +','+String(pwmRate,4)+','+String(uz,8)+','+String(u_phi,8)+','+String(u_theta,8)+','+String(u_psi,8)+','+String(servo_angles(0),8)+','+String(servo_angles(1),8)+','+String(servo_angles(2),8)+','+String(servo_angles(3),8);
//  sData = sData + ',' + String(flap1_angle) + ',' + String(flap2_angle) + ',' + String(flap3_angle) + ',' + String(flap4_angle);
//  writeStrings();
  
//  sData = String(vState);
//  sData = sData + ","+ altitudeData+","+attitudeData;
//  sData = sData+","+attitudeData;
// length = 52
  sprintf(sendBuffer,"%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%1.4f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%2.2f,%1.4f,%1.4f,%1.4f,%1.3f,%1.3f,%2.3f,%2.3f,%2.2f,%2.2f,%2.2f,%1.4f,%2.3f,%1.4f,%1.4f,%1.4f,%1.4f",measured_angles(0),measured_angles(1),measured_angles(2),desired_angles(0),desired_angles(1),desired_angles(2),measured_omega(0),measured_omega(1),measured_omega(2),desired_d_angles(0),desired_d_angles(1),desired_d_angles(2),lambda_angles(0,0),lambda_angles(1,1),lambda_angles(2,2),A_angles(0,0),A_angles(1,1),A_angles(2,2),B_angles(0,0),B_angles(1,1),B_angles(2,2),eps_angles(0,0),eps_angles(1,1),eps_angles(2,2),v_z,des_z,v_d_z,desired_d_r(2),lambda_z,A_z,B_z,eps_z,uz,servo_angles(0),servo_angles(1),servo_angles(2),servo_angles(3));
  Udp.beginPacket(stationIP, stationPort);
  Udp.write(sendBuffer);
  Udp.endPacket();
  
  //if(serial){Serial.println(sendBuffer);}
}

void showLEDS(int num){
  if(num == 0){
    digitalWrite(led1Pin,LOW);
    digitalWrite(led2Pin,LOW);
    digitalWrite(led3Pin,LOW);
  }
  else if(num == 1){
    digitalWrite(led1Pin,HIGH);
    digitalWrite(led2Pin,LOW);
    digitalWrite(led3Pin,LOW);
  }
  else if(num == 2){
    digitalWrite(led1Pin,LOW);
    digitalWrite(led2Pin,HIGH);
    digitalWrite(led3Pin,LOW);
  }
  else if(num == 3){
    digitalWrite(led1Pin,HIGH);
    digitalWrite(led2Pin,HIGH);
    digitalWrite(led3Pin,LOW);
  }
  else if(num == 4){
    digitalWrite(led1Pin,LOW);
    digitalWrite(led2Pin,LOW);
    digitalWrite(led3Pin,HIGH);
  }
  else if(num == 5){
    digitalWrite(led1Pin,HIGH);
    digitalWrite(led2Pin,LOW);
    digitalWrite(led3Pin,HIGH);
  }
  else if(num == 6){
    digitalWrite(led1Pin,LOW);
    digitalWrite(led2Pin,HIGH);
    digitalWrite(led3Pin,HIGH);
  }
  else if(num == 7){
    digitalWrite(led1Pin,HIGH);
    digitalWrite(led2Pin,HIGH);
    digitalWrite(led3Pin,HIGH);
  }
  else{}
}

void state0(){ // Everything Off
  showLEDS(vState);
  digitalWrite(13,0);
  edf.writeMicroseconds(1000);;
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
}

void state1(){ // No Control Just Telemetry
  showLEDS(vState);
  getIMUData();
  sendTelemetry();

}

void state2(){ // Attitude control, Altitude Control, Manual PWM
    showLEDS(vState);
    getIMUData();
    PWM_Attitude = pw_user;
    pwmRate = 0.01*pw_user;
    
    edfPWM = int(interpolate(pwmRate,0.0,1.0,1000.0,2000.0));
    edf.writeMicroseconds(edfPWM);
    
    attitudeControl(measured_angles, measured_omega, desired_angles, desired_d_angles);
    

    
    sendTelemetry();
}

void state3(){ // Attitude control, Altitude Control EDF OFF
    showLEDS(vState);
    getIMUData();
    altitudeControlNoEDF();
    PWM_Attitude = pwmRate*100.0;
    attitudeControl(measured_angles, measured_omega, desired_angles, desired_d_angles);

    sendTelemetry();
     
}

void state4(){ // Attitude control, Altitude Control EDF ON 
  showLEDS(vState);
  getIMUData();
  
  if(pw_user < 60.0){
    PWM_Attitude = pw_user;
    pwmRate = pw_user*0.01;
    uz = pwmRate*47.93;
    edfPWM = int(interpolate(pwmRate,0.0,1.0,1000.0,2000.0));
    //if(serial){Serial.println(edfPWM);}
    edf.writeMicroseconds(edfPWM);
  }
  else{
    altitudeControl();
    PWM_Attitude = pwmRate*100.0;
  }
    
  attitudeControl(measured_angles, measured_omega, desired_angles, desired_d_angles);

//  if(serial){
//    Serial.print(lambda_angles(0,0));
//    Serial.print(" ");
//    Serial.print(A_angles(0,0));
//    Serial.print(" ");
//    Serial.print(B_angles(0,0));
//    Serial.print(" ");
//    Serial.print(eps_angles(0));
//    Serial.print(" ");
//    Serial.print(measured_angles(0));
//    Serial.print(" ");
//    Serial.print(measured_omega(0));
//    Serial.println(" ");
//  }
  sendTelemetry();
}

void state5(){
  // showLEDS(7);
  digitalWrite(13,HIGH);
}

void state6(){ // EDF Manual PWM
    pwmRate = pw_user*0.01;
    edfPWM = int(interpolate(pwmRate,0.0,1.0,1000.0,2000.0));
    edf.writeMicroseconds(edfPWM);
    sendTelemetry();
}

void servoStartup(){
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq+300);
  flap2.writeMicroseconds(flap2_eq+300);
  flap3.writeMicroseconds(flap3_eq+300);
  flap4.writeMicroseconds(flap4_eq+300);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq-300);
  flap2.writeMicroseconds(flap2_eq-300);
  flap3.writeMicroseconds(flap3_eq-300);
  flap4.writeMicroseconds(flap4_eq-300);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq+300);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq-300);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq-300);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq+300);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq+300);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq-300);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq-300);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq+300);
  delay(500);
  flap1.writeMicroseconds(flap1_eq);
  flap2.writeMicroseconds(flap2_eq);
  flap3.writeMicroseconds(flap3_eq);
  flap4.writeMicroseconds(flap4_eq);
}

void setup() {
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  delay(2000);
  digitalWrite(13,LOW);
  if(serial){ // Enable serial
  Serial.begin(9600);
  while (!Serial) {}
  Serial.println("Connecting to WiFi");
  }
  Wire.begin();
  if(bnoConnected){BNO_Init(&BNO);}
  while (status != WL_CONNECTED) { // Connect to WiFi
    status = WiFi.begin(ssid, pass);
    delay(5000);
  }
  if(serial){Serial.println("Connected to WiFi");}
  
  Udp.begin(localPort);
  
  
  pinMode(led1Pin,OUTPUT);
  pinMode(led2Pin,OUTPUT);
  pinMode(led3Pin,OUTPUT);

  digitalWrite(13,HIGH);
  delay(2000);
  digitalWrite(13,LOW);
  digitalWrite(led1Pin,LOW);
  digitalWrite(led2Pin,LOW);
  digitalWrite(led3Pin,LOW);
  
  if(bnoConnected){
    bno055_set_operation_mode(OPERATION_MODE_NDOF);
    delay(1);
  
    while(eUnit+gUnit+gBand+gRange != 4){ //Wait for correct settings on BNO
      bno055_set_euler_unit(1); //rad should be 1
      delay(1);
      bno055_set_gyro_unit(1); //rad should be 1
      delay(1);
      bno055_set_gyro_bandwidth(0x02); // 116 Hz should be 2
      delay(1);
      bno055_set_gyro_range(0x02); //500 rps should be 2
      delay(1);

      bno055_get_euler_unit(&myChar);
      if(serial){Serial.println(myChar);}
      if(myChar == 1){eUnit = 1;}
      else {eUnit = 0;}
      delay(2);
  
      bno055_get_gyro_unit(&myChar);
      if(serial){Serial.println(myChar);}
      if(myChar == 1){gUnit = 1;}
      else {gUnit = 0;}
      delay(2);
  
      bno055_get_gyro_bandwidth(&myChar);
      if(serial){Serial.println(myChar);}
      if(myChar == 2){gBand = 1;}
      else {gBand = 0;}
      delay(2);
  
      bno055_get_gyro_range(&myChar);
      if(serial){Serial.println(myChar);}
      if(myChar == 2){gRange = 1;}
      else {gRange = 0;}
      delay(2);

      digitalWrite(13,HIGH);
      delay(200);
      digitalWrite(13,LOW);
      delay(200);

      eUnit =1;
      gUnit =1;
      gBand =1;
      gRange =1;
    }
  }

  // Attitude Variables
  measured_angles.Fill(0);
  measured_omega.Fill(0);
  prev_angles.Fill(0);
  desired_angles.Fill(0);
  measured_d_angles.Fill(0);
  desired_d_angles.Fill(0);
  prev_servo_angles.Fill(0);
  
  
  lambda_angles(0,0) = lambda_phi;
  lambda_angles(1,1) = lambda_theta;
  lambda_angles(2,2) = lambda_psi;
  
  A_angles(0,0) = A_phi;
  A_angles(1,1) = A_theta;
  A_angles(2,2) = A_psi;

  B_angles(0,0) = B_phi;
  B_angles(1,1) = B_theta;
  B_angles(2,2) = B_psi;

  eps_angles(0,0) = eps_phi;
  eps_angles(1,1) = eps_theta;
  eps_angles(2,2) = eps_psi;

  // Position Variables
  desired_d_r.Fill(0);
  
  I(0,0) = Ix;
  I(1,1) = Iy;
  I(2,2) = Iz;
  
  flap1.attach(flap1Pin);
  flap2.attach(flap2Pin);
  flap3.attach(flap3Pin);
  flap4.attach(flap4Pin);
  edf.attach(edfPin,1000,2000);
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

  udpGet();
  //if(serial){Serial.println(str);}

  ind1 = str.indexOf(',');
  d1 = str.substring(0, ind1);
  if(connectedToStation == 0){
    udpSend("ArduinoON");
    delay(1);
    udpSend("ArduinoON");
    delay(1);
    connectedToStation = 1;
    digitalWrite(13,LOW);
    showLEDS(0);
  }
  else if(connectedToStation == 1 && d1.equals("Disconnect")){
    connectedToStation = 0;
    edf.writeMicroseconds(1000);
    digitalWrite(13,LOW);
    showLEDS(7);
  }
  else{
   vState = d1.toInt();
   ind2 = str.indexOf(',', ind1+1 );    
   d2 = str.substring(ind1+1);                // x pos
   ind3 = str.indexOf(',', ind2+1 );
   d3 = str.substring(ind2+1, ind3+1);        // y pos
   ind4 = str.indexOf(',', ind3+1 );
   d4 = str.substring(ind3+1, ind4+1);        // z pos
   ind5 = str.indexOf(',', ind4+1 );          
   d5 = str.substring(ind4+1, ind5+1);        // phi
   ind6 = str.indexOf(',', ind5+1 );          
   d6 = str.substring(ind5+1, ind6+1);        // theta
   ind7 = str.indexOf(',', ind6+1 );          
   d7 = str.substring(ind6+1, ind7+1);        // psi
   ind8 = str.indexOf(',', ind7+1 );          
   d8 = str.substring(ind7+1, ind8+1);        // x_des
   ind9 = str.indexOf(',', ind8+1 );          
   d9 = str.substring(ind8+1, ind9+1);        // y_des
   ind10 = str.indexOf(',', ind9+1 );          
   d10 = str.substring(ind9+1,ind10+1);       // z_des
   ind11 = str.indexOf(',', ind10+1 );          
   d11 = str.substring(ind10+1,ind11+1);      // pw_user
   ind12 = str.indexOf(',', ind11+1 );
   d12 = str.substring(ind11+1,ind12+1);      // lambda_phi
   ind13 = str.indexOf(',', ind12+1 );
   d13 = str.substring(ind12+1,ind13+1);      // lambda_theta
   ind14 = str.indexOf(',', ind13+1 );
   d14 = str.substring(ind13+1,ind14+1);      // lambda_psi
   ind15 = str.indexOf(',', ind14+1 );
   d15 = str.substring(ind14+1,ind15+1);      // A_phi
   ind16 = str.indexOf(',', ind15+1 );
   d16 = str.substring(ind15+1,ind16+1);      // A_theta
   ind17 = str.indexOf(',', ind16+1 );
   d17 = str.substring(ind16+1,ind17+1);      // A_psi
   ind18 = str.indexOf(',', ind17+1 );
   d18 = str.substring(ind17+1,ind18+1);      // B_phi
   ind19 = str.indexOf(',', ind18+1 );
   d19 = str.substring(ind18+1,ind19+1);      // B_theta
   ind20 = str.indexOf(',', ind19+1 );
   d20 = str.substring(ind19+1,ind20+1);      // B_psi
   ind21 = str.indexOf(',', ind20+1 );
   d21 = str.substring(ind20+1,ind21+1);      //eps_phi
   ind22 = str.indexOf(',', ind21+1 );
   d22 = str.substring(ind21+1,ind22+1);      //eps_theta
   ind23 = str.indexOf(',', ind22+1 );
   d23 = str.substring(ind22+1,ind23+1);      //eps_psi
   ind24 = str.indexOf(',', ind23+1 );
   d24 = str.substring(ind23+1,ind24+1);      // lambda_x
   ind25 = str.indexOf(',', ind24+1 );
   d25 = str.substring(ind24+1,ind25+1);      // lambda_y
   ind26 = str.indexOf(',', ind25+1 );
   d26 = str.substring(ind25+1,ind26+1);      // lambda_z
   ind27 = str.indexOf(',', ind26+1 );
   d27 = str.substring(ind26+1,ind27+1);      // A_x
   ind28 = str.indexOf(',', ind27+1 );
   d28 = str.substring(ind27+1,ind28+1);      // A_y
   ind29 = str.indexOf(',', ind28+1 );
   d29 = str.substring(ind28+1,ind29+1);      // A_z
   ind30 = str.indexOf(',', ind29+1 );
   d30 = str.substring(ind29+1,ind30+1);      // B_x
   ind31 = str.indexOf(',', ind30+1 );
   d31 = str.substring(ind30+1,ind31+1);      // B_y
   ind32 = str.indexOf(',', ind31+1 );
   d32 = str.substring(ind31+1,ind32+1);      // B_z
   ind33 = str.indexOf(',', ind32+1 );
   d33 = str.substring(ind32+1,ind33+1);      //eps_x
   ind34 = str.indexOf(',', ind33+1 );
   d34 = str.substring(ind33+1,ind34+1);      //eps_y
   ind35 = str.indexOf(',', ind34+1 );
   d35 = str.substring(ind34+1);              //eps_z
   
   
   v_x = d2.toFloat();
   v_y = d3.toFloat();
   v_z = d4.toFloat();
   v_phi = d5.toFloat();
   v_theta = d6.toFloat();
   v_psi = d7.toFloat();
   des_x = d8.toFloat();
   des_y = d9.toFloat();
   des_z = d10.toFloat();
   pw_user = d11.toFloat();
   lambda_angles(0,0) = d12.toFloat();
   lambda_angles(1,1) = d13.toFloat();
   lambda_angles(2,2) = d14.toFloat();
   A_angles(0,0) = d15.toFloat();
   A_angles(1,1) = d16.toFloat();
   A_angles(2,2) = d17.toFloat();
   B_angles(0,0) = d18.toFloat();
   B_angles(1,1) = d19.toFloat();
   B_angles(2,2) = d20.toFloat();
   eps_angles(0) = d21.toFloat();
   eps_angles(1) = d22.toFloat();
   eps_angles(2) = d23.toFloat();
   lambda_x = d24.toFloat();
   lambda_y = d25.toFloat();
   lambda_z = d26.toFloat();
   A_x = d27.toFloat();
   A_y = d28.toFloat();
   A_z = d29.toFloat();
   B_x = d30.toFloat();
   B_y = d31.toFloat();
   B_z = d32.toFloat();
   eps_x = d33.toFloat();
   eps_y = d34.toFloat();
   eps_z = d35.toFloat();
  }
  switch(vState){
    case 0: state0(); if(serial){Serial.println("Case 0");} delay(2); break;
    case 1: state1(); if(serial){Serial.println("Case 1");} delayMicroseconds(500); break;
    case 2: state2(); if(serial){Serial.println("Case 2");} delayMicroseconds(500);break;
    case 3: state3(); if(serial){Serial.println("Case 3");} delayMicroseconds(500);break;
    case 4: state4(); ;break;
    case 5: state5(); if(serial){Serial.println("Case 5");} delayMicroseconds(500);break;
    case 6: state6(); if(serial){Serial.println("Case 6");} delayMicroseconds(500);break;
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

void getIMUData(){
  if(bnoConnected){
    bno055_read_gyro_xyz(&omega);
    bno055_read_euler_hrp(&euler);

    measured_angles(0) = -(float(euler.p) / 900);
    measured_angles(1) = float(euler.r) / 900;
    measured_angles(2) = ((float(euler.h) / 900)-PI)-0;

    measured_omega(0) = float(omega.x) / 900;
    measured_omega(1) = -(float(omega.y) / 900);
    measured_omega(2) = -(float(omega.z) / 900);

    v_phi = measured_angles(0);
    v_theta = measured_angles(1);
    v_psi = measured_angles(2);
    
  }
  else{
    measured_omega(0) = v_phi;
    measured_omega(1) = v_theta;
    measured_omega(2) = v_psi;
  }
  if(serial){
    Serial.print(measured_omega(0));
    Serial.print(" ");
    }
  measured_omega(0) = 0.3*prevW1 + 0.7*measured_omega(0);
  measured_omega(1) = 0.95*prevW2 + 0.05*measured_omega(1);
  measured_omega(2) = 0.95*prevW3 + 0.05*measured_omega(2);

  if(serial){Serial.println(measured_omega(0));}
}

BLA::Matrix<3> sign(BLA::Matrix<3> s, BLA::Matrix<3> eps){
  BLA::Matrix<3> temp;
  for(int i=0;i<3;i++){
    if(s(i)>eps(i))
      temp(i) = -1;
    else if(s(i)<-eps(i))
      temp(i) = 1;
    else
      temp(i) = -s(i)/eps(i);
  }
  return temp;
}

float sat(float s, float eps){
  if(s>eps){
    return 1.0;
  }
  else if(s<-eps){
    return -1.0;
  }
  else{
    return s/eps;
  }
}

float interpolate(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void altitudeControl(){

  v_z = -(v_z - z_offset);
  des_z = -des_z;
  v_d_z = (v_z - prev_z)/sampleTime;
  prev_z = v_z;
  // Limit Vertival Velocity  
  v_d_z = max(-7.0,min(v_d_z,7.0));
  
  error_z = v_z - des_z;
  d_error_z = v_d_z - desired_d_r(2);
  s_z = d_error_z + lambda_z*error_z;

  uz = ((mass)/(cos(v_phi)*cos(v_theta)))*(A_z*sat(s_z,eps_z) + B_z*s_z + lambda_z*d_error_z - des_dd_z + gravity);

  pwmRate = uz/47.93;

  // PWM signal conditioning
  if(pwmRate > 0.9){pwmRate = 0.9;}
  if(pwmRate < 0.0){pwmRate = 0.0;}

  edfPWM = int(interpolate(pwmRate,0.0,1.0,1000.0,2000.0));
  edf.writeMicroseconds(edfPWM);

  

  
}

void altitudeControlNoEDF(){

  v_z = -(v_z - z_offset);
  des_z = -des_z;
  v_d_z = (v_z - prev_z)/sampleTime;
  prev_z = v_z;
  // Limit Vertival Velocity  
  v_d_z = max(-7.0,min(v_d_z,7.0));

  error_z = v_z - des_z;
  d_error_z = v_d_z - desired_d_r(2);
  s_z = d_error_z + lambda_z*error_z;

  uz = ((mass)/(cos(v_phi)*cos(v_theta)))*(A_z*sat(s_z,eps_z) + B_z*s_z + lambda_z*d_error_z - des_dd_z + gravity);

  pwmRate = uz/47.93;
  edf.write(0);
}

void attitudeControl(BLA::Matrix<3> angles, BLA::Matrix<3> omega, BLA::Matrix<3> des_angles, BLA::Matrix<3> des_d_angles){

  // Omega skewed matrix
  omega_skew(0,0) = 0.0;
  omega_skew(0,1) = -omega(2);
  omega_skew(0,2) = omega(1);
  
  omega_skew(1,0) = omega(2);
  omega_skew(1,1) = 0.0;
  omega_skew(1,2) = -omega(0);
  
  omega_skew(2,0) = -omega(1);
  omega_skew(2,1) = omega(0);
  omega_skew(2,2) = 0.0;
  
  // H matrix
  H(0,0) = 1.0;
  H(0,1) = 0.0;
  H(0,2) =-sin(angles(1));
  H(1,0) = 0.0;
  H(1,1) = cos(angles(0));
  H(1,2) =sin(angles(0))*cos(angles(1));
  H(2,0) = 0.0;
  H(2,1) = -sin(angles(0));
  H(2,2) = cos(angles(0))*cos(angles(1));
  
  // Inverse of H
  H_inv(0,0) =1.0;
  H_inv(0,1) =sin(angles(0))*tan(angles(1));
  H_inv(0,2) =cos(angles(0))*tan(angles(1));
  H_inv(1,0) =0.0;
  H_inv(1,1) =cos(angles(0));
  H_inv(1,2) =-sin(angles(0));
  H_inv(2,0) =0.0;
  H_inv(2,1) =sin(angles(0))/cos(angles(1));
  H_inv(2,2) =cos(angles(0))/cos(angles(1));
  
  // Time derivative of H_inv
  d_H_inv(0,0) =0.0;
  d_H_inv(0,1) = d_euler(0)*cos(angles(0))*tan(angles(1)) + sin(angles(0))*d_euler(1)*(1 + (tan(angles(1))*tan(angles(1))));
  d_H_inv(0,2) =-d_euler(0)*sin(angles(0))*tan(angles(1)) + cos(angles(0))*d_euler(1)*(1 + (tan(angles(1))*tan(angles(1))));
  d_H_inv(1,0) =0.0;
  d_H_inv(1,1) =-d_euler(0)*sin(angles(0));
  d_H_inv(1,2) =-d_euler(0)*cos(angles(0));
  d_H_inv(2,0) =0.0;
  d_H_inv(2,1) = (d_euler(0)*cos(angles(0))/cos(angles(1))) + (sin(angles(0))*d_euler(1)*sin(angles(1))/(cos(angles(1))*cos(angles(1))));
  d_H_inv(2,2) =(-d_euler(0)*sin(angles(0))/cos(angles(1))) + (cos(angles(0))*d_euler(1)*sin(angles(1))/(cos(angles(1))*cos(angles(1))));

  // Euler velocities
  d_euler = H_inv * omega;

//  if(isnan(d_euler(0))){d_euler(0) = 0.0;}
//  if(isnan(d_euler(1))){d_euler(1) = 0.0;}
//  if(isnan(d_euler(2))){d_euler(2) = 0.0;}


// des_angles(2) = angles(2);
  
  // Trajectory error and error rate
  error = angles - des_angles;
  d_error = d_euler - des_d_angles;

//  if(isnan(error(0))){error(0) = 0.0;}
//  if(isnan(error(1))){error(1) = 0.0;}
//  if(isnan(error(2))){error(2) = 0.0;}
//
//  if(isnan(d_error(0))){d_error(0) = 0.0;}
//  if(isnan(d_error(1))){d_error(1) = 0.0;}
//  if(isnan(d_error(2))){d_error(2) = 0.0;}

  // Surface vector
  s = d_error + lambda_angles * error;

//  if(isnan(s(0))){s(0) = 0.0;}
//  if(isnan(s(1))){s(1) = 0.0;}
//  if(isnan(s(2))){s(2) = 0.0;}
  
  s_phi = s(0);
  s_theta = s(1);
  s_psi = s(2);

  // Needed Torque
  u_torque = I * H * (-A_angles * sign(s,eps_angles) - B_angles*s - lambda_angles * d_error - d_H_inv * omega) + omega_skew * I * omega;

//  if(isnan(u_torque(0))){u_torque(0) = 0.0;}
//  if(isnan(u_torque(1))){u_torque(1) = 0.0;}
//  if(isnan(u_torque(2))){u_torque(2) = 0.0;}

  u_phi = u_torque(0);
  u_theta = u_torque(1);
  u_psi = u_torque(2);

//  if(serial){
//    Serial.print(angles(0));
//    Serial.print("\t");
//    Serial.print(angles(1));
//    Serial.print("\t");
//    Serial.print(angles(2));
//    Serial.print("\t");
//    
//    
//    Serial.print(u_phi);
//    Serial.print("\t");
//    Serial.print(u_theta);
//    Serial.print("\t");
//    Serial.print(u_psi);
//    Serial.println("\t");
//  }

  
  BLA::Matrix<4,3> toForce;
  toForce(0,0) =-1/L;
  toForce(0,1) =0.0;
  toForce(0,2) =1/d;
  toForce(1,0) =0.0;
  toForce(1,1) =-1/L;
  toForce(1,2) =1/d;
  toForce(2,0) =1/L;
  toForce(2,1) =0.0;
  toForce(2,2) =1/d;
  toForce(3,0) =0.0;
  toForce(3,1) =1/L;
  toForce(3,2) =1/d;
  
  u_force = toForce * u_torque;

//  if(isnan(u_force(0))){u_force(0) = 0.0;}
//  if(isnan(u_force(1))){u_force(1) = 0.0;}
//  if(isnan(u_force(2))){u_force(2) = 0.0;}

  servo_angles(0) = (1/6.639)*(asin(min(max(((2*u_force(0))/(1.143*rho*Ad*(48.648*PWM_Attitude-131.08))),-1),1)));
  servo_angles(1) = (1/6.639)*(asin(min(max(((2*u_force(1))/(1.143*rho*Ad*(48.648*PWM_Attitude-131.08))),-1),1)));
  servo_angles(2) = (1/6.639)*(asin(min(max(((2*u_force(2))/(1.143*rho*Ad*(48.648*PWM_Attitude-131.08))),-1),1)));
  servo_angles(3) = (1/6.639)*(asin(min(max(((2*u_force(3))/(1.143*rho*Ad*(48.648*PWM_Attitude-131.08))),-1),1)));

  if(isnan(servo_angles(0))){servo_angles(0) = prev_servo_angles(0);}
  if(isnan(servo_angles(1))){servo_angles(1) = prev_servo_angles(1);}
  if(isnan(servo_angles(2))){servo_angles(2) = prev_servo_angles(2);}
  if(isnan(servo_angles(3))){servo_angles(3) = prev_servo_angles(3);}
  prev_servo_angles = servo_angles;

  flap1_angle = int(flapScaling*(servo_angles(0)*rad_to_microSec));
  flap2_angle = int(flapScaling*(servo_angles(1)*rad_to_microSec));
  flap3_angle = int(flapScaling*(servo_angles(2)*rad_to_microSec));
  flap4_angle = int(flapScaling*(servo_angles(3)*rad_to_microSec));
  
  flap1_angle = min(max(flap1_angle,-300),300);
  flap2_angle = min(max(flap2_angle,-300),300);
  flap3_angle = min(max(flap3_angle,-300),300);
  flap4_angle = min(max(flap4_angle,-300),300);
  
  flap1_angle = flap1_eq+flap1_angle;
  flap2_angle = flap2_eq+flap2_angle;
  flap3_angle = flap3_eq+flap3_angle;
  flap4_angle = flap4_eq+flap4_angle;

//  flap1_angle = flap1_eq + int(flapScaling*(servo_angles(0)*rad_to_microSec));
//  flap2_angle = flap2_eq + int(flapScaling*(servo_angles(1)*rad_to_microSec));
//  flap3_angle = flap3_eq + int(flapScaling*(servo_angles(2)*rad_to_microSec));
//  flap4_angle = flap4_eq + int(flapScaling*(servo_angles(3)*rad_to_microSec));

  
  flap1.writeMicroseconds(flap1_angle);
  flap2.writeMicroseconds(flap2_angle);
  flap3.writeMicroseconds(flap3_angle);
  flap4.writeMicroseconds(flap4_angle);

  
}
