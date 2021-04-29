#include "MKRIMU.h"
#include "EKF.h"
EKF Filter;
EKF F_YAW;

unsigned long timer = 0;
long loopTime = 10000;    // in micro seconds
float rate = loopTime/1000000.0;
float dt = loopTime/1000000.0;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float rv_w,rv_x,rv_y,rv_z;
float xx,yy,zz;
float phi, theta, psi;
float sphi,stheta,spsi;
float varPhi, varTheta, varPsi;
float varGx, varGy, varGz;
float varAx, varAy, varAz;
float rawPhi, rawTheta,rawPsi;
float Mx,My;
float lambda = 1.0;
#define debug 0

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  for(int i=0;i<100;i++){
    getAcc();
    getGyro();
  }
  getRawPsiMeasurements(1000);
  getRawPhiMeasurements(1000);
  getRawThetaMeasurements(1000);
  getGxMeasurements(1000);
  getGyMeasurements(1000);
  getGzMeasurements(1000);
  getAxMeasurements(1000);
  getAyMeasurements(1000);
  getAzMeasurements(1000);
  calculateAngles();

  Filter.initalAngle(rawPhi,rawTheta,-2.22); 
//  Filter.updateR(varPhi + lambda*varAx*varAx, varTheta + lambda*varAy*varAy, varPhi + lambda*varAz*varAz);
  Filter.updateR(varPhi + lambda*varAx*varAx, varTheta + lambda*varAy*varAy, varPhi + lambda*varAz*varAz*0.0);
  Filter.updateQ(varPhi*rate*rate, varTheta*rate*rate, varPsi*rate*rate*0.0, 2*varGx, 2*varGy, 2*varGz);
//    Filter.updateQ(varPhi*rate*rate, varTheta*rate*rate, varPsi*rate*rate, 2*varGx, 2*varGy, 2*varGz);
  
//  while(true){};
  timer = micros();
}

void loop() {
  timeSync(loopTime);
 
  getAcc();
  getGyro();
  getMagnet();
  calculateAngles();
  Filter.estimate(rawPhi,rawTheta,atan2(My,Mx),gx,gy,gz,rate);
//  Filter.estimate(rawPhi,rawTheta,rawPsi,gx,gy,gz,rate);
  phi = Filter.getPhi()*RAD_TO_DEG;
  theta = Filter.getTheta()*RAD_TO_DEG;
  psi = Filter.getPsi()*RAD_TO_DEG;

  if (IMU.eulerAnglesAvailable()) {
    IMU.readEulerAngles(stheta,sphi,spsi);
  }
///*
  Serial.print(theta*DEG_TO_RAD,4);
  Serial.print(",");
  Serial.print(phi*DEG_TO_RAD,4);
  Serial.print(",");
  Serial.print(psi*DEG_TO_RAD,4);////////////
  Serial.print(",");
  
  Serial.print(stheta*DEG_TO_RAD,4);
  Serial.print(",");
  Serial.print(sphi*DEG_TO_RAD,4);
  Serial.print(",");
  Serial.println(spsi*DEG_TO_RAD,4);
//*/ 

/*
  Serial.print(rawTheta,4);
  Serial.print(",");
  Serial.print(rawPhi,4);
  Serial.print(",");
  Serial.print(rawPsi,4);
  Serial.print(",");
  Serial.print(stheta*DEG_TO_RAD,4);
  Serial.print(",");
  Serial.print(sphi*DEG_TO_RAD,4);
  Serial.print(",");
  Serial.println(spsi*DEG_TO_RAD,4);
*/


/*
Serial.print("accx:");Serial.print(ax);Serial.print("\t");
Serial.print("accy:");Serial.print(ay);Serial.print("\t");
Serial.print("accz:");Serial.print(az);Serial.print("\t");
Serial.print("gx:");Serial.print(gx);Serial.print("\t");
Serial.print("gy:");Serial.print(gy);Serial.print("\t");
Serial.print("gz:");Serial.println(gz);
Serial.print("mx:");Serial.print(mx);Serial.print("\t");
Serial.print("my:");Serial.print(my);Serial.print("\t");
Serial.print("mz:");Serial.println(mz);
*/


/*
  ////////
  Serial.print(xx*DEG_TO_RAD,4);  
  Serial.print(",");
  Serial.print(yy*DEG_TO_RAD,4);  
  Serial.print(",");
  Serial.print(zz*DEG_TO_RAD,4);  
  Serial.println(",");
  */
 /*
  Serial.print("mx:");Serial.print(mx);Serial.print("\t");
  Serial.print("Mx:");Serial.print(Mx);Serial.print("\t");
  Serial.print("My:");Serial.print(My);Serial.println("\t");
  Serial.print("Theta:");Serial.print(rawTheta);Serial.print("\t");
  Serial.print("Phi:");Serial.print(rawPhi);Serial.print("\t");
  Serial.print("Psi:");Serial.print(rawPsi);Serial.println("\t");
*/  
  if(debug)Serial.print(rawPsi*RAD_TO_DEG,6); Serial.print("\t");
//  Serial.print(gx,6); Serial.print(" ");

//    Serial.print(rawTheta*RAD_TO_DEG,6); Serial.print("\t");
//  Serial.print(gy,6); Serial.println(" ");
  
  if(debug)Serial.print(psi); Serial.println("\t");
//    Serial.print(theta); Serial.println("\t");
//  Serial.println(psi); 
//  i+=1;
//}

//Serial.println("]");
//while(1){};
}

void timeSync(unsigned long deltaT)
{
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

void getquat(){
  float f=IMU.quaternion(rv_w,rv_x,rv_y,rv_z);
  xx = 2*(rv_x *rv_z - rv_w * rv_y);
  yy = 2*(rv_y*rv_z + rv_w*rv_x);
  zz = rv_w*rv_w - rv_x*rv_x - rv_y*rv_y - rv_z* rv_z;
  }

void getAcc(){
  int16_t bu[1];
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
//    if(!IMU.readRegisters(0x55, (uint8_t*)bu, sizeof(bu)))
ax=ax+bu[0]/1000.0-0.02;
//if(!IMU.readRegisters(0x57, (uint8_t*)bu, sizeof(bu)))
ay=ay+bu[0]/1000.0-0.15;
//if(!IMU.readRegisters(0x59, (uint8_t*)bu, sizeof(bu)))
az=az+bu[0]/1000.0

;

  }
}

void getGyro(){
    if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }
/*  int16_t bu[1];
  if(!IMU.readRegisters(0x61, (uint8_t*)bu, sizeof(bu)))
    gx = (float(gx)+bu[0]/16.0)*DEG_TO_RAD;//131.0;
    if(!IMU.readRegisters(0x63, (uint8_t*)bu, sizeof(bu)))
    gy = (float(gy)+bu[0]/16.0)*DEG_TO_RAD;//131.0;
    if(!IMU.readRegisters(0x65, (uint8_t*)bu, sizeof(bu)))
    gz = (float(gz)+bu[0]/16.0)*DEG_TO_RAD;//131.0;
*/
    gx = (float(gx))*DEG_TO_RAD;//131.0;
    gy = (float(gy))*DEG_TO_RAD;//131.0;
    gz = (float(gz))*DEG_TO_RAD;//131.0;
}

void getMagnet(){
    int16_t bu[1];
//    if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
//if(!IMU.readRegisters(0x5B, (uint8_t*)bu, sizeof(bu)))
mx=mx+bu[0]/16.0;
//if(!IMU.readRegisters(0x5D, (uint8_t*)bu, sizeof(bu)))
my=my+bu[0]/16.0;
//if(!IMU.readRegisters(0x5F, (uint8_t*)bu, sizeof(bu)))
mz=mz+bu[0]/16.0;
//  }
}


void calculateAngles(){
//  rawPhi = atan2(ay,az); //pitch
//  rawTheta = atan2(ax,sqrt(ay*ay+az*az)); //roll
  rawPhi = atan2(ay,sqrt(ay*ay+az*az)); //pitch
  rawTheta = atan2(ax,sqrt(ax*ax+az*az)); //roll
  Mx=mx*cos(rawTheta)+my*sin(rawPhi)*sin(rawTheta)-mz*cos(rawPhi)*sin(rawTheta);
  My=my*cos(rawPhi)-mz*sin(rawPhi);
  
  rawPhi = atan2(ay,az); //pitch
  rawTheta = atan2(ax,sqrt(ax*ax+az*az)); //roll
  Mx=mx*cos(rawTheta)+my*sin(rawPhi)*sin(rawTheta)-mz*cos(rawPhi)*sin(rawTheta);
//  Serial.print(mx,4);
//  Serial.print(",");
//  Serial.print(my,4);
//  Serial.print(",");
//  Serial.println(atan2(My,Mx)*RAD_TO_DEG,4);
  float TestPsi =atan2(My,Mx)*RAD_TO_DEG;
  //Serial.println(TestPsi);
  rawPsi = atan2(My,Mx);

}


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
void getRawPsiMeasurements(int j){
  float measurements[j];
  float sumMeasurements = 0;
  float tempSum = 0;
  float tempVal;

  for(int i=0;i<j;i++){
    getAcc();
    getMagnet();
    calculateAngles();
    measurements[i] = rawPsi;
  }
  
  if(debug)Serial.print("[");
  for(int i=0;i<j;i++){
    if(debug)Serial.print(measurements[i],6);if(debug)Serial.print(" ");
    sumMeasurements += measurements[i];
  }
  if(debug)Serial.println("[");
  float mean = sumMeasurements/j;
  if(debug)Serial.print("Mean rawPsi value: ");if(debug)Serial.println(mean,6);
  float meanSqr = mean * mean;
  for(int i=0;i<j;i++){
    tempVal = measurements[i] - mean;
    tempSum += tempVal * tempVal;
   }
   varPsi = tempSum/j;
  if(debug)Serial.print("Variance rawPsi value: ");if(debug)Serial.println(varPsi,10);
}

void getRawPhiMeasurements(int j){
  float measurements[j];
  float sumMeasurements = 0;
  float tempSum = 0;
  float tempVal;

  for(int i=0;i<j;i++){
    getAcc();
    calculateAngles();
    measurements[i] = rawPhi;
  }
  
  if(debug)Serial.print("[");
  for(int i=0;i<j;i++){
    if(debug)Serial.print(measurements[i],6);if(debug)Serial.print(" ");
    sumMeasurements += rawPhi;
  }
  if(debug)Serial.println("[");
  float mean = sumMeasurements/j;
  if(debug)Serial.print("Mean rawPhi value: ");if(debug)Serial.println(mean,6);
  float meanSqr = mean * mean;
  for(int i=0;i<j;i++){
    tempVal = measurements[i] - mean;
    tempSum += tempVal * tempVal;
   }
   varPhi = tempSum/j;
  if(debug)Serial.print("Variance rawPhi value: ");if(debug)Serial.println(varPhi,10);
}

void getRawThetaMeasurements(int j){
  float measurements[j];
  float sumMeasurements = 0;
  float tempSum = 0;
  float tempVal;

  for(int i=0;i<j;i++){
    getAcc();
    calculateAngles();
    measurements[i] = rawTheta;
  }
  
 if(debug) Serial.print("[");
  for(int i=0;i<j;i++){
    if(debug)Serial.print(measurements[i],6);if(debug)Serial.print(" ");
    sumMeasurements += measurements[i];
  }
  if(debug)Serial.println("[");
  float mean = sumMeasurements/j;
  if(debug)Serial.print("Mean rawTheta value: ");if(debug)Serial.println(mean,6);
  float meanSqr = mean * mean;
  for(int i=0;i<j;i++){
    tempVal = measurements[i] - mean;
    tempSum += tempVal * tempVal;
   }
   varTheta = tempSum/j;
  if(debug)Serial.print("Variance rawTheta value: ");if(debug)Serial.println(varTheta,10);
   
  

}

void getGxMeasurements(int j){
  float measurements[j];
  float sumMeasurements = 0;
  float tempSum = 0;
  float tempVal;

  for(int i=0;i<j;i++){
    getGyro();
    measurements[i] = gx;
  }
  
  if(debug)Serial.print("[");
  for(int i=0;i<j;i++){
    if(debug)Serial.print(measurements[i],6);if(debug)Serial.print(" ");
    sumMeasurements += measurements[i];
  }
  if(debug)Serial.println("[");
  float mean = sumMeasurements/j;
  if(debug)Serial.print("Mean Gx value: ");if(debug)Serial.println(mean,6);
  
  for(int i=0;i<j;i++){
    tempVal = measurements[i] - mean;
    tempSum += tempVal * tempVal;
   }
   varGx = tempSum/j;
 if(debug) Serial.print("Variance Gx value: ");if(debug)Serial.println(varGx,20);  
}

void getGyMeasurements(int j){
  float measurements[j];
  float sumMeasurements = 0;
  float tempSum = 0;
  float tempVal;

  for(int i=0;i<j;i++){
    getGyro();
    measurements[i] = gy;
  }
  
 if(debug) Serial.print("[");
  for(int i=0;i<j;i++){
   if(debug) Serial.print(measurements[i],6);if(debug)Serial.print(" ");
    sumMeasurements += measurements[i];
  }
  if(debug)Serial.println("[");
  float mean = sumMeasurements/j;
  if(debug)Serial.print("Mean Gy value: ");if(debug)Serial.println(mean,6);
  
  for(int i=0;i<j;i++){
    tempVal = measurements[i] - mean;
    tempSum += tempVal * tempVal;
   }
   varGy = tempSum/j;
  if(debug)Serial.print("Variance Gy value: ");if(debug)Serial.println(varGy,20);  
}

void getGzMeasurements(int j){
  float measurements[j];
  float sumMeasurements = 0;
  float tempSum = 0;
  float tempVal;

  for(int i=0;i<j;i++){
    getGyro();
    measurements[i] = gz;
  }
  
  if(debug)Serial.print("[");
  for(int i=0;i<j;i++){
    if(debug)Serial.print(measurements[i],6);if(debug)Serial.print(" ");
    sumMeasurements += measurements[i];
  }
  if(debug)Serial.println("[");
  float mean = sumMeasurements/j;
  if(debug)Serial.print("Mean Gz value: ");if(debug)Serial.println(mean,6);
  
  for(int i=0;i<j;i++){
    tempVal = measurements[i] - mean;
    tempSum += tempVal * tempVal;
   }
   varGz = tempSum/j;
  if(debug)Serial.print("Variance Gz value: ");if(debug)Serial.println(varGz,20);  
}

void getAxMeasurements(int j){
  float measurements[j];
  float sumMeasurements = 0;
  float tempSum = 0;
  float tempVal;

  for(int i=0;i<j;i++){
    getAcc();
    measurements[i] = ax;
  }
  
  if(debug)Serial.print("[");
  for(int i=0;i<j;i++){
    if(debug)Serial.print(measurements[i],6);if(debug)Serial.print(" ");
    sumMeasurements += measurements[i];
  }
  if(debug)Serial.println("[");
  float mean = sumMeasurements/j;
 if(debug) Serial.print("Mean Ax value: ");if(debug)Serial.println(mean,6);
  
  for(int i=0;i<j;i++){
    tempVal = measurements[i] - mean;
    tempSum += tempVal * tempVal;
   }
   varAx = tempSum/j;
  if(debug)Serial.print("Variance Ax value: ");if(debug)Serial.println(varAx,20);  
}

void getAyMeasurements(int j){
  float measurements[j];
  float sumMeasurements = 0;
  float tempSum = 0;
  float tempVal;

  for(int i=0;i<j;i++){
    getAcc();
    measurements[i] = ay;
  }
  
  Serial.print("[");
  for(int i=0;i<j;i++){
    Serial.print(measurements[i],6);if(debug)Serial.print(" ");
    sumMeasurements += measurements[i];
  }
  Serial.println("[");
  float mean = sumMeasurements/j;
  if(debug)Serial.print("Mean Ay value: ");if(debug)Serial.println(mean,6);
  
  for(int i=0;i<j;i++){
    tempVal = measurements[i] - mean;
    tempSum += tempVal * tempVal;
   }
   varAy = tempSum/j;
  if(debug)Serial.print("Variance Ay value: ");if(debug)Serial.println(varAy,20);  
}

void getAzMeasurements(int j){
  float measurements[j];
  float sumMeasurements = 0;
  float tempSum = 0;
  float tempVal;

  for(int i=0;i<j;i++){
    getAcc();
    measurements[i] = az;
  }
  
  if(debug)Serial.print("[");
  for(int i=0;i<j;i++){
    if(debug)Serial.print(measurements[i],6);if(debug)Serial.print(" ");
    sumMeasurements += measurements[i];
  }
  if(debug)Serial.println("[");
  float mean = sumMeasurements/j;
  if(debug)Serial.print("Mean Az value: ");if(debug)Serial.println(mean,6);
  
  for(int i=0;i<j;i++){
    tempVal = measurements[i] - mean;
    tempSum += tempVal * tempVal;
   }
   varAz = tempSum/j;
  if(debug)Serial.print("Variance Az value: ");if(debug)Serial.println(varAz,20);  
}
