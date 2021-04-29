#include <BasicLinearAlgebra.h>
#ifndef _EKF_h_
#define _EFF_h_

class EKF {
public:
    EKF(){
      float Q_angle = 0.001;//0.00001;//0.1f;
      float Q_bias = 0.003;
      float R_measure = 0.01;//0.01f;

      Q_phi =Q_angle; // 0.093;
      Q_theta =Q_angle; // 0.093;
      Q_psi =Q_angle; // 0.093;

      Q_w1_bias =Q_bias; // 0.0093; //0.00003f;
      Q_w2_bias =Q_bias; // 0.0093; //0.00003f;
      Q_w3_bias =Q_bias; // 0.0093; //0.00003f;

      R_raw_phi =R_measure; // 0.002;
      R_raw_theta =R_measure; // 0.002;
      R_raw_psi =R_measure; // 0.002;

      X.Fill(0);           // State Vector
      U.Fill(0);            // Inout Vector
      Y.Fill(0);           // Measurement Vector (Raw angles from Accelerometer and Magnetometer)
      P.Fill(0);        // Error covariance matrix - This is a 2x2 matrix
      //F.fill(0);        // Jacobian of State transition functions
      H = {1, 0, 0, 0, 0, 0,
           0, 1, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0};        //Jacobian of measurement functions
      X_estimate.Fill(0);  // State estimates

      Q = {Q_phi, 0, 0, 0, 0, 0,  // Process Noise
           0, Q_theta, 0, 0, 0, 0,
           0, 0, Q_psi, 0, 0, 0,
           0, 0, 0, Q_w1_bias, 0, 0,
           0, 0, 0, 0, Q_w2_bias, 0,
           0, 0, 0, 0, 0, Q_w3_bias};
      R = {R_raw_phi, 0, 0,  // Measurement Noise
           0, R_raw_theta, 0,
           0, 0, R_raw_psi};
      K.Fill(0);           // Kalman Gain
      S.Fill(0);        // Helper Matrix
      I = {1, 0, 0, 0, 0, 0,  // Identity Matrix
           0, 1, 0, 0, 0, 0,
           0, 0, 1, 0, 0, 0,
           0, 0, 0, 1, 0, 0,
           0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, 0, 1};
    }

    ~EKF(){}

    void initalAngle(float roll,float pitch, float yaw){
      X(0) = roll;
      X(1) = pitch;
      X(2) = yaw;
    }

    void updateR(float r1, float r2, float r3){
      R = {r1, 0, 0,  // Measurement Noise
           0, r2, 0,
           0, 0, r3};
    }

    void updateQ(float q_phi, float q_theta, float q_psi, float qW1b, float qW2b, float qW3b){
      Q = {q_phi, 0, 0, 0, 0, 0,  // Process Noise
           0, q_theta, 0, 0, 0, 0,
           0, 0, q_psi, 0, 0, 0,
           0, 0, 0, qW1b, 0, 0,
           0, 0, 0, 0, qW2b, 0,
           0, 0, 0, 0, 0, qW3b};
    }

    void estimate(float newPhi, float newTheta, float newPsi, float newW1, float newW2, float newW3, float loopTime){

      // Store filter loop time in seconds!
      dT = loopTime;
      // Store inputs U = [newW1, newW2, newW3]
      U(0) = newW1; U(1) = newW2; U(2) = newW3;

      // Store measurements Y = [phi, theta, psi]
      Y(0) = newPhi; Y(1) = newTheta; Y(2) = newPsi;

      // X = [phi, theta, psi, w1_bias, w2_bias, w3_bias]
      // Xk_minus = f(x_k-1, u_k-1)
      X(0) = X(0) + (U(0) - X(3))*dT + sin(X(0))*tan(X(1))*(U(1) - X(4))*dT + cos(X(0))*tan(X(1))*(U(2) - X(5))*dT;
      X(1) = X(1) + cos(X(0))*(U(1) - X(4))*dT - sin(X(0))*(U(2) - X(5))*dT;
      X(2) = X(2) + sin(X(0))*(U(1) - X(4))*dT/cos(X(1)) + cos(X(0))*(U(2) - X(5))*dT/cos(X(1));

      // Linearize f(f(x_k-1, u_k-1)) around X at Xk_minus = F
      getJacobianF();

      // Pk_minus = F * Pk-1 * F^T + Q
      // calculate_Pk_minus();
      P = Fj * P * ~Fj + Q;


      // Linearize h(x_k-1, u_k-1) around X at Xk_minus = H

      // H is static

      // S = H * Pk_minus * H^T + R
      // calculate_S();
      // inverse_S();
      S = H * P * ~H + R;
      // Invert(S);
      BLA::Matrix<3,3> S_inv;
      S_inv = S.Inverse();

      // K = Pk_minus * H^T * inv(S)
      K = P * ~H * S_inv;

      // X_estimate = Xk_minus + K*(Y - h(x,u))
      // estimate_X();
      X = X + K*(Y - H * X);

      // P_esimate = (I - K * H) * Pk_minus
      // estimate_P();
      P = (I - K * H) * P;

    }

    float getPhi(){
      return X(0);
    }
    float getTheta(){
      return X(1);
    }
    float getPsi(){
      return X(2);
    }

private:
    /* Extanded kalman filter variables
    * w1 = rotation around X
    * w2 = rotation around Y
    * w3 = rotation around Z
    */
    // Process noise variance for the accelerometer
    float Q_phi;
    float Q_theta;
    float Q_psi;
    // Process noise variance for the gyro bias
    float Q_w1_bias;
    float Q_w2_bias;
    float Q_w3_bias;
    // Measurement noise variance - this is actually the variance of the measurement noise
    float R_raw_phi;
    float R_raw_theta;
    float R_raw_psi;

    float phi;
    float theta;
    float psi;
    float w1_bias;
    float w2_bias;
    float w3_bias;

    float dT;             // Filter loop Time
    BLA::Matrix<6> X;           // State Vector
    BLA::Matrix<3> U;            // Inout Vector
    BLA::Matrix<3> Y;           // Measurement Vector (Raw angles from Accelerometer and Magnetometer)
    BLA::Matrix<6,6> P;        // Error covariance matrix - This is a 2x2 matrix
    BLA::Matrix<6,6> Fj;        // Jacobian of State transition functions
    BLA::Matrix<3,6> H;        //Jacobian of measurement functions
    BLA::Matrix<6> X_estimate;  // State estimates
    BLA::Matrix<6,6> Q;        // Process Noise
    BLA::Matrix<3,3> R;        // measurement Noise
    BLA::Matrix<6,3> K;           // Kalman Gain
    BLA::Matrix<3,3> S;        // Helper Matrix
    BLA::Matrix<6,6> I;
    void getJacobianF(){
      //x0 - phi
      //x1 - theta
      //x2 - psi
      //x3 - 
      //x4 - w2b
      //x5 - w3b
      float w2 = U(1)-X(4);
      float w3 = U(2)-X(5);

      Fj(0,0) = 1+(w2*cos(X(0)) - w3*sin(X(0)))*tan(X(1))*dT;
      Fj(0,1) = (w2*sin(X(0)) + w3*cos(X(0)))*(1+sq(tan(X(1))))*dT;
      Fj(0,2) = 0;
      Fj(0,3) = -dT;
      Fj(0,4) = -sin(X(0))*tan(X(1))*dT;
      Fj(0,5) = -cos(X(0))*tan(X(1))*dT;

      Fj(1,0) = -sin(X(0))*w2*dT - cos(X(0))*w3*dT;
      Fj(1,1) = 1;
      Fj(1,2) = 0;
      Fj(1,3) = 0;
      Fj(1,4) = -cos(X(0))*dT;
      Fj(1,5) = sin(X(0))*dT;

      Fj(2,0) = ((cos(X(0))*w2*dT - sin(X(0))*w3*dT)/cos(X(1)));
      Fj(2,1) = ((sin(X(0))*sin(X(1))*w2*dT + cos(X(0))*sin(X(1))*w3*dT)/sq(cos(X(1))));
      Fj(2,2) = 1;
      Fj(2,3) = 0;
      Fj(2,4) = -(sin(X(0))*dT)/cos(X(1));
      Fj(2,5) = -(cos(X(0))*dT)/cos(X(1));

      Fj(3,0) = 0;
      Fj(3,1) = 0;
      Fj(3,2) = 0;
      Fj(3,3) = 1;
      Fj(3,4) = 0;
      Fj(3,5) = 0;

      Fj(4,0) = 0;
      Fj(4,1) = 0;
      Fj(4,2) = 0;
      Fj(4,3) = 0;
      Fj(4,4) = 1;
      Fj(4,5) = 0;

      Fj(5,0) = 0;
      Fj(5,1) = 0;
      Fj(5,2) = 0;
      Fj(5,3) = 0;
      Fj(5,4) = 0;
      Fj(5,5) = 1;
    }
/*
    void getJacobianF(){
      Fj(0,0) = cos(X(0))*tan(X(1))*(U(1) - X(4))*dT - sin(X(0))*tan(X(1))*(U(2) - X(5))*dT + 1;
      Fj(0,1) = (sin(X(0))/cos(X(1)))*(U(1) - X(4))*dT + (cos(X(0))/cos(X(1)))*(U(2) - X(5))*dT;
      Fj(0,2) = 0;
      Fj(0,3) = -dT;
      Fj(0,4) = -sin(X(0))*tan(X(1))*dT;
      Fj(0,5) = -cos(X(0))*tan(X(1))*dT;

      Fj(1,0) = -sin(X(0))*(U(1) - X(4))*dT - cos(X(0))*(U(2) - X(5))*dT;
      Fj(1,1) = 1;
      Fj(1,2) = 0;
      Fj(1,3) = 0;
      Fj(1,4) = -cos(X(0))*dT;
      Fj(1,5) = sin(X(0))*dT;

      Fj(2,0) = (cos(X(0))/cos(X(1)))*(U(1) - X(4))*dT - (sin(X(0))/cos(X(1)))*(U(2) - X(5))*dT;
      Fj(2,1) = (sin(X(0))*sin(X(1))/(cos(X(1))*cos(X(1))))*(U(1) - X(4))*dT + (cos(X(0))*sin(X(1))/(cos(X(1))*cos(X(1))))*(U(2) - X(5))*dT;
      Fj(2,2) = 1;
      Fj(2,3) = 0;
      Fj(2,4) = -(sin(X(0))*dT)/cos(X(1));
      Fj(2,5) = -(cos(X(0))*dT)/cos(X(1));

      Fj(3,0) = 0;
      Fj(3,1) = 0;
      Fj(3,2) = 0;
      Fj(3,3) = 1;
      Fj(3,4) = 0;
      Fj(3,5) = 0;

      Fj(4,0) = 0;
      Fj(4,1) = 0;
      Fj(4,2) = 0;
      Fj(4,3) = 0;
      Fj(4,4) = 1;
      Fj(4,5) = 0;

      Fj(5,0) = 0;
      Fj(5,1) = 0;
      Fj(5,2) = 0;
      Fj(5,3) = 0;
      Fj(5,4) = 0;
      Fj(5,5) = 1;
    }
*/
};

#endif
