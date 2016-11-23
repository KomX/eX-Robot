#ifndef EX_FUNCTIONS_H
#define EX_FUNCTIONS_H

#include <JJ_MPU6050_DMP_6Axis.h>

// MPU control/status vars
bool       dmpReady = false;         // set true if DMP init was successful
uint8_t    mpuIntStatus;             // holds actual interrupt status byte from MPU
uint8_t    devStatus;          // return status after each device operation (0 = success, !0 = error)
//uint16_t   packetSize;         // expected DMP packet size (for us 18 bytes)
uint16_t   fifoCount;                // count of all bytes currently in FIFO
uint8_t    fifoBuffer[18];     // FIFO storage buffer

float Kp=KP;
float Kd=KD;
float Kp_thr=KP_THROTTLE;
float Ki_thr=KI_THROTTLE;
float Kp_user=KP;
float Kd_user=KD;
float Kp_thr_user=KP_THROTTLE;
float Ki_thr_user=KI_THROTTLE;
bool newControlParameters = false;
bool modifing_control_parameters=false;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float throttle;
float steering;
float max_throttle     = MAX_THROTTLE;
float max_steering     = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;


MPU6050           mpu;               // class default I2C address is 0x68 for MPU6050
Quaternion        q;

// Функция определения свободной памяти
int eX_Free_Ram () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


// Quick calculation to obtein Phi angle from quaternion solution (from DMP internal quaternion solution)
float dmpGetPhi()
{
  mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
  mpu.dmpGetQuaternion(&q, fifoBuffer); 
  mpu.resetFIFO();  // We always reset FIFO

  //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
  //return Phi angle (robot orientation) from quaternion DMP output
  return (atan2(2*(q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)* RAD2GRAD);
}


// PD controller implementation(Proportional, derivative). DT is in miliseconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint-input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp*error + (Kd*(setPoint - setPointOld) - Kd*(input - PID_errorOld2))/DT;       // + error - PID_error_Old2
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return(output);
}


// PI controller implementation (Proportional, integral). DT is in miliseconds
float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint-input;
  PID_errorSum += constrain(error,-ITERM_MAX_ERROR,ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum,-ITERM_MAX,ITERM_MAX);
  
  //Serial.println(PID_errorSum);

  output = Kp*error + Ki*PID_errorSum*DT*0.001; // DT is in miliseconds...
  return(output);
}

#endif //EX_FUNCTIONS_H
