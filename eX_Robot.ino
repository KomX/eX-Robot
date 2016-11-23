// eX-Robot  Балансирующий робот с шаговым приводом
// Author: Oleg Kravtchenko
// Date: 06/06/2014
// Updated: 18/10/2014
// License: GPL v2

// Для создания балансирующего робота использовано: 
// - 1 SparkFun (Deek-Robot) Pro Micro Board (ATmega32u4)
// - 1 DMP processor MPU6050
// - 1 Distance sensor (sonar) HC-SR04
// - 1 WiFi module ESP8266
// - 2 Drivers Stepper Motors A4988
// - 2 Stepper Motors 17HS2408 (0.6A, 1.8 deg/step)
// - 1 Futaba Micro 9g Servo 

// КРАТКИЕ ПОЯСНЕНИЯ ПО ПРИНЦИПУ РАБОТЫ:
// Управление шаговыми двигателями производится на частоте 200 Гц и осуществляется
// с помощью аппаратного счетчика прерываний timer1.
// Расчет угла наклона и управляющих сигналов производится асинхронно в цикле loop
// по данным от DMP обновляющимся с частотой 200 Гц.
// DMP использует gyro_bias_no_motion метод коррекции.
// Схема выходит на стабильный режим с достоверными данными в течении 10-15 сек.

//#include <Arduino.h>
#include <Wire.h>
#include <I2Cdev.h> 
#include <JJ_MPU6050_DMP_6Axis.h>
#include "eX_Hard.h"
#include "eX_WiFi.h"
#include "eX_OSC.h"
#include "eX_Functions.h"

bool       Robot_shutdown = false;   // Robot shutdown flag => Out of 

uint8_t    loop_counter;             // To generate a medium loop 40Hz 
uint16_t   slow_loop_counter;        // slow loop 2Hz

long       timer_old;
long       timer_value;
int        debug_counter;
//float      debugVariable;
float      dt;

// Angle of the robot (used for stability control)
float angle_adjusted;
float angle_adjusted_Old;

int16_t motor1;
int16_t motor2;

uint8_t mode;  // mode = 0 Normal mode, mode = 1 Pro mode ()

int16_t actual_robot_speed;        // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;    
float   estimated_speed_filtered;    // Estimated robot speed


// Read control PID parameters from user. This is only for advanced users that want to "play" with the controllers...
void readControlParameters()
{
  // Parameter initialization (first time we enter page2)
  if ((page==2)&&(!modifing_control_parameters))
  {
    fadder1 = 0.5;
    fadder2 = 0.5;
    fadder3 = 0.5;
    fadder4 = 0.0;
    modifing_control_parameters=true;
  }
  // Parameters Mode (page2 controls)
  // User could adjust KP, KD, KP_THROTTLE and KI_THROTTLE (fadder1,2,3,4)
  // Now we need to adjust all the parameters all the times because we don´t know what parameter has been moved
  if (page==2)
  {
    Kp_user = KP*2*fadder1;
    Kd_user = KD*2*fadder2;
    Kp_thr_user = KP_THROTTLE*2*fadder3;
    Ki_thr_user = (KI_THROTTLE+0.1)*2*fadder4;
    #if DEBUG>0 
    Serial.print("Par: ");
    Serial.print(Kp_user);
    Serial.print(" ");
    Serial.print(Kd_user);
    Serial.print(" ");
    Serial.print(Kp_thr_user);
    Serial.print(" ");
    Serial.println(Ki_thr_user);
    #endif
    // Kill robot => Sleep
    while (toggle2)
    {
      //Reset external parameters
      mpu.resetFIFO();
      PID_errorSum = 0;
      timer_old = millis(); 
      eX_Set_Motors_Speed(0,0);
      eX_WiFi_MSG_Read();
    }
    newControlParameters = true;
  }
  if ((newControlParameters)&&(!modifing_control_parameters))
  {
    // Reset parameters
    fadder1 = 0.5;
    fadder2 = 0.5;
    newControlParameters=false;
  }
}


void setup() {
  pinMode(SERVO_PIN,OUTPUT);             // SERVO
  pinMode(SONAR_TRIG_PIN,OUTPUT);        // TRIG SONAR
  pinMode(SONAR_ECHO_PIN,INPUT);         // ECHO SONAR
  pinMode(MOTORS_DIR_PIN,OUTPUT);        // DIR MOTORS
  pinMode(MOTORS_ENABLE_PIN,OUTPUT);     // ENABLE MOTORS
  pinMode(MOTOR1_STEP_PIN,OUTPUT);       // STEP MOTOR1
  pinMode(MOTOR2_STEP_PIN,OUTPUT);       // STEP MOTOR2
  pinMode(BATTERY_PIN,INPUT);            // BATTERY
  pinMode(RED_LED_PIN,OUTPUT);           // RED LED
  pinMode(GREEN_LED_PIN,OUTPUT);         // GREEN LED
  pinMode(BLUE_LED_PIN,OUTPUT);          // BLUE LED 
  digitalWrite(MOTORS_ENABLE_PIN,HIGH);  // Деактивируем моторы
  
  Serial.begin(SERIAL_SPEED);            // Запускаем последовательный интерфейс на консоль
  Wire.begin();                          // Запускаем I2C шину на скорости 400Khz  
  TWSR = 0;
  TWBR = ((16000000L/I2C_SPEED)-16)/2;
  TWCR = 1<<TWEN;
  // Init servos
#if DEBUG==9
  Serial.println(F("Servo initialization"));
#endif
  eX_Servo_LED_Init();
  eX_Set_Servo(SERVO_AUX_NEUTRO);
#if DEBUG==9
  Serial.println(F("Steper motors initialization"));
#endif
  eX_Motors_Sonar_Init();
#if DEBUG==9
  Serial.println(F("Initialization I2C devices"));
#endif
  if(!eX_WiFi_Init())                                   // Запуск WiFi модуля
  {
    eX_Set_Color_LED(5,BLUE,BLUE,BLUE,RED,15,1);
  }
  else
  {
    eX_Set_Color_LED(1,BLUE,BLUE,BLUE,GREEN,15,1);
  }

   // Manual MPU initialization... 
   // accel=2G, gyro=2000º/s, filter=20Hz BW, output=200Hz
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_10);              //10,20,42,98,188  // Default factor for BROBOT:10
  mpu.setRate(4);                                   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);
#if DEBUG==9
  Serial.println(F("Initialization DMP"));
#endif
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    eX_Set_Color_LED(1,VIOLET,VIOLET,VIOLET,GREEN,15,1);
  } 
  else 
  { 
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    eX_Set_Color_LED(5,VIOLET,VIOLET,VIOLET,RED,15,1);
#if DEBUG==9
    Serial.print(F("ERROR! (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
#endif
  }

  // Калибровка гироскопов
  // Робот должен быть неподвижным на время инициализации
#if DEBUG==9
  Serial.print(F("Gyro calibration!  Don't move the robot in 10 seconds..."));
#endif
  eX_Set_Color_LED(5,YELLOW,BLACK,YELLOW,BLACK,45,1);
#if DEBUG==9
  Serial.print(F("Free RAM: "));
  Serial.println(eX_Free_Ram());
  Serial.print(F("Max_throttle:     "));
  Serial.println(max_throttle);
  Serial.print(F("Max_steering:     "));
  Serial.println(max_steering);
  Serial.print(F("Max_target_angle: "));
  Serial.println(max_target_angle);
#endif
  // Verify connection
#if DEBUG==9
  Serial.print(F("Testing MPU6050 connections..."));
#endif
  if (mpu.testConnection())
    eX_Set_Color_LED(1,YELLOW,YELLOW,YELLOW,GREEN,15,1);
  else
    eX_Set_Color_LED(5,YELLOW,YELLOW,YELLOW,RED,15,1);
#if DEBUG==9
  Serial.println(mpu.testConnection() ? " OK!" : " Failed!");
  //Настройка уровня реакции датчика
  Serial.println(F("Adjusting DMP sensor fusion gain"));
#endif
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0x20);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
  digitalWrite(MOTORS_ENABLE_PIN,LOW);
// Незначительная вибрация моторами и сервоприводом для контроля их готовностиe
for (uint8_t k=0;k<5;k++)
  {
    eX_Set_Motors_Speed(3,3);
    eX_Set_Servo(SERVO_AUX_NEUTRO+100);
    delay(200);
    eX_Set_Motors_Speed(-3,-3);
    eX_Set_Servo(SERVO_AUX_NEUTRO-100);
    delay(200);
  }
  eX_Set_Motors_Speed(0,0);
  eX_Read_Battery();

  // OSC initialization
  fadder1 = 0.5;
  fadder2 = 0.5;
  eX_Set_Color_LED(3,BLACK,BLUE,RED,GREEN,20,0);
#if DEBUG==9
  Serial.println("Let's start...");
#endif
  mpu.resetFIFO();
  timer_old = millis();
  Robot_shutdown = false;
  mode = 0;
  digitalWrite(MOTORS_ENABLE_PIN,HIGH);
}



void loop() {
  // If we run out of , we do nothing... STOP
#if SHUTDOWN_WHEN_BATTERY_OFF==1
  if (Robot_shutdown)
    return;
#endif
  debug_counter++;
  eX_WiFi_MSG_Read();                               // Чтение из порта UDP одного байта из входящего OSC сообщения. 
   //Если на N-м обращении к процедуре сообщение наконец-то дочитано до конца, команда и её параметры распознаны, то выставляется флаг newMessage=1
  if (newMessage)
  {
    newMessage = 0;
    if (page==1)                                  // Получена команда управления от пользователя(throttle, steering...)
    {
      if ((fadder1 > 0.45)&&(fadder1<0.55))       // Малые скорости игнорируем
        throttle = 0;
      else
        throttle = (fadder1-0.5)*max_throttle;
                                                 // Для малых скоростях поворота вводим экспоненциальную зависимость
      steering = fadder2-0.5;
      if (steering>0)
        steering = (steering*steering+0.5*steering)*max_steering;
      else
        steering = (-steering*steering+0.5*steering)*max_steering;
      modifing_control_parameters=false;         // устанавливаем флаг необходимости модижфцировать параметры управления
      if ((mode==0)&&(toggle1))
      {
        // Change to PRO mode
        max_throttle = MAX_THROTTLE_PRO;
        max_steering = MAX_STEERING_PRO;
        max_target_angle = MAX_TARGET_ANGLE_PRO;
        mode = 1;    
      }
      if ((mode==1)&&(toggle1==0))
      {
        // Change to NORMAL mode
        max_throttle = MAX_THROTTLE;
        max_steering = MAX_STEERING;
        max_target_angle = MAX_TARGET_ANGLE;
        mode = 0;
      }
    }
  }

  timer_value = millis();

  // New DMP Orientation solution?
  fifoCount = mpu.getFIFOCount();
  if (fifoCount>=18)
  {
    if (fifoCount>18)  // If we have more than one packet we take the easy path: discard the buffer 
    {
//      Serial.println(F("FIFO RESET!"));
      mpu.resetFIFO();
      return;
    }
    loop_counter++;
    slow_loop_counter++;
    dt = (timer_value-timer_old);
    timer_old = timer_value;

    angle_adjusted_Old = angle_adjusted;
    // Get new orientation angle from IMU (MPU6050)
    angle_adjusted = dmpGetPhi();

#if DEBUG==8
    Serial.print(throttle);
    Serial.print(" ");
    Serial.print(steering);
    Serial.print(" ");
    Serial.println(mode);
#endif

    //angle_adjusted_radians = angle_adjusted*GRAD2RAD;
#if DEBUG==1
    Serial.println(angle_adjusted);
#endif
    mpu.resetFIFO();  // We always reset FIFO

      // Мы рассчитываем расчетную скорость робота
      // Расчетная скорость = угловая скорость шаговых двигателей (в сочетании) - угловая скорость робота (угол измеряется IMU)
    actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_M1 + speed_M2)/2;                                           // Positive: forward
    int16_t angular_velocity = (angle_adjusted-angle_adjusted_Old)*90.0;                    // 90 эмпирический коэффициент полученный при корректировке реальных показателей
    int16_t estimated_speed = -actual_robot_speed_Old - angular_velocity;                   // We use robot_speed(t-1) or (t-2) to compensate the delay
    estimated_speed_filtered = estimated_speed_filtered*0.95 + (float)estimated_speed*0.05;

#if DEBUG==2
    Serial.print(" ");
    Serial.print(estimated_speed_filtered);
    Serial.print(" ");
    Serial.print(Kp_thr);
    Serial.print(" ");
    Serial.println(Ki_thr);
#endif
    // SPEED CONTROL: This is a PI controller. 
    // input: user throttle
    // variable: estimated robot speed
    // output: target robot angle to get the desired speed
    //target_angle = target_angle*0.3 + speedPIControl(dt,estimated_speed_filtered,throttle,Kp_thr,Ki_thr)*0.7;   // Some filtering 
    target_angle = speedPIControl(dt,estimated_speed_filtered,throttle,Kp_thr,Ki_thr); 
    target_angle = constrain(target_angle,-max_target_angle,max_target_angle);                                   // limited output

#if DEBUG==3
    Serial.print(" ");
    Serial.print(estimated_speed_filtered);
    Serial.print(" ");
    Serial.println(target_angle);
#endif

    // Stability control: This is a PD controller. 
    // input: robot target angle(from SPEED CONTROL)
    // variable: robot angle
    // output: Motor speed
    // Мы интегрируем реакцию (суммируя), так что на выходе имеем ускорение двигателя, а не его скорость вращения.
    control_output += stabilityPDControl(dt,angle_adjusted,target_angle,Kp,Kd);  

    // Limit max output from control
  //  control_output = constrain(control_output,-MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT);

    // Вводим пользовательские корректировки по рулению в сигнал управления
    motor1 = control_output + steering;
    motor2 = control_output - steering;

    // Limit max speed (control output)
    motor1 = constrain(motor1,-MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT);   
    motor2 = constrain(motor2,-MAX_CONTROL_OUTPUT,MAX_CONTROL_OUTPUT);
#if DEBUG==4
    Serial.print("motor1 ");
    Serial.print(motor1);
    Serial.print("motor2 ");
    Serial.println(motor2);
#endif
    // NOW we send the commands to the motors
    if ((angle_adjusted<74)&&(angle_adjusted>-74))  // Робот в рабочем положении?
    {
      // NORMAL MODE
      digitalWrite(MOTORS_ENABLE_PIN,LOW);   // Motors enable
      eX_Set_Motors_Speed(motor1,motor2);

      // Push1 Move servo arm
      if (push1)  // Move arm
        eX_Set_Servo(SERVO_MIN_PULSEWIDTH+100);
      else
        eX_Set_Servo(SERVO_AUX_NEUTRO);

      // Push2 reset controls to neutral position
      if (push2)
      {
        fadder1 = 0.5;
        fadder2 = 0.5;
      }

    // Normal condition?
      if ((angle_adjusted<40)&&(angle_adjusted>-40))
      {
        Kp = Kp_user;            // Получает контроль пользователя по умолчанию
        Kd = Kd_user; 
        Kp_thr = Kp_thr_user;
        Ki_thr = Ki_thr_user;
      }     
      else    // Во время поднятия робота в рабочее положение мы используем специальные параметры контроля
      {
        Kp = KP_RAISEUP;         // CONTROL GAINS FOR RAISE UP
        Kd = KD_RAISEUP;
        Kp_thr = KP_THROTTLE_RAISEUP; 
        Ki_thr = KI_THROTTLE_RAISEUP;
      }   
    }
    else   // Robot not ready (flat), angle > 70º => ROBOT OFF
    {
      digitalWrite(MOTORS_ENABLE_PIN,HIGH);   // Disable motors
      eX_Set_Motors_Speed(0,0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;

      // if we pulse push1 button we raise up the robot with the servo arm
      if (push1)
      {
        // Because we know the robot orientation (face down of face up), we move the servo in the appropiate direction for raise up
        if (angle_adjusted>0)
          eX_Set_Servo(SERVO_MIN_PULSEWIDTH);
        else
          eX_Set_Servo(SERVO_MAX_PULSEWIDTH);
      }
      else
        eX_Set_Servo(SERVO_AUX_NEUTRO);

    }
    // Check for new user control parameters
    readControlParameters();

  } // End of new IMU data

  // Medium loop 40Hz
  if (loop_counter >= 5) 
  {
    loop_counter = 0;
    // We do nothing here now...
  } // End of medium loop

  if (slow_loop_counter>=990)
  {
    slow_loop_counter = 0;
    eX_Read_Battery();
#if DEBUG==5
    Serial.print(F("Distance: "));
    Serial.println(Distance);
#endif
    // Read  status
#if DEBUG==6
    Serial.print(F("Level BAT: "));
    Serial.println(BattLevel);
#endif
#if BATTERY_CHECK==1
    if (BattLevel < BATTERY_SHUTDOWN)
    {
      eX_Set_Color_LED(1,RED,0,0,0,1,0);
      // Robot shutdown !!!
  #if SHUTDOWN_WHEN_BATTERY_OFF==1
      Serial.println(F("LOW BAT!! SHUTDOWN")); 
      Robot_shutdown = true;
      // Disable steppers
      digitalWrite(MOTORS_ENABLE_PIN,HIGH);   // Disable motors
  #endif  
    }
    else if (BattLevel < BATTERY_WARNING)
    {
      eX_Set_Color_LED(1,YELLOW,0,0,0,1,0);
      // Battery warning
      // What to do here???
      Serial.print(F("LOW BAT!! "));
      Serial.println(BattLevel);
      //eX_Set_Servo(SERVO_AUX_NEUTRO+300);  // Move arm?
    }
    else
     eX_Set_Color_LED(1,GREEN,0,0,0,1,0);
#endif   // BATTERY_CHECK
  }  // End of slow loop
}


