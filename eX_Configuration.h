#ifndef EX_CONFIGURATION_H
#define EX_CONFIGURATION_H

// Режим отладки (вывод информации в последовательный порт Serial)
// ---------------------------------------------------------------
// DEBUG 1 - вывод информации о angle_adjusted
// DEBUG 2 - вывод информации о estimated_speed_filtered
// DEBUG 3 - вывод информации о angle_target
// DEBUG 4 - вывод информации о 
// DEBUG 5 - вывод информации о distance_sensor и autonomous_mode_status в mode==1
// DEBUG 6 - вывод информации о battery
// DEBUG 7 - вывод информации о distance_sensor и autonomous_mode_status
// DEBUG 8 - вывод информации о throttle, steering и mode
// DEBUG 9 - вывод сообщений инициализации устройств

#define  DEBUG                     0

// Определяем константы цветов RGB светодиодов
#define BLACK      0     // RGB=000
#define BLUE       1     // RGB=001
#define GREEN      2     // RGB=010
#define CYAN       3     // RGB=011
#define RED        4     // RGB=100
#define VIOLET     5     // RGB=101
#define YELLOW     6     // RGB=110
#define WHITE      7     // RGB=111
#define COLOR_INV  1     // 1 - общий анод, 0 - общий катод

/// Выбираем частоту шины I2C и UART's///
#define  I2C_SPEED                  400000L
#define  SERIAL_SPEED               115200
#define  SERIAL1_SPEED              115200

// Параметры источника питания (батареи)
//============= start ================//
#define  SHUTDOWN_WHEN_BATTERY_OFF    0             // 0: Not used, 1: Robot will shutdown
#define  BATTERY_CHECK                1
#define  BATTERY_WARNING              3.6
#define  BATTERY_SHUTDOWN             3.3
//============== end =================//


///     Параметры управляемости      ///
//============= start ================//
#define  ZERO_SPEED                 65535
#define  MAX_ACCEL                  8
#define  MAX_CONTROL_OUTPUT         500
#define  MICROSTEPPING              8

// NORMAL MODE = smooth, moderately
#define  MAX_THROTTLE               480      // дроссель, скорость
#define  MAX_STEERING               130      // руление, поворачиваемость
#define  MAX_TARGET_ANGLE           12       // угол наклона

// PRO MODE = more aggressive
#define  MAX_THROTTLE_PRO           680
#define  MAX_STEERING_PRO           250 
#define  MAX_TARGET_ANGLE_PRO       20
//============== end =================//


///    Параметры модуля MPU6050      ///
//============= start ================//
#define  ACCEL_SCALE_G              8192             // (2G range) G = 8192
#define  ACCEL_WEIGHT               0.01
#define  GYRO_BIAS_WEIGHT           0.005
// MPU6000 sensibility   (0.0609 => 1/16.4LSB/deg/s at 2000deg/s, 0.03048 1/32.8LSB/deg/s at 1000deg/s)
#define  Gyro_Gain                  0.03048
#define  Gyro_Scaled(x)             x*Gyro_Gain      //Return the scaled gyro raw data in degrees per second
#define  RAD2GRAD                   57.2957795
#define  GRAD2RAD                   0.01745329251994329576923690768489
//============== end =================//


///         PID-параметры            ///
//============= start ================//
// Условия управления по умолчанию 
#define  KP                         0.19    // альтернативные значения: 0.20, 0.22        
#define  KD                         30      // 26 28        
#define  KP_THROTTLE                0.07    // 0.065, 0.08
#define  KI_THROTTLE                0.04    // 0.05
// Прирост управления при поднятии робота из лежачего положения
#define  KP_RAISEUP                 0.16
#define  KD_RAISEUP                 40
#define  KP_THROTTLE_RAISEUP        0         // При поднятии скорость моторов не контролируется
#define  KI_THROTTLE_RAISEUP        0.0
//============== end =================//


/// Параметры серво-привода ///
//============= start ================//
#define  SERVO_AUX_NEUTRO           1470     // нейтральное положение серво-привода
#define  SERVO_MIN_PULSEWIDTH       800
#define  SERVO_MAX_PULSEWIDTH       2200
//============== end =================//

#define  ITERM_MAX_ERROR            40   // Iterm windup constants
#define  ITERM_MAX                  5000
#define  RAD2GRAD                   57.2957795
#define  GRAD2RAD                   0.01745329251994329576923690768489

///  Параметры дальномера ///
//============= start ================//
#define  OBSTACLE_DISTANCE_MIN      35     // Минимальное расстояние до препятствия в мм ???
#define  WALK_DISTANCE_MIN          100    // Минимальное расстояние до препятствия, при котором робот может "упасть лицом вниз"
#define  TIMING_DISTANCE_MAX        41000  // в тактах по 0,5 мксек
#define  TIMING_TRIG_PULSE          20     // определяет длительность импульса сонара равную 10 мксек в периодах по 0.5 мксек
//============== end =================//

#endif // EX_CONFIGURATION_H
