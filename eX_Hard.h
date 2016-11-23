#ifndef EX_HARD_H
#define EX_HARD_H

#include "eX_Pins.h"
#include "eX_Configuration.h"

///  Переменные подпрограмм управления моторами ///
int16_t    speed_M1, speed_M2;        // текущая скорость вращения моторов
uint16_t   period_M1, period_M2;      // текущий период вращения моторов
int8_t     dir_M1, dir_M2;            // текущее направление вращения моторов

///  Переменные подпрограмм управления ультразвуковым дальномером ///
uint16_t   SonarStart = 0;
uint16_t   SonarEnd   = 0;
uint16_t   SonarValue = 0;
float      Distance   = 0;

///  Переменные подпрограмм измерения уровня заряда батареи ///
uint16_t   tBattLevel = 815;
float      BattLevel  = 4.1;

///  Переменные подпрограмм световой индикации состояния одним RGB светодиодом ///
/*
Светодиод управляется 4-мя фазами по 0.5 сек. и использует следующий формат LigthState:

бит         интерпретация
===         =============
 F          флаг непрерывной работы (счетчик циклов игнорируется, если 1)
E-C         счетчик циклов (до 8 циклов)
B-9         байты активности соответсвующих светодиодов RGB в фазе 3 (0 - OFF, 1 - ON)
8-6         байты активности соответсвующих светодиодов RGB в фазе 2 (0 - OFF, 1 - ON)
5-3         байты активности соответсвующих светодиодов RGB в фазе 1 (0 - OFF, 1 - ON)
2-0         байты активности соответсвующих светодиодов RGB в фазе 0 (0 - OFF, 1 - ON)
*/
uint16_t   LigthState;
int8_t     show_led_counter  = 0;
int8_t     phase_led_counter = 0;
int8_t     timing_led        = 60;


 // Для управления моторами, сервоприводом и обслуживания сонара используем timer1:
 // D4        ->  PD4 (ICP1/ADC8)                : SONAR Echo
 // D9        ->  PB5 (PCINT5/OC1A/!OC4B/ADC12)  : SONAR Trigger (длительность положительного импульса 10 микросекунд)
 // D10       ->  PB6 (PCINT6/OC1B/OC4B/ADC13)   : SERVO (длительность импульса изменяется от SERVO_MIN_PULSEWIDTH до SERVO_MAX_PULSEWIDTH в микросекундах)
 // D5        ->  PC6 (OC3A/!OC4A)               : DIR  Motor1 & Motor2
 // D6        ->  PD7 (T0/OC4D/ADC10)            : ENABLE Motors
 // D7        ->  PE6 (INT.6/AIN0)               : STEP Motor1
 // D8        ->  PB4 (PCINT4/ADC11)             : STEP Motor2

#define CLR(x,y)             (x&=(~(1<<y)))
#define SET(x,y)             (x|=(1<<y))


void delay_1us()  
{
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
}


//  Функция инициализации таймера обеспечивающего работу моторов, сервопривода и сонара
void eX_Motors_Sonar_Init(void)
{ 
  dir_M1 = 0;                            // Выставляем dir_Mx в ноль, 
  dir_M2 = 0;                            // что блокирует подачу импульсов step на шаговые моторы
//  cli(); 		                 // Запрещаемаем прерывания.
  TCCR1A = 0;                            // Устанавливаем норм. режим со сбросом счетчика по переполнению (WGM1x=0000) и отключенными выходами OC1A, OC1B и OC1C (COMnx1:0=00)
  TCCR1B = B00000010;                    // Предделение = 8 (CS1x=010), при этом частота на входе таймера 2 МГц (период 0,5 мксек)
  OCR1A  = ZERO_SPEED;                   // Максимальная длительность периода управления моторами как команда отсановки мотора1
  OCR1C  = ZERO_SPEED;                   // Максимальная длительность периода управления моторами как команда отсановки мотора2
  TCNT1  = 0;                            // Сбрсываем счетчик timer1
  OCR1B  = TCNT1 + TIMING_TRIG_PULSE;    // Длительность импульса запуска сонара 10 мксек = 20 * 0,5 мксек
  ICR1   = 0;                            // Обнуляем регистр захвата значения таймера по внешнему событию
  TIMSK1 = B00001111;                    // Разрешаем прерывания от timer1 по переполнению и трём совпадениям
//  sei(); 		                 // Разрешаем прерывания.
  SET(PORTB,5);                          // Запускамем фронт trig-импульса, аналог digitalWrite(SONAR_TRIG_PIN, HIGH);
  SET(PORTD,7);                          // Деактивируем моторы, аналог digitalWrite(MOTORS_ENABLE_PIN,HIGH);
}

///     Обслуживание прерываний (моторная часть)   ///

ISR(TIMER1_OVF_vect)                     // прерывание наступает по переполнению timer1
{
  if(!SonarEnd==0)
  {
    SonarValue = SonarEnd - SonarStart;
    Distance   = (Distance + SonarValue/116)/2;
  }
  SonarStart = 0;
  SonarEnd = 0;
  TIMSK1 = B00001111;                      // запрещаем прерывание по любому типу внешнего события
  OCR1B  = TCNT1 + TIMING_TRIG_PULSE;      // выставляем задержку на 10 мксек относительно времени начала обработки прерывания
  SET(PORTB,5);                            // Выставляем фронт запускающего сонар импульса, аналог digitalWrite(SONAR_TRIG_PIN, HIGH); 
}


ISR( TIMER1_CAPT_vect )                  // прерывание наступает по изменению уровня сигнала на входе SONAR_ECHO_PIN
{
  if (TCCR1B & B01000000)                // если внешнее событие ФРОНТ импульса
  {
    SonarStart = ICR1;                   // сохранияем стартовое значение счетчика
    ICR1 = 0; 
    TCCR1B = B10000010;                  // устанавливаем вид внешнего прерывания по СПАДу импульса
  }
  else
  {
    SonarEnd = ICR1;
    ICR1 = 0;
    TIMSK1 = B00001111;                  // запрещаем прерывание по любому типу внешнего события
  }
}

ISR(TIMER1_COMPA_vect)                   // прерывание наступает по истечении времени между импульсами step управления шаговым мотором 1
{
  if (dir_M1==0)                         // Если нет движения шаговый импульс не генерировать
    return;
  if (dir_M1>0)
  {
    SET(PORTC,6);                        // DIR Motors (Forward for Motor1)
  }
  else
  {
    CLR(PORTC,6);                        // DIR Motors (Revers for Motor1)
  }
  OCR1A  = OCR1A + period_M1;
    __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
  SET(PORTE,6);                          // STEP MOTOR 1 (см. таблицу в начале для PINOUT D7)
  delay_1us();  
  CLR(PORTE,6);
}

ISR( TIMER1_COMPC_vect )                 // прерывание наступает по истечении времени между импульсами step управления шаговым мотором 2
{
  if (dir_M2==0)                         // Если нет движения шаговый импульс не генерировать
    return;
  if (dir_M2>0)
  {
    CLR(PORTC,6);                        // DIR Motors (Revers for Motor2)
  }
  else
  {
    SET(PORTC,6);                        // DIR Motors (Forward for Motor2)
  }
  OCR1C  = OCR1C + period_M2;
      __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
  SET(PORTB,4);                          // STEP MOTOR 2 (см. таблицу в начале для PINOUT D8)
  delay_1us();
  CLR(PORTB,4);
}

ISR( TIMER1_COMPB_vect )                 // прерывание наступает по истечении времени импульса запуска сонара
{
  CLR(PORTB,5);                          // Снимаем запускающий сонар импульс, аналог digitalWrite(SONAR_TRIG_PIN, LOW);
  TCCR1B = B11000010;                    // Устанавливаем тип прерывания по ФРОНТу импульса внешнего события
  TIMSK1 = B00101111;                    // Разрешаем прерывание по внешнему событию
}

// Установка скорости вращения моторов
// tspeed_Mx может принимать как положительные, так и отрицательные значения (reverse)
void eX_Set_Motors_Speed(int16_t tspeed_M1, int16_t tspeed_M2)
{ 
  long      timer_period_1, timer_period_2;
  int16_t   speed_1, speed_2;
  int8_t    dir_1, dir_2;
  // Лимитируем изменение скорости с учётом максимального ускорения для motor1
  if ((speed_M1 - tspeed_M1)>MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed_M1)<-MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed_M1;
    
  // Лимитируем изменение скорости с учётом максимального ускорения для motor2
  if ((speed_M2 - tspeed_M2)>MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed_M2)<-MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed_M2;

#if MICROSTEPPING==16
  speed_1 = speed_M1*46;   // Adjust factor from control output speed to real motor speed in steps/second
  speed_2 = speed_M2*46;
#else
  speed_1 = speed_M1*23;   // 1/8 Microstepping
  speed_2 = speed_M2*23;
#endif
// Расчёт периода для motor1
  if (speed_1==0)
  {
    timer_period_1 = ZERO_SPEED;
    dir_1 = 0;
  }
  else if (speed_1>0)
  {
    timer_period_1 = 2000000/speed_1;   // 2Mhz timer
    dir_1 = 1;
  }
  else if (speed_1<0)
    {
      timer_period_1 = 2000000/-speed_1;
      dir_1 = -1;
    }
  if (timer_period_1 > 65535)          // Check for minimun speed (maximun period without overflow)
    timer_period_1 = ZERO_SPEED;
// Расчёт периода для motor2
  if (speed_2==0)
  {
    timer_period_2 = ZERO_SPEED;
    dir_2 = 0;
  }
  else if (speed_2>0)
  {
    timer_period_2 = 2000000/speed_2;   // 2Mhz timer
    dir_2 = 1;
  }
  else if (speed_2<0)
    {
      timer_period_2 = 2000000/-speed_2;
      dir_2 = -1;
    }
  if (timer_period_2 > 65535)          // Check for minimun speed (maximun period without overflow)
    timer_period_2 = ZERO_SPEED;
  dir_M1 = dir_1;
  dir_M2 = dir_2;
  period_M1 = timer_period_1;
  period_M2 = timer_period_2;
}

void eX_Servo_LED_Init()
{
  int temp;
  // Initialize Timer4 as Fast PWM
  TCCR4A = 1<<PWM4B;                   // Инициируем выход PB6 (Pin10)
//  TCCR4A = (1<<PWM4A)|(1<<PWM4B);    // или два выхода: PB6 (Pin10) и PC7 (Pin13)
  TCCR4B = 0;
  TCCR4C = 0;
  TCCR4D = 0;                          // WGM41=WGM40=0 устанавливают режим FastPWM со сбросом по OCR4C
  TCCR4E = (1<<ENHC4);                 // повышаем разрядность таймера до 10 бит

  temp = 1500>>3;
  TC4H = temp >> 8;
  OCR4B = temp & 0xff;

  // Reset timer
  TC4H = 0;
  TCNT4 = 0;

  // Set TOP to 1023 (10 bit timer)
  TC4H = 3;
  OCR4C = 0xFF;

  TCCR4A |= 1<<COM4B1;                 // задаем реакция установки выхода в 0 по совпадению с OC4B
//  TCCR4A |= (1<<COM4B1)|(1<<COM4A1); // Активируем выводы по совпадению с OC4B и с OC4A
  TCCR4B = (1 << CS43)|(1 << CS40);    // set prescaler to 256 and enable timer    16Mhz/256/1024 = 61Hz (16.3ms)
  TIMSK4 = 1<<TOIE4;                   // разрешаем прерывание по переполнению счётчика
} 

ISR(TIMER4_OVF_vect)
{
  uint16_t temp_led_state;
  
  show_led_counter++;
  if (show_led_counter >= timing_led) 
  {
    show_led_counter = 0;
    phase_led_counter++;
    if(phase_led_counter > 3)                // Все фазы предыдущего цикла показаны, то запускаем следующий цикл
    {
      phase_led_counter = 0;
      if((LigthState & 0xf000) > 0)          // Если старшие 4 бита (счётчик повторов) не 0... 
      {
        LigthState = LigthState - 0x1000;    //... то уменьшаем счетчик циклов на единицу...
        if ((LigthState & 0xf000) == 0)
          if(COLOR_INV)                      //...  и выключаем все светодиоды, если счетчик циклов обнулён
            LigthState = 0x0fff;
          else
            LigthState = 0;
      }
    } // end if(phase_led_counter > 3)
    temp_led_state = LigthState & 0x0fff;
    temp_led_state = (temp_led_state >> (3*phase_led_counter)) & 0x0007;
    PORTF = (PORTF & 0x8f) | (temp_led_state <<4);
  }
}

void eX_Set_Servo(int pwm)
{
  pwm = constrain(pwm,SERVO_MIN_PULSEWIDTH,SERVO_MAX_PULSEWIDTH)>>3;  // Check max values and Resolution: 8us
  // 11 bits => 3 MSB bits on TC4H, LSB bits on OCR4B
  TC4H = pwm>>8;
  OCR4B = pwm & 0xFF;
}

void eX_Set_Color_LED(int8_t cycle, int8_t c_ph0, int8_t c_ph1, int8_t c_ph2, int8_t c_ph3, int8_t timing, int8_t waiting)
{
  uint16_t LedState = cycle;
  if(COLOR_INV)
  {
    LedState = (LedState << 3) + (c_ph3 ^ 0x07);
    LedState = (LedState << 3) + (c_ph2 ^ 0x07);
    LedState = (LedState << 3) + (c_ph1 ^ 0x07);
    LedState = (LedState << 3) + (c_ph0 ^ 0x07);
  }
  else
  {
    LedState = (LedState << 3) + c_ph3;
    LedState = (LedState << 3) + c_ph2;
    LedState = (LedState << 3) + c_ph1;
    LedState = (LedState << 3) + c_ph0;
  }
  LigthState = LedState;
  timing_led = timing;
  phase_led_counter = 0;
  if(waiting) delay(66*timing*cycle);   // 66 > 1024*256/16000000 
}

void eX_Set_Color(int8_t color, int8_t pulse)
{
  if(pulse)
    eX_Set_Color_LED(0, color, 0, color, 0, 30,0);
  else
    eX_Set_Color_LED(0, color, 0, color, 0, 60,0);
}

float eX_Read_Battery()  
// Определяем напряжения батареи, подключенной на аналоговый вход A0(ADC7) через делитель напряжения ~1:2
// Опорное напряжение со встроенного источника Vref = 2.56В
{
  BattLevel = (BattLevel*4 + (tBattLevel/202.86))/5;     // коэффициент 5.043 получен как 2.56 * коэфициент резистивного делителя напряжения
  ADMUX  = (1<<REFS0) | (1<<REFS1);                      // источником опорного напряжениея выбран встроенный источник Vref = 2.56В
  ADMUX  |= (1<<MUX2) | (1<<MUX1) | (1<<MUX0);           // задаем порт измерения ADC7 (Pin A0)
  ADCSRA |= (1<<ADSC) | (1<<ADIE);                       // запускаем измерение и разрешаем прерывание
  return BattLevel;
}

ISR(ADC_vect)                    // прерывание по готовности данных в АЦП
{
  tBattLevel = ADCL;                         // необходимо считать ADCL прежде, что разблокирует ADCH
  tBattLevel |= ADCH<<8;
}

#endif //EX_HARD_H
