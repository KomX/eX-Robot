#ifndef EX_WIFI_H
#define EX_WIFI_H

#define DST_IP        "192.168.4."     // удалённый хост
String  DST_IP_FULL = "";              // удалённый хост (полный адрес)

boolean eX_WiFi_Init()
{
  Serial1.begin(SERIAL1_SPEED);
//  Serial.println("Init WiFi");
  Serial1.println("AT+RST");    // сброс и проверка, если модуль готов
  delay(5000);
  if(!Serial1.find("OK"))
  {
//   Serial.println("Module dosn't respond.");
    return false;
  }
//  Serial.println("Use only AP mode");
  Serial1.println("AT+CWMODE=3");
  delay(1000);
  Serial1.println("AT+CIPMUX=1");
  delay(1000);
  Serial1.println("AT+CIPSTART=1,\"UDP\",\"192.168.4.101\",9000,8000,0");
  delay(1000);
  return true;
/*  boolean connected=false;
  Serial.print("Finding Host ");
  for(int i=0;i<99;i++)
  {
    Serial.print(".");
    Serial1.println("AT+CWLIF");
    delay(3000);
    if(Serial1.find(DST_IP)){
      connected = true;
      break;
    }
  }
  if (!connected){
    Serial.println(" Host dosn't respond.");
    return false;
  }
  Serial.println(" OK!");
  Serial.print("Connected to ");
  Serial.print(DST_IP);
  Serial.print(Serial1.read()-48);
  Serial.print(Serial1.read()-48);
  Serial.println(Serial1.read()-48);
  Serial1.println("AT+CIPMUX=1");
  delay(1000);
 // Serial1.println("AT+CIPSERVER=1,8000");
 // delay(1000);
  Serial1.println("AT+CIPSTART=1,\"UDP\",\"192.168.4.101\",9000,8000,0");
  delay(1000);
  */
};







//#include "Arduino.h"

/*
// UPD input buffer
char UDPBuffer[8];
uint8_t rx_bufferS1_overflow = 0;

// OSC message read variables
unsigned char readStatus;
unsigned char readCounter;
unsigned char readNumParams;
unsigned char commandType;
float         extractParamFloat1();
float         extractParamFloat2();

// Controls
uint8_t   page;
uint8_t   newMessage;
float     fadder1;
float     fadder2;
float     fadder3;
float     fadder4;
float     xy1_x;
float     xy1_y;
float     xy2_x;
float     xy2_y;
uint8_t   push1;
uint8_t   push2;
uint8_t   push3;
uint8_t   push4;
uint8_t   toggle1;
uint8_t   toggle2;
uint8_t   toggle3;
uint8_t   toggle4;

#define SERIAL_BUFFER_SIZE 64

struct ring_buffer
{
  unsigned char buffer[SERIAL_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
};
ring_buffer rx_bufferS1  =  { { 0 }, 0, 0 };

inline void store_char(unsigned char c, ring_buffer *buffer)
{
  int i = (unsigned int)(buffer->head + 1) % SERIAL_BUFFER_SIZE;

  // Always store new byte on ring buffer.
  buffer->buffer[buffer->head] = c;
  buffer->head = i;
  //  Overflow? => Discard old bytes
  if (i == buffer->tail) {
    buffer->tail = (unsigned int)(buffer->tail + 1) % SERIAL_BUFFER_SIZE;  // Move tail to discard old bytes
	rx_bufferS1_overflow = 1;                                          // Buffer overflow flag
    }
}

ISR(USART1_RX_vect)
  {
    if (bit_is_clear(UCSR1A, UPE1)) {
      unsigned char c = UDR1;
      store_char(c, &rx_bufferS1);
    } else {
      unsigned char c = UDR1;
    };
  }
  
int Serial1_available(void)
{
  return (unsigned int)(SERIAL_BUFFER_SIZE + rx_bufferS1.head - rx_bufferS1.tail) % SERIAL_BUFFER_SIZE;
}

unsigned char Serial1_read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if (rx_bufferS1.head == rx_bufferS1.tail) {
    return -1;
  } else {
    unsigned char c = rx_bufferS1.buffer[rx_bufferS1.tail];
    rx_bufferS1.tail = (unsigned int)(rx_bufferS1.tail + 1) % SERIAL_BUFFER_SIZE;
    return c;
  }
}

void Serial1_begin(unsigned long baud)
{
  uint16_t baud_setting;
  
  UCSR1A = (1 << U2X1);
  baud_setting = (F_CPU / 4 / baud - 1) / 2;
  
  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  UBRR1H = baud_setting >> 8;
  UBRR1L = baud_setting;

  //transmitting = false;
  UCSR1B = (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
  
  //sbi(UCSR1B, RXEN1);
  //sbi(UCSR1B, TXEN1);
  //sbi(UCSR1B, RXCIE1);
  //cbi(UCSR1B, UDRIE1);
} 

float extractParamFloat1(){
union{
  unsigned char Buff[4];
  float d;
}u;
 
  u.Buff[0] = (unsigned char)UDPBuffer[0];
  u.Buff[1] = (unsigned char)UDPBuffer[1];
  u.Buff[2] = (unsigned char)UDPBuffer[2];
  u.Buff[3] = (unsigned char)UDPBuffer[3];
  return(u.d); 
}
float extractParamFloat2(){
union{
  unsigned char Buff[4];
  float d;
}u;
 
  u.Buff[0] = (unsigned char)UDPBuffer[4];
  u.Buff[1] = (unsigned char)UDPBuffer[5];
  u.Buff[2] = (unsigned char)UDPBuffer[6];
  u.Buff[3] = (unsigned char)UDPBuffer[7];
  return(u.d); 
}

void WiFi_MSG_Read()
{
  uint8_t i;
  uint8_t tmp;
  float value;
  float value2;
  // Overflow on rx buffer?
  if (rx_bufferS1_overflow) {
	// We lost old bytes so we need to reset the parser and discard old bytes until we reach a message start character '/'
	readStatus = 0;
	rx_bufferS1_overflow = 0;
	while (Serial1_available())
	  {
	  tmp = Serial1_read();
	  if (tmp=='/') {
		UDPBuffer[0] = tmp;
		Serial.println("OSC:D");
		return;
		}
	  }
	}
  // New byteas available to process?
  if (Serial1_available() > 0) {
    //Serial.print("B:");
    //Serial.println(Serial1_available());
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (i=7;i>0;i--){
      UDPBuffer[i] = UDPBuffer[i-1];
    }
    UDPBuffer[0] = Serial1_read();
    #ifdef DEBUG3
        Serial.print(UDPBuffer[0]);
    #endif
    // We look for an OSC message start like /x/
    if ((UDPBuffer[0] == '/')&&(UDPBuffer[2] == '/')){
      if (readStatus == 0){
	    page = UDPBuffer[1] - '0';  // Convert page to int
        readStatus = 1;
        //Serial.print("$");
      }
      else{
        readStatus = 1;
        Serial.println("!ERR:osc");
      }
      return;
    } else if (readStatus==1){   // looking for the message type
      // Fadder    /1/fadder1 ,f  xxxx   
      if ((UDPBuffer[3] == 'd')&&(UDPBuffer[2] == 'e')&&(UDPBuffer[1] == 'r')){
        readStatus=2;    // Message type detected
        readCounter=11;  // Bytes to read the parameter
        readNumParams=1; // 1 parameters
        switch (UDPBuffer[0]){  // fadder number
          case '1':
            commandType = 1;
			#ifdef DEBUG2 
				Serial.print("$FAD1$");
			#endif
            break;
          case '2':
            commandType = 2;
			#ifdef DEBUG2 
				Serial.print("$FAD2$");
			#endif
            break;
          case '3':
            commandType = 3;
			#ifdef DEBUG2 
				Serial.print("$FAD3$");
			#endif
            break;
          case '4':
            commandType = 4;
			#ifdef DEBUG2 
				Serial.print("$FAD4$");
			#endif
            break;
        }
        return;
      } // end fadder
      // XY message
      if ((UDPBuffer[2] == 'x')&&(UDPBuffer[1] == 'y')){
        readStatus=2;    // Message type detected
        readCounter=14;  // Bytes to read the parameters
        readNumParams=2; // 2 parameters
        switch (UDPBuffer[0]){  // xy number
          case '1':
            commandType = 11;
			#ifdef DEBUG2 
				Serial.print("$XY1:");
			#endif
            break;
          case '2':
            commandType = 12;
			#ifdef DEBUG2 
				Serial.print("$XY2:");
			#endif
            break;
          default:
            commandType = 11;
			#ifdef DEBUG2 
				Serial.print("$XY:");
			#endif
            break;
        }
        return;
      }  // End XY message
      // Push message
       if ((UDPBuffer[3] == 'u')&&(UDPBuffer[2] == 's')&&(UDPBuffer[1] == 'h')){
        readStatus=2;    // Message type detected
        readCounter=10;  // Bytes to read the parameter
        readNumParams=1; // 1 parameters
        switch (UDPBuffer[0]){  // push number
          case '1':
            commandType = 21;
			#ifdef DEBUG2 
				Serial.print("$P1:");
			#endif
            break;
          case '2':
            commandType = 22;
			#ifdef DEBUG2 
				Serial.print("$P2:");
			#endif
            break;
          case '3':
            commandType = 23;
			#ifdef DEBUG2 
				Serial.print("$P3:");
			#endif
            break;
          case '4':
            commandType = 24;
			#ifdef DEBUG2 
				Serial.print("$P4:");
			#endif
            break;
        }
        return;
      } // end push
	  // Toggle message
       if ((UDPBuffer[3] == 'g')&&(UDPBuffer[2] == 'l')&&(UDPBuffer[1] == 'e')){
        readStatus=2;    // Message type detected
        readCounter=10;  // Bytes to read the parameter
        readNumParams=1; // 1 parameters
        switch (UDPBuffer[0]){  // push number
          case '1':
            commandType = 31;
            #ifdef DEBUG2 
				Serial.print("$T1:");
			#endif
            break;
          case '2':
            commandType = 32;
			#ifdef DEBUG2 
				Serial.print("$T2:");
			#endif
            break;
          case '3':
            commandType = 33;
			#ifdef DEBUG2 
				Serial.print("$T3:");
			#endif
            break;
          case '4':
            commandType = 34;
			#ifdef DEBUG2
				Serial.print("$T4:");
			#endif
            break;
        }
        return;
      } // end toggle
    } else if (readStatus==2){
      readCounter--;   // Reading counter until we reach the Parameter position
      if (readCounter<=0){
        readStatus=3;
        value = extractParamFloat1();
        readStatus=0;
		if ((value<0.0)||(value>1.0))
            {
            Serial.println("!ERR:f1!");
            return;
            }
        if (readNumParams==2){
          value2 = extractParamFloat2();
          if ((value2<0.0)||(value2>1.0))
            {
            Serial.println("!ERR:OSCf2!");
            return;
            }
        }
		newMessage = 1;
        //Serial.println(value);
        switch (commandType){
          case 1:
            fadder1 = value;
			#ifdef DEBUG 
                Serial.print("$F1:");
				Serial.println(fadder1);
			#endif
            break;
          case 2:
            fadder2 = value;
			#ifdef DEBUG
                Serial.print("$F2:");
				Serial.println(fadder2);
			#endif
            break;
          case 3:
            fadder3 = value;
			#ifdef DEBUG
                Serial.print("$F3:");
				Serial.println(fadder3);
			#endif
            break;
          case 4:
            fadder4 = value;
			#ifdef DEBUG 
                Serial.print("$F4:");
				Serial.println(fadder4);
			#endif
            break;
          case 11:
            xy1_x = value;
            xy1_y = value2;
			#ifdef DEBUG 
                Serial.print("$XY1:");
				Serial.print(xy1_x);
				Serial.print(",");
				Serial.println(xy1_y);
			#endif
            break;
          case 12:
            xy2_x = value;
            xy2_y = value2;
			#ifdef DEBUG
                Serial.print("$XY2:");
				Serial.print(xy2_x);
				Serial.print(",");
				Serial.println(xy2_y);
			#endif
            break;
          case 21:
            if (value==0)
              push1 = 0;
            else
              push1 = 1;
			#ifdef DEBUG
                Serial.print("$P1:");
				Serial.println((int)push1);
			#endif
            break;
          case 22:
            if (value==0)
              push2 = 0;
            else
              push2 = 1;
			#ifdef DEBUG 
                Serial.print("$P2:");
				Serial.println((int)push2);
			#endif
            break;
          case 23:
            if (value==0)
              push3 = 0;
            else
              push3 = 1;
			#ifdef DEBUG
                Serial.print("$P3:");
				Serial.println((int)push3);
			#endif
            break;
          case 24:
            if (value==0)
              push4 = 0;
            else
              push4 = 1;
			#ifdef DEBUG
                Serial.print("$P4:");
				Serial.println((int)push4);
			#endif
            break;
		  case 31:
            if (value==0)
              toggle1 = 0;
            else
              toggle1 = 1;
			#ifdef DEBUG
                Serial.print("$T1:");
				Serial.println((int)toggle1);
			#endif
            break;
          case 32:
            if (value==0)
              toggle2 = 0;
            else
              toggle2 = 1;
			#ifdef DEBUG
                Serial.print("$T2:");
				Serial.println((int)toggle2);
			#endif
            break;
          case 33:
            if (value==0)
              toggle3 = 0;
            else
              toggle3 = 1;
			#ifdef DEBUG
                Serial.print("$T3:");
				Serial.println((int)toggle3);
			#endif
            break;
          case 34:
            if (value==0)
              toggle4 = 0;
            else
              toggle4 = 1;
			#ifdef DEBUG
                Serial.print("$T4:");
				Serial.println((int)toggle4);
			#endif
            break;
        }
      }
    }
  }  // end Serial.available()
};
void MsgRead2();
void ParseMsg();
void WiFi_MsgSend(char *c,unsigned char msgSize, float p)
{
  
};	
*/
#endif
