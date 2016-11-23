#ifndef EX_OSC_H
#define EX_OSC_H

//#define DEBUG0
//#define DEBUG1
//#define DEBUG2
//#define DEBUG3


char UDPBuffer[8];
char readStatus = -1;
unsigned char readCounter;
unsigned char readNumParams;
unsigned char commandType;

uint8_t page;
uint8_t newMessage;
float fadder1;
float fadder2;
float fadder3;
float fadder4;
float xy1_x;
float xy1_y;
float xy2_x;
float xy2_y;
uint8_t push1;
uint8_t push2;
uint8_t push3;
uint8_t push4;
uint8_t toggle1;
uint8_t toggle2;
uint8_t toggle3;
uint8_t toggle4;

float extractParamFloat1()
{
  union
  {
    unsigned char Buff[4];
    float d;
  } u;
 
  u.Buff[0] = (unsigned char)UDPBuffer[0];
  u.Buff[1] = (unsigned char)UDPBuffer[1];
  u.Buff[2] = (unsigned char)UDPBuffer[2];
  u.Buff[3] = (unsigned char)UDPBuffer[3];
  return(u.d); 
}
float extractParamFloat2()
{
  union
  {
    unsigned char Buff[4];
    float d;
  } u;
 
  u.Buff[0] = (unsigned char)UDPBuffer[4];
  u.Buff[1] = (unsigned char)UDPBuffer[5];
  u.Buff[2] = (unsigned char)UDPBuffer[6];
  u.Buff[3] = (unsigned char)UDPBuffer[7];
  return(u.d); 
}

void eX_WiFi_MSG_Read()
{
  uint8_t i;
  float value;
  float value2;
  
  if (readStatus == -1)
  {
  //Поскольку мы случайным образом опрашиваем UDP порт, то мы должны (при наличие данных в порту) найти первый символ OSC протокола '/'
    while (Serial1.available())
    {
       UDPBuffer[0] = Serial1.read();
       if (UDPBuffer[0]=='/')
       {
         readStatus = 0;       // статус 0 говорит, что первое вхождение '/' найдено и помещено в буфер приема
         return;
       }
     } 
  }
  else
  {
    // New byteas available to process?
    if (Serial1.available() > 0)
    {
      for (i=7;i>0;i--)
      {
        UDPBuffer[i] = UDPBuffer[i-1];
      }
      UDPBuffer[0] = Serial1.read();
        #ifdef DEBUG0  
           Serial.print(UDPBuffer[0]);
        #endif
      // We look for an OSC message start like /x/
      if ((UDPBuffer[0] == '/')&&(UDPBuffer[2] == '/'))
      {
        if (readStatus == 0)
        {
	        page = UDPBuffer[1] - '0';  // Convert page to int
          readStatus = 1;
            #ifdef DEBUG1
              Serial.print("$");
              Serial.print(page);
            #endif
        }
        else
        {
          readStatus = -1;
            #ifdef DEBUG1
              Serial.println("!ERR:osc");
            #endif
        }
        return;
      }
      else if (readStatus==1)
      {
        // looking for the message type
        if ((UDPBuffer[3] == 'd')&&(UDPBuffer[2] == 'e')&&(UDPBuffer[1] == 'r'))
        {
          readStatus    = 2;    // Message type detected
          readCounter   = 11;   // Bytes to read the parameter
          readNumParams = 1;    // 1 parameters
          switch (UDPBuffer[0]) // fadder number
          {  
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
          }  // end switch (UDPBuffer[0])
          return;
        }    // end fadder

        // XY message
        else if ((UDPBuffer[2] == 'x')&&(UDPBuffer[1] == 'y'))
        {
          readStatus    = 2;    // Message type detected
          readCounter   = 14;   // Bytes to read the parameters
          readNumParams = 2;    // 2 parameters
          switch (UDPBuffer[0])  // xy number
          {
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
        else if ((UDPBuffer[3] == 'u')&&(UDPBuffer[2] == 's')&&(UDPBuffer[1] == 'h'))
        {
          readStatus    = 2;    // Message type detected
          readCounter   = 10;   // Bytes to read the parameter
          readNumParams = 1;    // 1 parameters
          switch (UDPBuffer[0])
          {  // push number
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
        else if ((UDPBuffer[3] == 'g')&&(UDPBuffer[2] == 'l')&&(UDPBuffer[1] == 'e'))
        {
          readStatus    = 2;    // Message type detected
          readCounter   = 10;   // Bytes to read the parameter
          readNumParams = 1;    // 1 parameters
          switch (UDPBuffer[0])
          {  // push number
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
      } 
      else if (readStatus==2)
      {
        readCounter--;   // Reading counter until we reach the Parameter position
        if (readCounter<=0)
        {
          readStatus=-1;
          value = extractParamFloat1();
	        if ((value<0.0)||(value>1.0))
          {
              #ifdef DEBUG3
                Serial.println("!ERR:f1!");
              #endif
            return;
          }
          if (readNumParams==2)
          {
            value2 = extractParamFloat2();
            if ((value2<0.0)||(value2>1.0))
            {
                #ifdef DEBUG3
                  Serial.println("!ERR:OSCf2!");
                #endif
              return;
            }
          }
	        newMessage = 1;
          switch (commandType)
          {
            case 1:
              fadder1 = value;
	              #ifdef DEBUG3 
                  Serial.print("$F1:");
                  Serial.println(fadder1);
                #endif
              break;
            case 2:
              fadder2 = value;
                #ifdef DEBUG3
                  Serial.print("$F2:");
                  Serial.println(fadder2);
                #endif
              break;
            case 3:
              fadder3 = value;
                #ifdef DEBUG3
                  Serial.print("$F3:");
                  Serial.println(fadder3);
                #endif
              break;
            case 4:
              fadder4 = value;
                #ifdef DEBUG3
                  Serial.print("$F4:");
                  Serial.println(fadder4);
                #endif
              break;
            case 11:
              xy1_x = value;
              xy1_y = value2;
                #ifdef DEBUG3
                  Serial.print("$XY1:");
                  Serial.print(xy1_x);
                  Serial.print(",");
                  Serial.println(xy1_y);
                #endif
              break;
            case 12:
              xy2_x = value;
              xy2_y = value2;
                #ifdef DEBUG3
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
                #ifdef DEBUG3
                  Serial.print("$P1:");
                  Serial.println((int)push1);
                #endif
              break;
            case 22:
              if (value==0)
                push2 = 0;
              else
                push2 = 1;
                #ifdef DEBUG3
                  Serial.print("$P2:");
                  Serial.println((int)push2);
                #endif
              break;
            case 23:
              if (value==0)
                push3 = 0;
              else
                push3 = 1;
                #ifdef DEBUG3
                  Serial.print("$P3:");
                  Serial.println((int)push3);
                #endif
              break;
            case 24:
              if (value==0)
                push4 = 0;
              else
                push4 = 1;
                #ifdef DEBUG3
                  Serial.print("$P4:");
                  Serial.println((int)push4);
                #endif
              break;
            case 31:
              if (value==0)
                toggle1 = 0;
              else
                toggle1 = 1;
                #ifdef DEBUG3
                  Serial.print("$T1:");
                  Serial.println((int)toggle1);
                #endif
              break;
            case 32:
              if (value==0)
                toggle2 = 0;
              else
                toggle2 = 1;
                #ifdef DEBUG3
                  Serial.print("$T2:");
                  Serial.println((int)toggle2);
                #endif
              break;
            case 33:
              if (value==0)
                toggle3 = 0;
              else
                toggle3 = 1;
                #ifdef DEBUG3
                  Serial.print("$T3:");
                  Serial.println((int)toggle3);
                #endif
              break;
            case 34:
              if (value==0)
                toggle4 = 0;
              else
                toggle4 = 1;
	              #ifdef DEBUG3
	                Serial.print("$T4:");
	                Serial.println((int)toggle4);
	              #endif
	            break;
	        } // end case
        }   // end read parameters
      }     // end read command type
    }       // end Serial.available()
  }  
} 



#endif //EX_OSC_H
