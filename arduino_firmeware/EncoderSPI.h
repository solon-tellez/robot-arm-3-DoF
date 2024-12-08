

uint16_t ABSposition = 0;
uint16_t ABSposition_last = 0;
uint8_t temp[2];

float deg = 0.00;

int CS =9;

uint8_t SPI_T (uint8_t msg)    //Repetive SPI transmit sequence
{
   uint8_t msg_temp = 0;  //vairable to hold recieved data
   digitalWrite(CS,LOW);     //select spi device
   msg_temp = SPI.transfer(msg);    //send and recieve
   digitalWrite(CS,HIGH);    //deselect spi device
   return(msg_temp);      //return recieved byte
}


float Leer_ENC()
{
  uint8_t recieved = 0xA5;    //just a temp vairable
  ABSposition = 0;    //reset position vairable
  
  SPI.begin();    //start transmition
  digitalWrite(CS,LOW);
  
  SPI_T(0x10);   //issue read command
  delayMicroseconds(25);
  
  recieved = SPI_T(0x00);    //issue NOP to check if encoder is ready to send
  delayMicroseconds(25);
  
  while (recieved != 0x10)    //loop while encoder is not ready to send
  {
    recieved = SPI_T(0x00);    //cleck again if encoder is still working 
    delayMicroseconds(25);    //wait a bit
  }
  
  temp[0] = SPI_T(0x00);    //Recieve MSB
    delayMicroseconds(25);    //wait a bit
  temp[1] = SPI_T(0x00);    // recieve LSB
  
  digitalWrite(CS,HIGH);  //just to make sure   
  SPI.end();    //end transmition
  
  temp[0] &=~ 0xF0;    //mask out the first 4 bits
    
  ABSposition = temp[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
  ABSposition += temp[1];    // add LSB to ABSposition message to complete message
    

  //if (ABSposition != ABSposition_last)    //if nothing has changed dont wast time sending position
  {
    ABSposition_last = ABSposition;    //set last position to current position
    deg = ABSposition;
    deg = deg * 0.08789;    // aprox 360/4096
    //Serial.println(deg);     //send position in degrees
  }   

  #if (DEVICE == 3) 
    if((deg<360)&&(deg>270))
    {
      deg=495-deg;
    }
    else if((deg>=0)&&(deg<180))
    {
      deg=135-deg; //-360+135
    }
   
  #endif

  #if (DEVICE == 2) 
    if((deg>=0)&&(deg<180))
    {
      deg=deg+45;
    }
    else if((deg>180)&&(deg<=360))
    {
      deg=deg-315; //-360+135
    }
   
  #endif

  #if (DEVICE == 1) 
    if((deg>=0)&&(deg<180))
    {
      deg=deg+90;
    }
    else if((deg>180)&&(deg<=360))
    {
      deg=deg-270; //-360+135
    }
   
  #endif


  
  return deg;
 
}


void Config_ENCSPI(int SS)
{
  CS=SS;
  pinMode(CS,OUTPUT);//Slave Select

  //Configuracion SPI
  digitalWrite(CS,HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  //SPI.setClockDivider(SPI_CLOCK_DIV32);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.end();
}


void Set_Zero_Point()
{

  uint8_t recieved=0x10;


  SPI.begin();    //start transmition
  digitalWrite(CS,LOW);
  SPI_T(0x70);   //issue read command


  while (recieved != 0x80)    //loop while encoder is not ready to send
  {
    recieved = SPI_T(0x00);    //cleck again if encoder is still working 
    delayMicroseconds(25);    //wait a bit
  }
  digitalWrite(CS,HIGH);  //just to make sure   
  SPI.end();    //end transmition
  
}
