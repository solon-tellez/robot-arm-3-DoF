#define LED 2


#define DEVICE 1

#if (DEVICE == 1) 
#define DEVICEADDR 3

#elif (DEVICE == 2 )
#define DEVICEADDR 5

#else
#define DEVICEADDR 7

#endif

volatile int cntrl=1;
volatile int Time_Open_Loop=0;
volatile int Time_O_L_Flag=0;



#include <Wire.h>
#include <SPI.h>
#include "ComunicacionI2C.h"
#include "EncoderSPI.h"
#include "PWM_HighFrequency.h"
#include "Control.h"

#if (DEVICE == 1)
float Ang_des=90;
#endif

#if (DEVICE == 2)
float Ang_des=45;
#endif

#if (DEVICE == 3)
float Ang_des=135;
#endif


float Ang_real=0;

float Ang_tmp=0;




void setup() 
{

  /*---------------------Configuracion entradas y salidas---------------------*/
  pinMode(LED, OUTPUT); //GPP (LED)

  /*---------------------Configuracion incial PWM---------------------*/
  Config_PWM_HighFrequency(); //PWM PIN 9 y 10 a 31.25 kHz

  /*---------------------Configurar comunicaciones---------------------*/
  //Configuracion SPI
  Config_ENCSPI(6);       //PIN Slave Select 
  // Configuracion I2C
  Config_COMI2C(DEVICEADDR);      //Direccion en el bus I2C
  ptr_ang_real=&Ang_real;

  // Configuracion Serie
  Serial.begin(9600);         //
  Serial.println("starting"); //handshake para la UI
  Serial.flush();
  delay(2000);

  /*---------------------Configurar tm del PID---------------------*/
  Configurar_t_muestreo(15); //t en milisengundo 
  ptr_ang_real_control=&Ang_real;
  ptr_ang_des_control=&Ang_des;

  digitalWrite(LED,HIGH);

  //delay(32000);  // 1 segundo
  //CambioCanal(IZQ);
  //SalidaPWM(IZQ, duty_max*0.5); 

   
}

void loop() 
{

  /*
  if(cont_muestras==20)
  { 
    int index=0;

    while(index<20)
    {
      Serial.print(index); 
      Serial.print(" : ");    
      Serial.println(muestas[index]);
      index++;
    }
    cont_muestras++;
    
  }
  */

 // Ang_des=30;
 // delay(20000);
 // Ang_des=150; 
  //delay(20000); 
  

  if (cdm_I2C == 5)
  {
    
    Set_Zero_Point();
    cdm_I2C = 0;
    dato_I2C= 0;
    dato_2_I2C= 0;
    Serial.println("Set Zero");
  }



  if (cdm_I2C == 4)
  {
    

    if(dato_I2C==3)
    {
      cntrl=!cntrl;
      Serial.print("V MODE - CONTROL: "); 
      Serial.println(cntrl); 
    }
    
    if(dato_I2C==1)
    {
      duty= PWM_Calc( duty_max,dato_2_I2C);
      CambioCanal(DER);
      SalidaPWM(DER, duty);

      Serial.print("Giro Der"); 
      Serial.print("   -   V: ");   
      Serial.print(dato_2_I2C);    
      Serial.print("   -   d: ");   
      Serial.println(duty);   
      Time_O_L_Flag=1;
      Time_Open_Loop=0;

    }
    if(dato_I2C==2)
    {
      duty= PWM_Calc(duty_max,dato_2_I2C);
      CambioCanal(IZQ);
      SalidaPWM(IZQ, duty);

      Serial.print("Giro Izq"); 
      Serial.print("   -   V: ");   
      Serial.print(dato_2_I2C);    
      Serial.print("   -   d: ");   
      Serial.println(duty);  
      Time_O_L_Flag=1;    
      Time_Open_Loop=0; 
    }

    

    cdm_I2C = 0;
    dato_I2C= 0;
    dato_2_I2C= 0;
  }
   

  if (cdm_I2C == 3)
  {
    digitalWrite(LED,dato_I2C);

    cdm_I2C = 0;
    dato_I2C= 0;
    dato_2_I2C= 0;
  }

  if (cdm_I2C == 2)
  {
    

    
    
    //Ang_request=Ang_real;
    Serial.println(Ang_real);    

    cdm_I2C = 0;
    dato_I2C= 0;
    dato_2_I2C= 0;
  }

  if (cdm_I2C == 1)
  {
   
    Ang_tmp=float(dato_2_I2C);
   
    while(Ang_tmp>=1)
    {
      Ang_tmp=Ang_tmp/10.0;

    }

    Ang_tmp=Ang_tmp+float(dato_I2C);

    Ang_des=Ang_tmp;

    cdm_I2C = 0;
    dato_I2C= 0;
    dato_2_I2C= 0;

    Serial.println(Ang_des); 

  }

  
}
