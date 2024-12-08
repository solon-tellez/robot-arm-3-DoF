#include <Wire.h>
#include <ESP32Servo.h> 

#include <math.h>

#include "I2C_Master.h"




Servo myservo;  // create servo object to control a servo

int servoPin = 5;      // GPIO pin used to connect the servo control (digital out)


#define LED 2
#define BUTTON 33



String cmd;
char acction;


bool Gripper_state=3;
float theta1=0;
float beta1=0;
float gamma1=0;

float gamma_des=0;

float ent_temp=0;
int   dec_ent_temp=0;
float dec_temp=0;

float ang_recv[4]={0,0,0,0};
int v_cmd[4]={0,0,0,0};



int z_dir=0;

int Led=0;
bool Led_Estado=1;

int buttonState = 1;
unsigned long t1 = 0;
unsigned long t2 = 0; 

bool timer_flag=0;
 
volatile int interruptCounter=0;
int totalInterruptCounter;
 
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() 
{
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux); 

  timerStop(timer);
}


void setup() 
{
    timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
  timerStart(timer);
  /*---------------------Configuracion entradas y salidas---------------------*/
  // Allow allocation of all timers
  
	//ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50);// Standard 50hz servo
  myservo.attach(servoPin, 500, 2400);   // attaches the servo on pin 18 to the servo object
                                         // using SG90 servo min/max of 500us and 2400us
                                         // for MG995 large servo, use 1000us and 2000us,
                                         // which are the defaults, so this line could be
                                         // "myservo.attach(servoPin);"


  // Unimos este dispositivo al bus I2C
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  Wire.begin();
  digitalWrite(LED,HIGH);

  Serial.begin(9600);
  Serial.println("START");
  Serial.flush();
  delay(2000);

  myservo.write(130);    


  
  

}
 
byte pin[] = {2, 3, 4, 5, 6};
byte estado = 0;
 
void loop() 
{
/*
  if (interruptCounter > 0) 
  {
 
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
 
    totalInterruptCounter++;
 
    Serial.print("An interrupt as occurred. Total number: ");
    Serial.println(totalInterruptCounter);
 
  }
*/
  buttonState = digitalRead(BUTTON);

   
  if ((buttonState == LOW)&&((interruptCounter > 0))) 
  {

    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    portEXIT_CRITICAL(&timerMux);
    
    timerStart(timer);
    timerRestart(timer);

    //Led_Estado=!Led_Estado;
    //digitalWrite(LED,Led_Estado); 

    Serial.println("GAME_");
    
  } 
  

  
  

  if (Serial.available()) 
  {
    cmd=Serial.readStringUntil('\n');
   // Serial.println(cmd);

    char buffer_c[cmd.length() + 1];

    cmd.toCharArray(buffer_c, cmd.length() + 1);

    //acction=cmd.toCharArray( unsigned int bufsize);

   // for (int i = 0; i < cmd.length(); i++)
    //   Serial.println(buffer_c[i]);

    //String stringOne = String(buffer_c);

   // Serial.println(stringOne); 
    

    /*
    T=Tomar ficha, argumento de entrada 3 angulos de 3 digitos
    Espera a llegar a su posicion final y cierra el gripper 
    T_XXX;XXX;XXX

    M=Tomar ficha, argumento de entrada 3 angulos de 3 digitos
    M_XXX;XXX;XXX

    T=Tomar ficha, argumento de entrada 3 angulos de 3 digitos
    Espera a llegar a su posicion final y abre el gripper 
    P_XXX;XXX;XXX

    A=Retorna la posicion angular de cada eje en formato  A_XXX;XXX;XXX
    A_

    Cambia el estado del gripper
    G_

    L=activa los leds correpondientes donde x es un numero de 0 al 4, y indica el estado
    L_x_y
    */

    /*V_D_E_Vl
    D=Dispositivo 1, 2, 3
    E=Estado  1=derecha 2=izquierda 3=control off
    Vl=Voltaje  00 - 24
    */


    if(buffer_c[0]=='Z')
    {

      z_dir=cmd.substring(2, 3).toInt();
      
      if((z_dir>=1)||(z_dir<=3))
      {
        Wire.beginTransmission(1+(z_dir*2));
        Wire.write(5);
        Wire.write(0);
        Wire.write(0);
        Wire.endTransmission();
      }

      Serial.print("Set zero to disp: ");
      Serial.println( z_dir); 
    }



    if(buffer_c[0]=='V')
    {
      v_cmd[1]=cmd.substring(2, 3).toInt();
      v_cmd[2]=cmd.substring(4, 5).toInt();
      v_cmd[3]=cmd.substring(6, 8).toInt();

      Serial.print("Disp: ");
      Serial.print( v_cmd[1]); 
      Serial.print("  -   Est: ");
      Serial.print( v_cmd[2]); 
      Serial.print("  -   Volt: ");
      Serial.println( v_cmd[3]); 

      Wire.beginTransmission(1+(v_cmd[1]*2));
      Wire.write(4);
      Wire.write(v_cmd[2]);
      Wire.write(v_cmd[3]);
      Wire.endTransmission();


      //Wire.beginTransmission(1+(Led*2));
       // Wire.write(3); //1:escribe angulo 2:lee angulo 3:estado led 
       // Wire.write(Led_Estado);
       // Wire.write(0);
       // Wire.endTransmission();

    }



    

    if(buffer_c[0]=='G')
    {

      Gripper_state=cmd.substring(2, 3).toInt();

      //Gripper_state=!Gripper_state;

      if (Gripper_state==0) 
      {
       myservo.write(130);    
      // Serial.println(115);
      }

      if (Gripper_state==1) 
      {
       myservo.write(169); 
     //  Serial.println(169);   
      }

      Gripper_state=3;

    }

    if(buffer_c[0]=='M')
    {

      //angulo con XXX.X

       ang_recv[1]=cmd.substring(2, 7).toFloat();
       ang_recv[2]=cmd.substring(8, 13).toFloat();
       ang_recv[3]=cmd.substring(14, 19).toFloat();
      // Serial.println(ang_recv[1]);  
      // Serial.println(ang_recv[2]);  
       //Serial.println(ang_recv[3]);  
       


       //angulo con XXX
        /*
       ang_recv[1]=cmd.substring(2, 5).toFloat();
       ang_recv[2]=cmd.substring(6, 9).toFloat();
       ang_recv[3]=cmd.substring(10, 13).toFloat();
        */

      //----------------Leer Beta1----------------
      

      Wire.beginTransmission(5);
      Wire.write(2); //1:escribe angulo 2:lee angulo 3:estado led 
      Wire.write(0);
      Wire.write(0);
      Wire.endTransmission();

      delay(20);
      

      beta1=requestToSlave(5);

      //Serial.println(beta1); 

      //------------------Alpha------------------

      dec_temp = modf(ang_recv[1], &ent_temp); 
      dec_temp =(dec_temp*10.0)+0.9;

      dec_ent_temp=dec_temp/1;

      Wire.beginTransmission(3);
      Wire.write(1); //1:escribe angulo 2:lee angulo 3:estado led 
      Wire.write(int(ent_temp));
      Wire.write(dec_ent_temp);
      Wire.endTransmission();

      //------------------Beta------------------

      dec_temp = modf(ang_recv[2], &ent_temp); 
      dec_temp =(dec_temp*10.0)+0.9;

      dec_ent_temp=dec_temp/1;

      Wire.beginTransmission(5);
      Wire.write(1); //1:escribe angulo 2:lee angulo 3:estado led 
      Wire.write(int(ent_temp));
      Wire.write(dec_ent_temp);
      Wire.endTransmission();

      //------------------Gamma------------------
      //gamma_des=ang_recv[3]+180-beta1;
      gamma_des=ang_recv[3]+180-ang_recv[2];

      dec_temp = modf(gamma_des, &ent_temp); 
      dec_temp =(dec_temp*10.0)+0.9;

      dec_ent_temp=dec_temp/1;

      

      Wire.beginTransmission(7);
      Wire.write(1); //1:escribe angulo 2:lee angulo 3:estado led 
      Wire.write(int(ent_temp));
      Wire.write(dec_ent_temp);
      Wire.endTransmission();

      //Serial.println(ang_recv[3]);    
      //----------------------------------------  
 

    }

    if(buffer_c[0]=='A')
    {

      //------------------Alpha------------------
      
      Wire.beginTransmission(3);
      Wire.write(2); //1:escribe angulo 2:lee angulo 3:estado led 
      Wire.write(0);
      Wire.write(0);
      Wire.endTransmission();

      delay(20);
      

      theta1=requestToSlave(3);

     // Serial.println(theta1);     

      delay(20);
      
      //------------------Beta------------------

      Wire.beginTransmission(5);
      Wire.write(2); //1:escribe angulo 2:lee angulo 3:estado led 
      Wire.write(0);
      Wire.write(0);
      Wire.endTransmission();

      delay(20);
      

      beta1=requestToSlave(5);

      //Serial.println(beta1); 

      

      //------------------Gamma------------------
      


      Wire.beginTransmission(7);
      Wire.write(2); //1:escribe angulo 2:lee angulo 3:estado led 
      Wire.write(0);
      Wire.write(0);
      Wire.endTransmission();

      delay(20);
      

      gamma1=requestToSlave(7);
      gamma1=gamma1-180+beta1;


      //Serial.println(gamma1); 
      
      
      //---------------------------------------- 

      
      String msg;
      msg.clear();
      msg.concat(theta1);
      msg.concat(";");
      msg.concat(beta1);
      msg.concat(";");
      msg.concat(gamma1);
      Serial.println(msg); 


    }

    if(buffer_c[0]=='L')
    {
      Led=cmd.substring(2, 3).toInt();
      //Serial.println(Led);          
      Led_Estado=cmd.substring(4, 5).toInt();
      //Serial.println(Led_Estado);    

      if(Led==0)
      {
        digitalWrite(LED,Led_Estado); 
      }

      else if(Led>0 && Led<4)
      {
        Wire.beginTransmission(1+(Led*2));
        Wire.write(3); //1:escribe angulo 2:lee angulo 3:estado led 
        Wire.write(Led_Estado);
        Wire.write(0);
        Wire.endTransmission();
      }
    }

    




  }


 

}