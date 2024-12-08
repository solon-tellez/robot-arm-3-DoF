#define IZQ 0
#define DER 1

// Constantes para el control PID 
#if (DEVICE == 1)
float kp = 1.59;
float ki = 3.5;
float kd = 0.018;
float N = 20;
float b = 0.7;
float c = 0.002;
float Ts = 0.015; //tiempo de muestreo en segundos

//Saturacion
volatile float satpos = 50;
volatile float satneg = -20;

float duty_max=197.0;

float Kp=40;
float U_Kp=0.5;

#elif (DEVICE == 3) 
//----------MOTOR 12V-----------------
//IZQ pwm=202 a 13.2v       
float kp = 3.4;
float ki = 15;
float kd = 0.11;
float N = 10;
float b = 0.1;
float c = 0.0;
float Ts = 0.015; //tiempo de muestreo en segundos

//Saturacion
volatile float satpos = 13.2;
volatile float satneg = -13.2;

float duty_max=235.0;

float Kp=1;
float U_Kp=0;


//dispo 3 duty max 235
//------------------------------------------

#elif (DEVICE == 2) 
//----------MOTOR 24V-----------------
// pwm max = 194 -->23.5 fuente en 25.5 **** 22 en fuente =24 
float kp = 1.59;
float ki = 3.5;
float kd = 0.018;
float N = 20;
float b = 0.7;
float c = 0.002;
float Ts = 0.015; //tiempo de muestreo en segundos

//Saturacion
volatile float satpos = 50;
volatile float satneg = -20;

float duty_max=197.0;
//------------------------------------------

float Kp=5;
float U_Kp=0.5;

#endif


//variables para PID
int duty=0; 

//Variables "globales"
float Dprevio = 0;
float Iprevio = 0;
float rprevio = 0;
float yprevio = 0;
int satur = 0;

//------------------

volatile int tm=0;
volatile int sub_timer;

float error=0;


float Vel=0;
int Vel_timer=0;
int Vel_flag=0;

int Vel_Pulse=2;

float Pos_0=0;
float Pos_1=0;





float *ptr_ang_real_control;
float *ptr_ang_des_control;

void ControlSR();
void Control_PID();
void Control_ON_OFF();
void ta_Enc();
float Regulador_PID(float rk, float yk) ;
int PWM_Calc( float PwmMax, float V);

void Lazo_Control();
void Control_Kp();


float Velocidad(float y1);


/*
volatile int cont_muestras=0;
volatile float muestas[20];
*/



void  Configurar_t_muestreo(int t)
{
  tm=t/5;

   cli();   //Deshabilitar interrupciones 
  // Configurar TBC1, ISR cada 5ms
  TCB1.CTRLA   = 0;
  TCB1.CTRLA  |= TCB_CLKSEL_CLKDIV2_gc;     // Divider 2 from CLK_PER ( = 1/16MHz )
  TCB1.CTRLA  |= TCB_ENABLE_bm;             // Enable Timer
  TCB1.CTRLB   = 0;                         // Periodic Interrupt mode, waveform output disabled
  TCB1.INTCTRL = 0x00000001;                // Enable interrupt on capture
  TCB1.CCMP    = 40000;                      // @16Mhz it is 5ms ( = 40000 /(16MHz/2) )
  TCB1.CNT     = 0;                         // Make sure to start time at zero
  sei();  //Habilitar interrupciones 
}

ISR(TCB1_INT_vect) // called every 5000us
  {                      

    TCB1.INTFLAGS |= 0x00000001;               // Clear interupt flag
    sub_timer++;
    Vel_timer++;

    if(sub_timer==tm)
    {
      if(cntrl==1)
      {
        sub_timer=0;        

        #if ((DEVICE == 1)||(DEVICE == 2))
          Lazo_Control(); //PID
        #endif

        #if (DEVICE == 3)
          Control_Kp(); 
        #endif

      }

      if((Time_Open_Loop==100)&&(Time_O_L_Flag==1))
      {
        Time_O_L_Flag=0;
        SalidaPWM(IZQ, 0);
        SalidaPWM(DER, 0);
        Serial.print("PARO");
      }

      if(cntrl==0)
      {
        sub_timer=0;
        *ptr_ang_real_control=Leer_ENC();

        if(Time_O_L_Flag==1)
        {
          Time_Open_Loop++;
        }
      }      



    }

    if(Vel_timer==60)
    {
      Vel_timer=0;
      Vel_flag=1;
    }

  }


void ControlSR()
{
  static boolean on = false;
  if (on) {
    digitalWrite(2, LOW);
    on = false;    
  }
  else {
    digitalWrite(2, HIGH);
    on = true;
  }
 
}


void Control_Kp()
{
  *ptr_ang_real_control=Leer_ENC();

  error=*ptr_ang_des_control-*ptr_ang_real_control;

  if(Vel_flag==1)
  {
    Vel=Velocidad(*ptr_ang_real_control);
    Vel_flag=0;
  }

  if(error<-0.5)
  {
    CambioCanal(IZQ);
    SalidaPWM(IZQ, 100);
  }
  else if(error>0.5)
  {
    
    if(Vel>0)
    {
      U_Kp=(1.55*error)+80;
    }
    else
    {
      U_Kp=150;
    }
    CambioCanal(DER);
    SalidaPWM(DER, U_Kp);    
  }
  else
  {
    SalidaPWM(IZQ, 0);
    SalidaPWM(DER, 0);
  }


}

void Control_ON_OFF()
{ 
  *ptr_ang_real_control=Leer_ENC();
  error=*ptr_ang_des_control-*ptr_ang_real_control;
  Vel=Velocidad(*ptr_ang_real_control);

  //DEVICE 1=140 //DEVICE 2=10 25 //DEVICE 3=70 80

   if(error<-0.5)
  {    
    CambioCanal(IZQ);
    SalidaPWM(IZQ, 10);
  }
  else if(error>0.5)
  {
    CambioCanal(DER);
    SalidaPWM(DER, 25);
  }
  else
  {
    SalidaPWM(IZQ, 0);
    SalidaPWM(DER, 0);
  }

}


/*
void ta_Enc()
{

  if(cont_muestras<20)
  { 

    muestas[cont_muestras]=Leer_ENC();
    //Serial.print( cont_muestras); 
    //Serial.print(" >> ");  
    //Serial.println(muestas[cont_muestras]);
    cont_muestras++;
  }

  
  

}
*/

void Lazo_Control()
{
  float rk= *ptr_ang_des_control; //Obtencion de la Referencia
  *ptr_ang_real_control=Leer_ENC();
  float yk=*ptr_ang_real_control;
  float u = Regulador_PID(rk, yk); 


  #if (DEVICE == 1)    
    error=*ptr_ang_des_control-*ptr_ang_real_control;
    
    if(Vel_flag==1)
    {
      Vel=Velocidad(*ptr_ang_real_control);
      Vel_flag=0;
    }
        
    if((error<0.2)&&(error>-0.2))
    {
      SalidaPWM(IZQ, 0);
      SalidaPWM(DER, 0);
    }

    else if ((Vel==0)&&(error>0.2)&&(Vel_timer<15))
    {
      CambioCanal(DER);
      SalidaPWM(DER, 120);
      if(Vel_timer==14)
      {
        Vel=0.1;
      }
    }

    else if((Vel==0)&&(error<-0.2)&&(Vel_timer<15))
    {
      CambioCanal(IZQ);
      SalidaPWM(IZQ, 120);
      if(Vel_timer==14)
      {
        Vel=0.1;
      }
    }
    
    else if (u >= 0) 
    {
      duty= PWM_Calc(duty_max,u);
      CambioCanal(DER);
      SalidaPWM(DER, duty);  
    } 
    else if (u < 0) 
    {
      u = -u;
      duty= PWM_Calc(duty_max,u);
      CambioCanal(IZQ);
      SalidaPWM(IZQ, duty);
    }

  #endif

  #if (DEVICE == 2)   

    if (u >= 0) 
    {
      duty= PWM_Calc(duty_max,u);
      CambioCanal(DER);
      SalidaPWM(DER, duty);  
    } 
    else if (u < 0) 
    {
      u = -u;
      duty= PWM_Calc(duty_max,u);
      CambioCanal(IZQ);
      SalidaPWM(IZQ, duty);
    } 

  #endif

}






float Regulador_PID(float rk, float yk) 
{
  float Q1 = 1 - N * Ts;
  float Q2 = kd * N;
  float incrI = ki * Ts * (rprevio - yprevio);
  float I;

  if (satur * incrI > 0) {
    I = Iprevio;
  } else {
    I = Iprevio + ki * Ts * (rprevio - yprevio);
  }

  float P = kp * (b * rk - yk);
  float D = Q1 * Dprevio + Q2 * c * (rk - rprevio) - Q2 * (yk - yprevio);
  float u = P + I + D; 

  if (u > satpos) {
    u = satpos;
    satur = 1;
  } else if (u < satneg) {
    u = satneg;
    satur = -1;
  } else {
    Iprevio = I;
    satur = 0;
  }

  Dprevio = D;
  rprevio = rk;
  yprevio = yk;
  //Serial.println(u); 
  return u;
}


int PWM_Calc( float PwmMax, float V)
{ 
  int y=0;
  float ytemp=0;
  
  ytemp=((PwmMax)/(satpos))*(V);
  y=ytemp;

  return y;

}


float Velocidad(float y1)
{
  Pos_1=Pos_0;
  Pos_0=y1;

  return (Pos_0-Pos_1)/2;

}

























