
byte cdm_I2C = 0;
byte dato_I2C = 0;
byte dato_2_I2C = 0;


double ent_temp=0;
int   dec_ent_temp=0;
double dec_temp=0;

float *ptr_ang_real;

void receiveEvent(int howMany) 
{
 
 
  if (Wire.available() == 3)
  {
    
    cdm_I2C = Wire.read();
  }
  
  if (Wire.available() == 2)
  {
    dato_I2C  = Wire.read();
  }

  if (Wire.available() == 1)
  {
    dato_2_I2C  = Wire.read();
  }

}


void requestEvent()
{
  dec_temp = modf(*ptr_ang_real, &ent_temp); 
  dec_temp =(dec_temp*10.0)+0.9;

  dec_ent_temp=dec_temp/1;
  
  Wire.write(int(ent_temp));
  Wire.write(dec_ent_temp);
}


void Config_COMI2C(int adrs)
{
  Wire.begin(adrs);                   // Unimos este dispositivo al bus I2C con dirección 15
  Wire.onReceive(receiveEvent);     // Registramos el evento al recibir datos
  Wire.onRequest(requestEvent);     //Evento de peticion de datos
}



