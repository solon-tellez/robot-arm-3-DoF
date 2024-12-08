
float requestToSlave(int dir)
{
  float nums=0;

  int cont=0;
  int ent=0;
  int dec=0;

  Wire.requestFrom(dir, 2);

  while(Wire.available())
  {
    if(cont==0)
    {
      ent = Wire.read();/* read data received from slave */ 
      cont=1;
    }  
    else
    {
      dec = Wire.read();/* read data received from slave */ 
    }    

  }

  nums=float(dec);

  while(nums>1)
  {
    nums=nums/10.0;
  }

  nums=nums+float(ent);
  
  return nums;
 
}