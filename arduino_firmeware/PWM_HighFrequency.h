  
  void Config_PWM_HighFrequency()
  {

    pinMode(10,OUTPUT); //M1_A
    pinMode(9,OUTPUT);  //M1_B


    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;    // Turn off timer while we change parameters.
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_CLKSEL_gm;    // Clear all CLKSEL bits.
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV2_gc; // Set prescaler to 2. // Can be changed from clk_per,clk_per/2..... clk_per/ 1024 (Datasheet PP 198)

    TCA0.SINGLE.CNT = 0x00;

    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;   //Select single slope PWM mode & enable compare channel
  
    TCA0.SINGLE.EVCTRL &= 0b11111110; //TCA0 counting clock ticks

    TCA0.SINGLE.PER = 255;
    TCA0.SINGLE.CMP0 = 0; /* PWM Period*/
    TCA0.SINGLE.CMP1 = 0; /* PWM Period*/
  
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;   // Re-enable timer. Pins 5 and 9 and 10 now run at 31.25 kHz.

  }
  



  void CambioCanal(int chn)
  {
    if(chn==0)
    {
      TCA0.SINGLE.CMP1 = 0;
      TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    }

    if(chn==1)
    {
      TCA0.SINGLE.CMP0 = 0;
      TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    }


  }


  void SalidaPWM(int chn, int duty)
  {

    if(chn==0)
    {
      TCA0.SINGLE.CMP0=duty;
    }

    if(chn==1)
    {
      TCA0.SINGLE.CMP1=duty;
    }

  }



