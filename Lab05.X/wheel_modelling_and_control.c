/*
  This program is the basis for all of the wheel labs
*/

//#include <p30f4011.h>

#include <libpic30.h>
#include <delay.h>
#include <adc10.h>
#include <pwm.h>
#include <uart.h>
#include <string.h>
#include <stdio.h>
#include <qei.h>
#include <timer.h>
#include <ports.h>

// Configuration Bits
#pragma config FPR = FRC_PLL16   // 117.92 MHz
#pragma config FOS = PRI
#pragma config FCKSMEN = CSW_FSCM_OFF
#pragma config WDT = WDT_OFF
#pragma config FPWRT = PWRT_16
#pragma config BODENV = BORV27
#pragma config BOREN = PBOR_OFF
#pragma config MCLRE = MCLR_EN
#pragma config GWRP = GWRP_OFF


#define max(A ,B) ((A) > (B) ? (A) : (B))
#define min(A, B) ((A) < (B) ? (A) : (B))

#define A2D_LOW 6 // lowest reading fron the pot, subtract this to make it zero
#define A2D_HIGH 998 // Maximum reading from the pot once the zero is set
#define MAX_DELTA_SPEED 5.0
#define MAX_DELTA_U 1000
#define MAX_ISUM 900
#define PERIOD 14739 // for 1000 Hz pwm frequency
#define MAX_DUTY 2*PERIOD
#define MAX_COUNT 1
#define PI 3.14159265
#define MAXCNT 719  // maximum count for QEI encoders before interrupt, 720 counts per revolution
#define U_FILT_LEN 5
#define FILT_LEN 4

// define some variables

int AD_value;
unsigned int GO;
int encindex, p1, p2, r1, r2;

/***************************************************************/

// external input interrupt handler

void __attribute__((interrupt,no_auto_psv)) _INT1Interrupt( void )
{
  unsigned int dutycyclereg, dutycycle;
  char updatedisable;

  // turn off the pwm signal

  dutycycle = (unsigned int)0;
  dutycyclereg = 3;
  updatedisable = 0;
  SetDCMCPWM( dutycyclereg, dutycycle, updatedisable );  // duty cycle set to low
  
  // turn off the LED to indicate power if off

  LATFbits.LATF6 = 0;  // signal the power is off
  
  // Disable the interrupt

  DisableINT1;

  // now just wait

  while(1);
}

/************************************************************/

// Initialize external interrupt 1

void Init_INT1(void)
{
  unsigned int config;

  config = FALLING_EDGE_INT & // interrupt on a falling edge
           EXT_INT_ENABLE &        // enable the interrupts
           //EXT_INT_PRI_0 ;
           GLOBAL_INT_ENABLE;
            
  ConfigINT1( config );


   // turn on the LED to show interrupt is set

  TRISFbits.TRISF6 = 0; 
  LATFbits.LATF6 = 1;  // signal the interrupt is set

  // prepare for an input on RD0

  TRISDbits.TRISD0 = 1;

  // enable the interrupt

  DisableINT1;
  EnableINT1;

  return;
}

/**********************************************************/

// timer 1 interrupt handler

void __attribute__((interrupt,auto_psv)) _T1Interrupt( void )
{ 
  unsigned int ReadQEI( void );
  extern int AD_value;
  extern unsigned int GO;
  extern int p2, r2, encindex;
  unsigned int AD_value_u;

 // read from the A/D channel

  ADCON1bits.SAMP = 1; // start the sampling
  while(!ADCON1bits.DONE);
  AD_value_u = ReadADC10(0);
  
  // convert and check range
  
  AD_value = (int) AD_value_u; 
  AD_value -= A2D_LOW;               // subtract to start at zero
  AD_value = max(AD_value,0);        // be sure 0 is the smallest
  AD_value = min(AD_value, A2D_HIGH);  // limit the maximum

  // update the position variables

  p2 = (int) ReadQEI();
  r2 = encindex;

  // reset Timer 1 interrupt flag 

  IFS0bits.T1IF = 0;

  // if GO is 1 we are not done before the next interrupt!
  
  if(GO == 1)
   LATEbits.LATE1 = 1;
  
  GO = 1;
}

/***********************************************************/ 

// Initialize timer 1

void Init_Timer1( unsigned int period )
{
  unsigned int config;

  config = T1_INT_PRIOR_4 & // set interrupt priority to 2
           T1_INT_ON;       // enable the interrupts
            
  ConfigIntTimer1( config );

  config =  T1_ON &  // turn on the timer
            T1_IDLE_CON & // operate during sleep
            T1_GATE_OFF & // timer gate accumulation is disabled
            T1_PS_1_256 &   // timer prescale is 256
            T1_SYNC_EXT_OFF & // don't synch with external clock
            T1_SOURCE_INT; // use the internal clock

  OpenTimer1( config, period );
  
  TRISEbits.TRISE1 = 0;  // prepare for the overrun LED indicator
  LATEbits.LATE1 = 0; // the LED should be off

  return;
}

/***************************************************/

// QEI interrupt handler

void __attribute__((interrupt,no_auto_psv)) _QEIInterrupt( void )
{ 
  extern int encindex;

 // update the encoder count every time the counter POSCNT gets to MAXCNT

  if (QEICONbits.UPDN )
  {
     encindex++;
  } 
  else
  {
     encindex--;
  }

  IFS2bits.QEIIF = 0;  // 
}    

/***********************************************/

// setup the QEI encoder

void encoder_init(void) {

unsigned int config1, config2;

config1 =  QEI_DIR_SEL_QEB &
           QEI_INT_CLK & 
           QEI_INDEX_RESET_DISABLE & // QEI index pulse resets postion counter 
           QEI_CLK_PRESCALE_1 &
           QEI_GATED_ACC_DISABLE &
           QEI_NORMAL_IO &  
           QEI_INPUTS_NOSWAP & 
           QEI_MODE_x2_MATCH &  // reset on match
           QEI_DOWN_COUNT & // count up
           QEI_IDLE_CON;  // continue on idle
          
config2 = POS_CNT_ERR_INT_DISABLE & // disable error interrupts
          QEI_QE_CLK_DIVIDE_1_1 &   //1_256
          QEI_QE_OUT_ENABLE &  // enable digital filter
          MATCH_INDEX_INPUT_PHASEA &
          MATCH_INDEX_INPUT_LOW;

OpenQEI( config1, config2 );
         
config1 = QEI_INT_ENABLE & // enable the interrupts
         QEI_INT_PRI_2 ;  // set the priority to two

WriteQEI( (unsigned int )MAXCNT );

ConfigIntQEI( config1 );
}

/********************************************************/

// setup ADC10

  void adc_init(void){

  unsigned int config1, config2, config3, configport, configscan;

  ADCON1bits.ADON = 0 ; // turn off ADC

  SetChanADC10(
       ADC_CH0_NEG_SAMPLEA_VREFN & // negative reference for channel 0 is VREF negative
       ADC_CH0_POS_SAMPLEA_AN2    // 
      // ADC_CHX_NEG_SAMPLEA_VREFN &  // negative reference for channel 1 is VREF negative
      // ADC_CHX_POS_SAMPLEA_AN3AN4AN5
                 );

  ConfigIntADC10( ADC_INT_DISABLE ); // disable the interrupts

  config1 = 
       ADC_MODULE_ON &  //turn on ADC module
       ADC_IDLE_CONTINUE & // let it idle if not in use
       ADC_FORMAT_INTG &   // unsigned integer format
       ADC_CLK_AUTO  &  // manual trigger source
       ADC_AUTO_SAMPLING_OFF  & // do not continue sampling
       ADC_SAMPLE_SIMULTANEOUS & // sample both channels at the same time
       ADC_SAMP_ON; // enable sampling

  config2 =
       ADC_VREF_EXT_EXT & // use voltage reference pins
       ADC_SCAN_OFF & // don't scan
       ADC_CONVERT_CH0 & // convert channel 0
       ADC_SAMPLES_PER_INT_1 & // 1 samples per interrupt
       ADC_ALT_BUF_OFF & // don't use the alternate buffer
       ADC_ALT_INPUT_OFF; // don't use an alternate input

  config3 = 
       ADC_SAMPLE_TIME_2 & // auto sample time bits
       ADC_CONV_CLK_SYSTEM & // use the system clock
       ADC_CONV_CLK_13Tcy;  // conversion clock speed (coeff of TCY is ADCS)
       
  configport = 
       ENABLE_AN2_ANA;   // parameters to be configured in the ADPCFG 
          
  configscan = 
       SCAN_NONE; // scan select parameter for the ADCSSL register
  
  OpenADC10( config1, config2, config3, configport, configscan );
}

/*********************************************************/

// setup pwm

  void pwm_init(void){

  unsigned int config1, config2, config3;
  unsigned int sptime;

  config1 = PWM_INT_DIS &     // disable the interrupt
            PWM_FLTA_DIS_INT; // disable the interrupt on fault

  ConfigIntMCPWM( config1 );

  config1 = PWM_EN & //  enable the PWM module
            PWM_IPCLK_SCALE1 & // input prescaler set to 1
            PWM_OP_SCALE1 & // post scalar set to 1
            PWM_MOD_UPDN; // free running mode

  config2 = PWM_MOD1_IND & // pwm modules run independently
            PWM_MOD2_IND & 
            PWM_MOD3_IND & 
            PWM_PDIS1H & // disable 1 high
            PWM_PDIS2H & // disable 2 high
            PWM_PEN3H & // enable 3 high
            PWM_PDIS1L & // disable 1 low
            PWM_PDIS2L & // disable 2 low
            PWM_PDIS3L ;  // disable 3 low

  config3 = PWM_UEN; // enable updates

  sptime = 0x0;

  OpenMCPWM(PERIOD, sptime, config1, config2, config3 );

}

/********************************************************/

// setup the UART

 void uart1_init(void) {
 
unsigned int config1, config2, ubrg;

config1 = UART_EN & // enable the UART
          UART_IDLE_CON & // set idle mode
          UART_DIS_WAKE & // disable wake-up on start
          UART_DIS_LOOPBACK & // disable loopback
          UART_DIS_ABAUD & // disable autobaud rate detect
          UART_NO_PAR_8BIT & // no parity, 8 bits
          UART_1STOPBIT;     // one stop bit

config2 = UART_INT_TX_BUF_EMPTY & // interrupt anytime a buffer is empty
          UART_TX_PIN_NORMAL & // set transmit break pin
          UART_TX_ENABLE & // enable UART transmission
          UART_INT_RX_CHAR & // receive interrupt mode selected
          UART_ADR_DETECT_DIS & // disable address detect
          UART_RX_OVERRUN_CLEAR; // overrun bit clear

ubrg = 15; // 115200 baud

OpenUART1( config1, config2, ubrg); 
}
 
/*****************************************************************/
//
// update an array
//
 void update_array( double arr[], int N)
 {
     int k;
     for (k=N-2; k>=0; k--) arr[k+1] = arr[k];
 }
 
  void filter( double A[], double B[], double fin[], double fout[], double newf, int N ){
     
     int k = 1;
     double sum;
     void update_array( double arr[], int N );
     
     update_array( fin, N );  // update the input array
     update_array( fout, N ); // update the output array
     
     fin[0] = newf;
     
     // now implement the filter
     
     sum = B[0]*fin[0];
     while(k < N){
         sum += B[k]*fin[k] - A[k]*fout[k];
         k++;
     }
     
     fout[0] = sum;
     
 }
 
 
/*****************************************************************/

int main ( void )
{
  extern int AD_value;
  extern unsigned int GO;
  extern int r1, r2, p1, p2;
  unsigned int dutycyclereg, dutycycle, period;
  char updatedisable;
  double convert_to_duty, time, dt = 0.05;
  int count, int_time, int_Rin, int_Rout, int_Y, int_R, int_u, int_Isum;
  double arg, speed, speed_scale;
  double error = 0.0, u;
  double Y, Yin[FILT_LEN] = {0.0, 0.0, 0.0, 0.0};
  double Yout[FILT_LEN] = {0.0, 0.0, 0.0, 0.0};
  double R, Rin[FILT_LEN] = {0.0, 0.0, 0.0, 0.0};
  double Rout[FILT_LEN] = {0.0, 0.0, 0.0, 0.0};
  double Uin[U_FILT_LEN] = {0.0, 0.0, 0.0, 0.0};
  double Uout[U_FILT_LEN] = {0.0, 0.0, 0.0, 0.0};
  double Rscal = 0.0;
  double A[FILT_LEN] = {1.0, -1.1619, 0.6959, -0.1378}, B[FILT_LEN] = {0.0495, 0.1486, 0.1486, 0.0495}; 
  double Au[U_FILT_LEN] = {1.0, -0.6, 0.0, 0.0, -0.4}, Bu[U_FILT_LEN] = {6.91, -6.53, 0.0, 0.0, 0.0}; 
  int Nr =FILT_LEN, Ny=FILT_LEN, Nu=U_FILT_LEN;
  double AD_scale = 0.1755;
  double last_speed = 0.0, last_u = 0.0;
  double last_error = 0.0, Derr = 0.0;
  double kp = 2.8, ki = 1.4*dt, kd = 0.1/dt, Isum = 0.0;
  double ref_scaling = 1.0;
  int ufp = 0.0, ufi = 0.0, ufd = 0.0;
  
  // set up the external interrupt
  
   Init_INT1();
  
  //  set up the A/D parameters

  adc_init();
 
  // set up the pwm

  pwm_init();

  // set up the uart

  uart1_init();

  // disable updates

  updatedisable = 0;
 
  // get some scaling out of the way

  convert_to_duty = ((double) MAX_DUTY)/1023.0;

  r1 = 0;
  r2 = 0;
  p1 = 0;
  p2 = 0;
  encindex = 0;

  // initialize timer1
  // dt can be no larger than 0.25 seconds

  dt = 0.05;  // the sampling interval in seconds

  // dt = N*256/29,480,000;  assuming a 256 prescaler.
  // so N = dt* 115156

  period = (unsigned int) (dt*115156.0);
 
  if (period > 32768 )
  {
     period = 32768;
  }
  printf("....period is %6u (should be < 32768) \n ", period);

  Init_Timer1( period );

  AD_value = 0;
  time = -dt;
  
  // set up the encoder to read the speed of the wheel

  // enable the input for QEI

  TRISBbits.TRISB3 = 1;
  TRISBbits.TRISB4 = 1;
  TRISBbits.TRISB5 = 1;
  
  // we also need to set these bits for the QEI 
  
  ADPCFGbits.PCFG3 = 1;
  ADPCFGbits.PCFG4 = 1;
  ADPCFGbits.PCFG5 = 1;

  encoder_init();

  // enable the QEI interrupt

  EnableIntQEI;

  // convert QEI encoder readings to radians

  speed_scale = (2.0*PI/720.0)/dt;
 

  TRISEbits.TRISE1 = 0;  // output when sampling too fast
  TRISEbits.TRISE3 = 0;  // pwm for servo

  count = MAX_COUNT;
  GO = 0;
  
/********************* MAIN LOOP **********************/

  while(1){

    while(!GO );

    // update the time

    time = time + dt;

    /*********************************************/ 
    //  implement the PREFILTER (Gpf) functions
    //
    //  the reference signal is the value in AD_value, and is
    //  read every time the Timer1 interrupt happens.
    //
    /*********************************************/

    // convert value read from pot [0-A2D_HIGH] to a speed [rad/sec]

    filter(A, B, Rin, Rout, (double) AD_value, Nr);
    R = Rout[0];
    R = max(0.0,R);
    R = min(R,A2D_HIGH);
    Rscal = R * AD_scale;
//    if (time < 0.5) Rscal = 0.0;
//    else if ((time >= 0.5) & (time < 8.5)) Rscal = 10.0;
//    else if ((time >= 8.5) & (time < 20.5)) Rscal = 25.0;
//    else if (time >= 20.5) Rscal = 40.0;
//    Rscal = 60.0;
    R = Rscal * ref_scaling;
    
    
    /*********************************************/ 
    //  implement the FEEDBACK (H) functions
    //
    //  Even if H is not explicitly written, we still need to
    //  sample the output and convert it to the correct units.       
    //  For the wheel system, the units are radians/second
    // 
    /*********************************************/

    // get the raw speed of the wheel from the QEI data
    // the if statement prevents overflow

    if ((r2-r1)< 0)
        arg = ((double) ((r2-r1)+65536))*720.0+ (double) (p2-p1);
    else
        arg = ((double) (r2-r1))*720.0 + (double) (p2-p1);
    
    speed = arg*speed_scale;
    speed = min(speed, last_speed+MAX_DELTA_SPEED);
    speed = max(speed, last_speed-MAX_DELTA_SPEED);
    speed = max(speed, 0.0);
    last_speed = speed;
    
    filter(A, B, Yin, Yout, (double) speed, Ny);
    Y = Yout[0];
    Y = max(0.0,Y);

    /*********************************************/ 
    //  implement the ERROR computation
    //
    //  The error is the difference between the (possibly)
    //  modified reference signal and the (possibly modified)
    //  output
    //
    /*********************************************/

//     error = R;      // use this for open loop control
     error = R - Y;  // use this for closed loop control

    /*********************************************/ 
    //  implement the CONTROLLER (Gc) functions
    //
    //  
    /*********************************************/

//     filter(Au, Bu, Uin, Uout, (double) error, Nu);
//     u = Uout[0];
     
     //I controller
     Isum += error;
     Isum = max(0.0, Isum);
     Isum = min(Isum, MAX_ISUM);
     
     //D controller
     Derr = error - last_error;
     last_error = error;
     
     u = kp*error + ki*Isum + kd*Derr; 
     ufp = (int) (kp*error*convert_to_duty/AD_scale);
     ufi = (int) (ki*Isum*convert_to_duty/AD_scale);
     ufd = (int) (kd*Derr*convert_to_duty/AD_scale);

    /*********************************************/ 
    // implement CONYTROl EFFORT CONVERSION
     //
    //  convert the control effort to the correct units for
    //  motor, be sure the control effort is within
    //  the allowable range
    //  
    /*********************************************/

    // scale = MAX_DUTY/1023
    // u/AD_scale corresponds to R/AD_scale
    // so u has units of MAX_DUTY/A2D_HIGH * [0-A2D_HIGH]
    
    u = u*convert_to_duty/AD_scale;  // convert back to a pwm signal
  
    u = min(u, last_u + MAX_DELTA_U);
    u = min(u,MAX_DUTY);  // don't let u get too large
    u = max(u,0.0);       // don't let u get negative, 
    last_u = u;
    
    dutycycle = (unsigned int) u;
 
    dutycyclereg = 3;
    SetDCMCPWM( dutycyclereg, dutycycle, updatedisable);
    
    /*********************************************/ 
    // PRINT OUTPUT
    // prepare to print out. All values should be scaled by
    // 100 and converted to integers. 
    // COUNT indicates how many time periods to wait before
    // printing out, usually it is set to 1
    // 
    /*********************************************/

   if (--count == 0)
   {
       int_time = (int)(100.0*time);  // convert for printout
       int_R = (int) (Rscal*100.0);
       int_u = (int) (u);
       int_Y = (int) (Y*100.0);
       int_Isum = (int) Isum;
       
       printf("%8d %8d %8d %8d %8d %8d %8d %8d\n", int_time, int_R, int_u, int_Y, int_Isum, ufp, ufi, ufd);
       count = MAX_COUNT ;
   }
    // save the current position
    r1 = r2;
    p1 = p2;
    
   GO = 0;  // all done
  } 
 
}
