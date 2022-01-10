/*
  Implementing and testing an IIR filter
*/

//#include <p30f4011.h>

#include <libpic30.h>
#include <uart.h>
#include <string.h>
#include <stdio.h>
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

#define MAX_COUNT 1

// define some variables

unsigned int GO;

/***************************************************************/

// external input interrupt handler

void __attribute__((interrupt,no_auto_psv)) _INT1Interrupt( void )
{
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
  extern unsigned int GO;
  
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
 void update_array( double arr[], int N )
 {
     int k;
     for (k=N-2; k>=0; k--) arr[k+1] = arr[k];
 }
 /*****************************************************************/
//
// implement an IIR filter
//  the output of the filter should be written to fout[0]

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
  extern unsigned int GO;
  double time, dt;
  int count, index, period;
  
  // Filter test1
  
// 
//  double Test_in[10] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
//  double Test_out[10] = {0.0317, 0.1731, 0.4455, 0.7523, 0.9799, 1.0865, 1.0955, 1.0566, 1.0128, 0.986};
//  double Rout[4] = {0.0,0.0,0.0,0.0};
//  double Rin[4] = {0.0,0.0,0.0,0.0};
//  double A[4] = {1.0, -1.459, 0.9104, -0.1978}, B[4] = {0.0317, 0.0951, 0.0951, 0.0317};
//  int Nr = 4, max_index = 9;
 
  
  // Filter test2
  
  
//  double Test_in[10] = { 1, 0, 1, -1, 1, -1, 1, 0, 1, -1};
//  double Test_out[10] = {0.0317, 0.1414, 0.3042, 0.4165, 0.3904, 0.2506, 0.0926, 0.0158, 0.0835, 0.2208};
//  double Rout[4] = {0.0,0.0,0.0,0.0};
//  double Rin[4] = {0.0,0.0,0.0,0.0};
//  double A[4] = {1.0, -1.459, 0.9104, -0.1978}, B[4] = {0.0317, 0.0951, 0.0951, 0.0317};
//  int Nr = 4, max_index = 9;
  
  
   // Filter test3
  
  double Test_in[10] = {-1, 1, 2, 3, 4, 3, 2, 1, 0, -1};
  double Test_out[10] = {2.0, 2.0, -3.0, -21.0, -47.0, -46.0, 33.0, 193.0, 281.0, -18.0};
  double Rout[3] = {0.0,0.0,0.0};
  double Rin[3] = {0.0,0.0,0.0};
  double A[3] = {1.0, -2.0, 3.0}, B[3] = {-2.0, 0.0, -3.0};
  int Nr = 3, max_index = 9; 
  
  
  // set up the external interrupt
  
   Init_INT1();

  // set up the uart

  uart1_init();

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
  printf("\n....period is %6u  (should be < 32768) \n", period);

  Init_Timer1( period );

  time = -dt;

  TRISEbits.TRISE1 = 0;  // output when sampling too fast

  count = MAX_COUNT;
  GO = 0;
  index = -1;
  
/********************* MAIN LOOP **********************/

  while(index < max_index){

    while(!GO );

    // update the time

    time = time + dt;
    index = index + 1;

    //  implement the IIR filter

    filter(A, B, Rin, Rout, Test_in[index], Nr);
   
   if (--count == 0)
   {
       printf("%8.4f %8.4f %8.4f  ERROR = > %8.4f\n", time, Rout[0], Test_out[index], Rout[0] - Test_out[index]);  
       count = MAX_COUNT ;
    
   GO = 0;  // all done
   } 
  }
  printf("\n");
  return(0);
}