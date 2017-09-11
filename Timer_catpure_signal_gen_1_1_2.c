/**
 * This is a very small example that shows how to use
 * === OUTPUT COMPARE and INPUT CAPTURE ===
 * The system uses hardware to generate precisely timed
 * pulses, then uses input capture tp compare the capture period
 * to the gereration period for acccuracy
 *
 * There is a command thread to set parametners
 * There is a capture time print-summary thread
 * There is a one second timer tick thread
 * 
 * -- Pin 4 is toggled by the timer2 interrupt
 * -- Pin 14 and 18 are output compare outputs
 * -- Pin 6 is input capture input -- connect this to one of the output compares
 *
 * -- Uart connections explained elsewhere
 * Modified by Bruce Land 
 * Jan 2015
 */

////////////////////////////////////
// clock AND protoThreads configure!
// You MUST check this file!
#include "config.h"
// threading library
#include "pt_cornell_1_1.h"

////////////////////////////////////
// graphics libraries
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>

// PORT B
#define EnablePullDownB(bits) CNPUBCLR=bits; CNPDBSET=bits;
#define DisablePullDownB(bits) CNPDBCLR=bits;
#define EnablePullUpB(bits) CNPDBCLR=bits; CNPUBSET=bits;
#define DisablePullUpB(bits) CNPUBCLR=bits;
//PORT A
#define EnablePullDownA(bits) CNPUACLR=bits; CNPDASET=bits;
#define DisablePullDownA(bits) CNPDACLR=bits;
#define EnablePullUpA(bits) CNPDACLR=bits; CNPUASET=bits;
#define DisablePullUpA(bits) CNPUACLR=bits;
////////////////////////////////////

// === thread structures ============================================
// thread control structs

// note that UART input and output are threads
static struct pt pt_print, pt_time, pt_keypad,pt_scankey;

// system 1 second interval tick
int sys_time_seconds ;

//The measured period of the wave
short capture1, last_capture1=0, capture_period=99 ;
//The actual period of the wave
int generate_period=10000 ;

int PushState;
int NoPush = 0;
int MaybeNoPush = 1;
int MaybePush = 2;
int Pushed = 3;
char buffer[60];

static int keypad, i, pattern;
static int keytable[12]={0x108, 0x81, 0x101, 0x201, 0x82, 0x102, 0x202, 0x84, 0x104, 0x204, 0x88, 0x208};
// == Capture 1 ISR ====================================================
// check every cpature for consistency
void __ISR(_INPUT_CAPTURE_1_VECTOR, ipl3) C1Handler(void)
{
    //capture1 = mIC1ReadCapture();
    
    //don't need the two lines below
    
//    capture_period = capture1 - last_capture1 ;
//    last_capture1 = capture1 ;
    // clear the timer interrupt flag
    mIC1ClearIntFlag();
}
/*
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    mT2ClearIntFlag();
    
    // generate  ramp
     DAC_data = (DAC_data + 1) & 0xfff ; // for testing
    
    // CS low to start transaction
     mPORTBClearBits(BIT_0); // start transaction
    // test for ready
     while (TxBufFullSPI2());
     // write to spi2
     WriteSPI2(DAC_config_chan_A | DAC_data);
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
     mPORTBSetBits(BIT_0); // end transaction
}
*/
// === Period print Thread ======================================================
// prints the captured period of the generated wave
static PT_THREAD (protothread_print(struct pt *pt))
{
    PT_BEGIN(pt);
      // string buffer
      char buffer[128];
      tft_setCursor(0, 0);
      tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
      tft_writeString("Connect pin18 OR pin14 to pin24\n");
      while(1) {
            // print every 200 mSec
            PT_YIELD_TIME_msec(200) ;
            // erase
            tft_fillRoundRect(0,50, 200, 20, 1, ILI9340_BLACK);// x,y,w,h,radius,color
            // print the periods
            tft_setCursor(0, 50);
             sprintf(buffer,"gen=%d cap=%d time=%d ",
                generate_period, capture_period, sys_time_seconds);
             
            tft_writeString(buffer);
      } // END WHILE(1)
  PT_END(pt);
} // thread 4

// === One second Thread ======================================================
// update a 1 second tick counter
static PT_THREAD (protothread_time(struct pt *pt))
{
    PT_BEGIN(pt);

      while(1) {
          //running at DDS sample rate to update the DDS and DAC
            // yield time 1 second
            sys_time_seconds++ ;
            // NEVER exit while
      } // END WHILE(1 )

  PT_END(pt);
} // 

static PT_THREAD (protothread_keypad(struct pt *pt))
{
    PT_BEGIN(pt);
    
    while(1) {
        //========================scan the keypad=============================
        tft_setCursor(30, 200);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        tft_writeString("entering keypad\n");
        //PT_SPAWN(pt,&pt_scankey,protothread_keypad(&pt_keypad));
        //==============run the debounce state machine=========================
        PT_YIELD_TIME_msec(30);
        
//        int keycode = -1; //keypad == TRUE;
//        int possible;
//        switch (PushState) {
//
//        case NoPush: 
//           if (keycode==-1) {
//               PushState=NoPush;
//               key = scan_keypad();
//           }
//           else 
//           {
//               PushState=MaybePush;
//               possible = keycode;
//           }
//           break;
//
//        case MaybePush:
//           if (keycode==possible) {
//              PushState=Pushed;  
//              digit = possible;
////              [more code to record a digit]
////              [or start a tone, or other event]
//           }
//           else 
//           {
//               PushState=NoPush;
//               key = scan_keypad();
//           }
//           break;
//
//        case Pushed:  
//           if (keycode==possible)
//           {
//               PushState=Pushed; 
//               key = scan_keypad();
//           }
//           else 
//           {
//               PushState=MaybeNoPush;
//               key = scan_keypad();
//           }
//           break;
//
//        case MaybeNoPush:
//           if (keycode==possible) 
//           {
//               PushState=Pushed;
//               key = scan_keypad();
//           }
//           else 
//           {
//               PushState=NoPush;
//               key = scan_keypad();
//           }
//           break;
//
//    } // end case
        //========places valid keypresses into a digit buffer and triggers speech production======================= 
        
        //=================handles erase/start commands=====================
        
        //=================waits for 30 milliseconds========================
        
        //============sets the DDS parameters for two sine waves (if any) which are playing===============
        
        //===================starts/stops the playback======================
        
        
    }
    PT_END(pt);
}

static PT_THREAD (protothread_scankey(struct pt *pt))
{
    PT_BEGIN(pt);
    // order is 0 thru 9 then * ==10 and # ==11
    // no press = -1
    // table is decoded to natural digit order (except for * and #)
    // 0x80 for col 1 ; 0x100 for col 2 ; 0x200 for col 3
    // 0x01 for row 1 ; 0x02 for row 2; etc
    // init the keypad pins A0-A3 and B7-B9
    // PortA ports as digital outputs
    //tft_fillRoundRect(30,200, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(30, 200);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"%d", i);
        if (i==10)sprintf(buffer,"*");
        if (i==11)sprintf(buffer,"#");
        tft_writeString(buffer);
    mPORTASetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3);    //Set port as output
    // PortB as inputs
    mPORTBSetPinsDigitalIn(BIT_7 | BIT_8 | BIT_9);    //Set port as input
    mPORTAClearBits(BIT_0 | BIT_1 | BIT_2 | BIT_3); 
        pattern = 1; mPORTASetBits(pattern);
        
        // yield time
        PT_YIELD_TIME_msec(30);
   
        for (i=0; i<4; i++) {
            keypad  = mPORTBReadBits(BIT_7 | BIT_8 | BIT_9);
            if(keypad!=0) {keypad |= pattern ; break;}
            mPORTAClearBits(pattern);
            pattern <<= 1;
            mPORTASetBits(pattern);
        }

        // search for keycode
        if (keypad > 0){ // then button is pushed
            for (i=0; i<12; i++){
                if (keytable[i]==keypad) break;
            }
        }
        else i = -1; // no button pushed
        
        // draw key number
        tft_fillRoundRect(30,200, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(30, 200);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"%d", i);
        if (i==10)sprintf(buffer,"*");
        if (i==11)sprintf(buffer,"#");
        tft_writeString(buffer);
    PT_END(pt);
}
// === Main  ======================================================

int main(void)
{
  
  // === Config timer and output compares to make pulses ========
  // set up timer2 to generate the wave period
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period);
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
  mT2ClearIntFlag(); // and clear the interrupt flag

  // set up compare3 for double compare mode
  // first number is the time to clear, second is the time to set the pin
  // in this case, the end of the timer period and 50% of the timer period
  OpenOC3(OC_ON | OC_TIMER2_SRC | OC_CONTINUE_PULSE , generate_period-1, generate_period>>1); //
  // OC3 is PPS group 4, map to RPB9 (pin 18)
  PPSOutput(4, RPB9, OC3);
  mPORTASetPinsDigitalOut(BIT_3);    //Set port as output -- not needed

  // set pulse to go high at 1/4 of the timer period and drop again at 1/2 the timer period
  OpenOC2(OC_ON | OC_TIMER2_SRC | OC_CONTINUE_PULSE, generate_period>>1, generate_period>>2);
  // OC2 is PPS group 2, map to RPB5 (pin 14)
  PPSOutput(2, RPB5, OC2);
  mPORTBSetPinsDigitalOut(BIT_5); //Set port as output -- not needed

  // === Config timer3 free running ==========================
  // set up timer3 as a souce for input capture
  // and let it overflow for contunuous readings
  OpenTimer3(T3_ON | T3_SOURCE_INT | T3_PS_1_2, 0xffff);

  // === set up input capture ================================
  OpenCapture1(  IC_EVERY_RISE_EDGE | IC_INT_1CAPTURE | IC_TIMER3_SRC | IC_ON );
  // turn on the interrupt so that every capture can be recorded
  ConfigIntCapture1(IC_INT_ON | IC_INT_PRIOR_3 | IC_INT_SUB_PRIOR_3 );
  INTClearFlag(INT_IC1);
  // connect PIN 24 to IC1 capture unit
  PPSInput(3, IC1, RPB13);

  //set up compare 1
  CMP1Open(CMP_ENABLE | CMP_OUTPUT_ENABLE | CMP1_NEG_INPUT_IVREF);
  PPSOutput(4, RPB9, C1OUT); //pin18
  mPORTBSetPinsDigitalIn(BIT_3); //Set port as input (pin 7 is RB3)
  
  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240
  tft_setCursor(0, 0);
  
  // === config the uart, DMA, vref, timer5 ISR ===========
  PT_setup();

  // === setup system wide interrupts  ====================
  INTEnableSystemMultiVectoredInt();
  
  // === now the threads ===================================
  // init the threads
  PT_INIT(&pt_print);
  PT_INIT(&pt_time);
  PT_INIT(&pt_keypad);
  //PT_INIT(&pt_blink);

  // schedule the threads
  while(1) {
    //PT_SCHEDULE(protothread_print(&pt_print));
    PT_SCHEDULE(protothread_time(&pt_time));
    //PT_SCHEDULE(protothread_findVoltage(&pt_findVoltage));
        
    PT_SCHEDULE(protothread_scankey(&pt_scankey));
    
    // PT_SCHEDULE(protothread_blink(&pt_blink));
  }
}// main
