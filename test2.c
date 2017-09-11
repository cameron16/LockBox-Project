 /*
 * File:        TFT_keypad_BRL4.c
 * Author:      Bruce Land
 * Adapted from:
 *              main.c by
 * Author:      Syed Tahmid Mahbub
 * Target PIC:  PIC32MX250F128B
 */

// graphics libraries
#include "config.h"
#include "tft_master.h"
#include "tft_gfx.h"
// need for rand function
#include <stdlib.h>

// threading library
#include <plib.h>
// config.h sets 40 MHz
#define	SYS_FREQ 40000000
#include "pt_cornell_1_2_1.h"
#include "voice.h"

#include <time.h>

#define NoPush 0
#define MaybePush 1
#define Pushed 2
#define MaybeNoPush 3

#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000
#define Fs 16000.0
#define two32 4294967296.0 // 2^32 

#define five_ms 80
#define sixtyfive_ms 1040

/* Demo code for interfacing TFT (ILI9340 controller) to PIC32
 * The library has been modified from a similar Adafruit library
 */
// Adafruit data:
/***************************************************
  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
// === 16:16 fixed point macros ==========================================
typedef signed int fix16 ;
#define multfix16(a,b) ((fix16)(((( signed long long)(a))*(( signed long long)(b)))>>16)) //multiply two fixed 16:16
#define float2fix16(a) ((fix16)((a)*65536.0)) // 2^16
#define fix2float16(a) ((float)(a)/65536.0)
#define fix2int16(a)    ((int)((a)>>16))
#define int2fix16(a)    ((fix16)((a)<<16))
#define divfix16(a,b) ((fix16)((((signed long long)(a)<<16)/(b)))) 
#define sqrtfix16(a) (float2fix16(sqrt(fix2float16(a)))) 
#define absfix16(a) abs(a)
#define onefix16 0x00010000 // int2fix16(1)

//sine lookup tables
#define sine_table_size 256
volatile fix16 sin_table[sine_table_size];

// for sine
#include <math.h>

//adc variables

static float V;
static fix16 Vfix, ADC_scale ;
static int adc_old;
//min and max values for adc
static int leftMinV1 = 320;//30;
static int leftMaxV1 = 380;//40;
static int centerMinV1=320;//880;
static int centerMaxV1=380;//900;
static int rightMinV1=320;//400;
static int rightMaxV1=380;//500;

static int leftMinV2;
static int leftMaxV2;
static int centerMinV2;
static int centerMaxV2;
static int rightMinV2;
static int rightMaxV2;

static int leftMinV2temp;
static int leftMaxV2temp;
static int centerMinV2temp;
static int centerMaxV2temp;
static int rightMinV2temp;
static int rightMaxV2temp;

static int potCodeChanged=0;

static int leftUnlock = 0;
static int centerUnlock = 0;
static int rightUnlock = 0;
static int adcHoldTime = 2000;//4000; //5000 milliseconds

// string buffer
char buffer[60];
static char buf[60];
static char key_buf[60];
static char freq_buf[60];
char cleared_buf[60];
char restart_buf[60];
char digit_buf[60];
char isr_buf[60];

static char codeBuf[4]; 
static int potBuf[3];
static int codeCounter = 0;

const static int password_length = 4;
static int firstNumber;
static int secondNumber;
static int thirdNumber;
static int fourthNumber;
//static int password[4] = {firstNumber,secondNumber,thirdNumber,fourthNumber};
static int password_checker = 0;

static int password[4];

static int keyCodeChanged = 0;
static int changePiezoLeft = 1;
static int changePiezoCenter= 0;
static int changePiezoRight = 0;

static int diffTimeChanged =0;
static int piezoChanged =0;

static int configureState = 0;

static int configRun = 0;

static int firstLowerBoundTime = 500;//550;
static int firstUpperBoundTime = 1500;//880;
static int secondLowerBoundTime = 50;//550;
static int secondUpperBoundTime = 500;//880;

static int adcRun = 0;

//static int diffTime;
//static int firstNumber = 0;
//static int secondNumber = 1;
//static int thirdNumber = 2;
//static int fourthNumber = 3;

volatile unsigned int vi, vj, packed, DAC_value2; // voice variables
//volatile unsigned /*?*/int CVRCON_setup; // stores the voltage ref config register after it is set up

static int keycode, keypad, i, restart_key, pound_key, curr_key, test_mode_key;
static int possible = -1;
static int digit = -1;
static int pattern, PushState;
//static int buf_count = 0;
volatile unsigned int isr_counter = 0;
//static int keypad, i, pattern;
//static int keytable[12]={0x108, 0x81, 0x101, 0x201, 0x82, 0x102, 0x202, 0x84, 0x104, 0x204, 0x88, 0x208};

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_input, pt_output, pt_DMA_output, pt_timer, pt_color, pt_anim, 
        pt_scanKey,pt_keypad, pt_setVoice, pt_unlock, pt_potRead, pt_adc, pt_piezo, pt_potUnlocked, pt_configureLock,
        pt_changePot, pt_changeKey, pt_changePiezo, pt_changeKeyHelper, pt_keypadConfigureState,
        pt_changeKeypad, pt_getNewPotValues, pt_printConfigScreen, pt_piezoUnlocked, pt_cmd2;

// system 1 second interval tick
int sys_time_seconds ;
//== Timer 2 interrupt handler ===========================================

volatile unsigned int DAC_data ;// output value
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 2 ; // 20 MHz max speed for this DAC
// the DDS units:
int buffer_ready = 0;
int phase_count = 0;
int test_mode = 0;

//== UART Variables ===============================
//static char cmd[16];
static int value;

int j = 0;
static int piezo_start;
static int average_piezo = 0;
static int num_knocks=0;

//unlocked variables 
//set to 1 iff unlocked
static int keypadUnlocked =0;
static int potUnlocked =0;
static int piezoUnlocked = 0;
static int fingerprintUnlocked=0;

static int changeKeyState =0;
static int confirmKeyState = 0;
static int changePiezoState =0;
static int confirmPiezoState =0;
static int piezoRange =50;

static int rightPiezoValue;
static int centerPiezoValue;
static int leftPiezoValue;





static int boxLocked =1;

void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    //int i;
//     tft_fillRoundRect(10,200, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(10, 200);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(isr_buf,"%d", isr_counter);
//        tft_writeString(isr_buf);
     mT2ClearIntFlag();
//    if(buffer_ready){
//        for(i=0;i<=buf_count-1;i++){
   
   

    // CS low to start transaction
     mPORTBClearBits(BIT_3); // start transaction
    // test for ready
     //while (TxBufFullSPI2()); we know it's empty
     // write to spi2
     WriteSPI2(DAC_config_chan_A | DAC_data); //
     //WriteSPI2(DAC_config_chan_B | DAC_data2); for voice
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
     mPORTBSetBits(BIT_3); // end transaction
     
     //--------------------VOICE---------------------------
     
     //mT2ClearIntFlag();
    // do the Direct Digital Synthesis
    
     //idx1 = 0;
     //idx2 = 0;
    // CS low to start transaction
     mPORTBClearBits(BIT_3); // start transaction
    // test for ready
     //while (TxBufFullSPI2()); we know it's empty
     // write to spi2
     WriteSPI2(DAC_config_chan_B | DAC_value2); //
     //WriteSPI2(DAC_config_chan_B | DAC_data2); for voice
    // test for done
    while (SPI2STATbits.SPIBUSY); // wait for end of transaction
     // CS high
     mPORTBSetBits(BIT_3); // end transaction
     
}

// === Timer Thread =================================================
// update a 1 second tick counter
static PT_THREAD (protothread_timer(struct pt *pt))
{
    PT_BEGIN(pt);

      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++ ;
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread





// === Keypad Thread =============================================
// connections:
// A0 -- row 1 -- thru 300 ohm resistor -- avoid short when two buttons pushed
// A1 -- row 2 -- thru 300 ohm resistor
// A2 -- row 3 -- thru 300 ohm resistor
// A3 -- row 4 -- thru 300 ohm resistor
// B7 -- col 1 -- 10k pulldown resistor -- avoid open circuit input when no button pushed
// B8 -- col 2 -- 10k pulldown resistor
// B9 -- col 3 -- 10k pulldown resistor

    
static PT_THREAD (protothread_scanKey(struct pt *pt)) //SCAN KEYPAD
{
    PT_BEGIN(pt);
    
    // order is 0 thru 9 then * ==10 and # ==11
    // no press = -1
    // table is decoded to natural digit order (except for * and #)
    // 0x80 for col 1 ; 0x100 for col 2 ; 0x200 for col 3
    // 0x01 for row 1 ; 0x02 for row 2; etc
    static int keytable[12]={0x108, 0x81, 0x101, 0x201, 0x82, 0x102, 0x202, 0x84, 0x104, 0x204, 0x88, 0x208};
    // init the keypad pins A0-A3 and B7-B9
    // PortA ports as digital outputs
    pound_key = 0;
    restart_key=0;
    mPORTASetPinsDigitalOut(BIT_0 | BIT_1 | BIT_2 | BIT_3);    //Set port as output
    // PortB as inputs
    mPORTBSetPinsDigitalIn(BIT_7 | BIT_8 | BIT_9);    //Set port as input
    // read each row sequentially
    mPORTAClearBits(BIT_0 | BIT_1 | BIT_2 | BIT_3);
    pattern = 1; mPORTASetBits(pattern);
   
    for (i=0; i<4; i++) {
        keypad  = mPORTBReadBits(BIT_7 | BIT_8 | BIT_9);
        if(keypad!=0) {
            keypad |= pattern; 
            break;
        }
        mPORTAClearBits(pattern);
        pattern <<= 1;
        mPORTASetBits(pattern);
    }

    // search for keycode
    if (keypad > 0){ // then button is pushed
        for (i=0; i<12; i++){
            if (keytable[i]==keypad){  
                break;
            }
        }
    }
    else {
        i = -1; // no button pushed   
    }
    keycode = i;
        
    if(keycode==11){
        pound_key = 1;
    }
        
    if(keycode==10){//10
        restart_key = 1;
        
    }
    PT_END(pt);
} // keypad thread

static PT_THREAD (protothread_unlock(struct pt *pt))
{
    static int i;
    PT_BEGIN(pt);
    tft_fillScreen(ILI9340_BLACK);
    tft_setCursor(0, 10);
    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
    tft_writeString("KEYPAD IS UNLOCKED");
    tft_setCursor(0,30);
    tft_writeString("MOVING ONTO POT IN:");
    i=5;
    char myBuf[60];
    while (i>-1){
        tft_fillRect(100,50,100,30,ILI9340_BLACK);
        tft_setCursor(100,50);
        sprintf(myBuf,"%d",i);
        tft_writeString(myBuf);
        PT_YIELD_TIME_msec(1000);
        i=i-1;
    }
    tft_fillScreen(ILI9340_BLACK);
    tft_fillRect(0,10,15,200,ILI9340_BLACK);
    tft_setCursor(0,10);
    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
    tft_writeString("Configure Pots");
    keypadUnlocked = 1;

  PT_END(pt);
} // timer thread

static PT_THREAD (protothread_keypad(struct pt *pt)) //THE THREAD W/ STUFF
{
    PT_BEGIN(pt);
    if (!configureState && !changeKeyState){
        tft_fillScreen(ILI9340_BLACK);
        tft_setCursor(10, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        tft_writeString("Enter 4-Digit Code");
    }  
    int res;
    while(1) {
        if (keyCodeChanged==1){
            password[0] = firstNumber;
            password[1] = secondNumber;
            password[2] = thirdNumber;
            password[3] = fourthNumber; 
        }
        else if (keyCodeChanged ==0){
            password[0] = 1;
            password[1] = 2;
            password[2] = 3;
            password[3] = 4;
        }
        //========================scan the keypad=============================
        PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
        //==============run the debounce state machine=========================
        PT_YIELD_TIME_msec(30);
        
        switch (PushState) {   
        case 0: //NoPush
           if (keycode==-1) {
                PushState=NoPush;
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
           }
           else //keycode is not -1, so there was some push
           {
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
                PushState=MaybePush;
                possible = keycode;
           }
           break;
        case 1: //MaybePush
           if (keycode==possible) {
                PushState=Pushed;  
                digit=possible;
                char buf3[2];
                //add digit to buffer
                if(digit < 10 && digit > -1) {
                        tft_fillScreen(ILI9340_BLACK);
                        tft_setCursor(120, 160);
                        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(5);
                        sprintf(buf,"%d",digit);
                        tft_writeString(buf);
                        
                        codeBuf[codeCounter] = digit;
                    
                        if (codeCounter < 4){
                            codeCounter++;
                        }
                        if (codeCounter == 4){
                            //check if numbers in code buf equal the required code
                            int i;
                            int unlockMe=1;
                            for(i =0; i<4; i++){
                                if (codeBuf[i]!=password[i]){
                                    unlockMe=0;
                                }
                            }
                            if (unlockMe==1){
                                codeCounter = 0;
                                PT_SPAWN(pt,&pt_unlock,protothread_unlock(&pt_unlock));
                            }
                            else{ //wrong code entered. output something to signify wrong passcode
                                //WRONG
                                //set codeCounter back to 0 so user can try again
                                tft_fillScreen(ILI9340_BLACK);
                                tft_setCursor(0, 10);
                                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                             
                                tft_setCursor(0,50);
                                tft_writeString("WRONG PASSCODE");
                                tft_setCursor(0,80);
                                tft_writeString("Try again");
                                codeCounter = 0;
                                //start over   
                            }
                        } 
                }
                if(restart_key==1){
                    //restart, so set the counter back to 0 so user can try again
                    codeCounter = 0;
                    tft_fillScreen(ILI9340_BLACK);
                    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                    tft_setCursor(0, 10);
                    tft_writeString("RESTART GUY");                     
                    //buf_count = 0;
                    restart_key=0;
                    //voice = 0;       
                }
           }
           else 
           {
               PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
               PushState=NoPush;
           }
           break;
        case 2: //Pushed
           if (keycode==possible)
           {
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
                PushState=Pushed; 
           }
           else 
           {
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
                PushState=MaybeNoPush;
           }
           break;
        case 3: //MaybeNoPush
           if (keycode==possible) 
           {
                PushState=Pushed;
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
           }
           else 
           {
               PushState=NoPush;
               PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
           }
           break;

        } // end case
        
        //========places valid keypresses into a digit buffer and triggers speech production======================= 
        
        //=================handles erase/start commands=====================
        
        //=================waits for 30 milliseconds========================
        
        //============sets the DDS parameters for two sine waves (if any) which are playing===============
        
        //===================starts/stops the playback======================
    }
    PT_END(pt);
}

static PT_THREAD (protothread_potUnlocked(struct pt *pt))
{
    static int i;
    PT_BEGIN(pt);
        // draw sys_time
       // tft_fillRect(0,0, 100, 100, ILI9340_BLACK);// x,y,w,h,color
        tft_fillScreen(ILI9340_BLACK);
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        tft_writeString("POT IS UNLOCKED");
        tft_setCursor(0,30);
        tft_writeString("MOVING ONTO PIEZO IN:");
        i=5;
        char myBuf[60];
        while (i>-1){
            tft_fillRect(100,50,100,30,ILI9340_BLACK);
            tft_setCursor(100,50);
            sprintf(myBuf,"%d",i);
            tft_writeString(myBuf);
            PT_YIELD_TIME_msec(1000);
            i=i-1;
        }
        tft_fillScreen(ILI9340_BLACK);
        tft_fillRect(0,10,80,20,ILI9340_BLACK);//x,y,w,h
        tft_setCursor(0,10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        tft_writeString("Enter Knock Pattern");
        potUnlocked = 1;

       // END WHILE(1)
  PT_END(pt);
} // timer thread


// === ADC Thread =============================================
// 

static PT_THREAD (protothread_adc(struct pt *pt))
{   
    PT_BEGIN(pt);
    tft_fillRect(0,10,80,20,ILI9340_BLACK);//x,y,w,h
    tft_setCursor(0,10);
    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
    tft_writeString("Configure Pots");
    static int adc_9;
    static float V;
    static fix16 Vfix, ADC_scale ;
    
    ADC_scale = float2fix16(3.3/1023.0); //Vref/(full scale)
    
    tft_fillScreen(ILI9340_BLACK);
    tft_setCursor(0,10);
    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
    tft_writeString("Configure Pots");
    
    static int left_pot;
    static int center_pot;
    static int right_pot;
    static int leftMin;
    static int leftMax;
    static int rightMin;
    static int rightMax;
    static int centerMin;
    static int centerMax;
    static int startTime;
    
    while(1) {
        adcRun=0;
        if (potCodeChanged==0){
            leftMin = leftMinV1;
            leftMax = leftMaxV1;
            rightMin = rightMinV1;
            rightMax = rightMaxV1;
            centerMin = centerMinV1;
            centerMax = centerMaxV1;     
        }
        else{
            leftMin = leftMinV2;
            leftMax=leftMaxV2;
            rightMin = rightMinV2;
            rightMax = rightMaxV2;
            centerMin = centerMinV2;
            centerMax = centerMaxV2;  
        }
        PT_YIELD_TIME_msec(60);
        SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11);
        mPORTBSetPinsDigitalOut(BIT_5 | BIT_15);
        
      // READ FROM CHANNEL 0
        mPORTBClearBits(BIT_5 | BIT_15);
        AcquireADC10();
        PT_YIELD_TIME_msec(1);
        left_pot = ReadADC10(0);
        left_pot = (899*left_pot)/1024 +100;
        
        //READ FROM CHANNEL 1
        mPORTBClearBits(BIT_5 | BIT_15);
        mPORTBSetBits(BIT_5);
        AcquireADC10();
        PT_YIELD_TIME_msec(1);
        center_pot = ReadADC10(0);
        center_pot = (899*center_pot)/1024 +100;
        
        //READ FROM CHANNEL 2
        mPORTBClearBits(BIT_5 | BIT_15);
        mPORTBSetBits(BIT_15);
        AcquireADC10();
        PT_YIELD_TIME_msec(1);
        right_pot = ReadADC10(0);
        right_pot = (899*right_pot)/1024 +100;

        //display left pot
        tft_fillRect(0,100,200,15,ILI9340_BLACK); //x,y,w,h,color
        sprintf(buffer,"Left Pot: %d", left_pot);
        tft_setCursor(10,100);
        tft_writeString(buffer);
        
        //display center pot
        tft_fillRect(0,150,200,15,ILI9340_BLACK); //x,y,w,h,color
        sprintf(buffer,"Center Pot: %d", center_pot);
        tft_setCursor(10,150);
        tft_writeString(buffer);
     
        //display right pot
        tft_fillRect(0,200,200,15,ILI9340_BLACK); //x,y,w,h,color
        sprintf(buffer,"Right Pot: %d", right_pot);
        tft_setCursor(10,200);
        tft_writeString(buffer);
        
        int adcChanged = 0;
        leftUnlock = left_pot > leftMin && left_pot < leftMax;
        centerUnlock = center_pot > centerMin && center_pot < centerMax;
        rightUnlock = right_pot > rightMin && right_pot < rightMax;
        
        if (leftUnlock && centerUnlock && rightUnlock){
            startTime = PT_GET_TIME();
            static int diffTime = 0;
            if (diffTimeChanged ==1){
                diffTime =0;
            }
            else{
                diffTime=0;
            }
            while (diffTime<adcHoldTime){
                PT_YIELD_TIME_msec(60);
                diffTime = PT_GET_TIME() - startTime;
                mPORTBSetPinsDigitalOut(BIT_5 | BIT_15);
        
                // READ FROM CHANNEL 0
                mPORTBClearBits(BIT_5 | BIT_15);
                AcquireADC10();
                PT_YIELD_TIME_msec(1);
                left_pot = ReadADC10(0);
                left_pot = (899*left_pot)/1024 +100;

                tft_fillRect(0,100,200,15,ILI9340_BLACK); //x,y,w,h,color
                sprintf(buffer,"Left Pot: %d", left_pot);
                tft_setCursor(10,100);
                tft_writeString(buffer);
                if(left_pot < leftMin || left_pot > leftMax){
                    adcChanged = 1;
                    break;
                }
                
                //READ FROM CHANNEL 1
                mPORTBClearBits(BIT_5 | BIT_15);
                mPORTBSetBits(BIT_5);
                AcquireADC10();
                PT_YIELD_TIME_msec(1);
                center_pot = ReadADC10(0);
                center_pot = (899*center_pot)/1024 +100;

                tft_fillRect(0,150,200,15,ILI9340_BLACK); //x,y,w,h,color
                sprintf(buffer,"Center Pot: %d", center_pot);
                tft_setCursor(10,150);
                tft_writeString(buffer);
                if (center_pot < centerMin || center_pot > centerMax){
                    adcChanged =1;
                    break;
                } 
       
                //READ FROM CHANNEL 2
                mPORTBClearBits(BIT_5 | BIT_15);
                mPORTBSetBits(BIT_15);
                AcquireADC10();
                PT_YIELD_TIME_msec(1);
                right_pot = ReadADC10(0);
                right_pot = (899*right_pot)/1024 +100;

               
                //display adc_12
                tft_fillRect(0,200,200,15,ILI9340_BLACK); //x,y,w,h,color
                sprintf(buffer,"Right Pot: %d", right_pot);
                tft_setCursor(10,200);
                tft_writeString(buffer);
                if (right_pot < rightMin || right_pot > rightMax){
                    adcChanged =1;
                    break;
                } 
            }
            if (!adcChanged){ //adc values did not change 
                PT_SPAWN(pt,&pt_potUnlocked,protothread_potUnlocked(&pt_potUnlocked));
            }
        }
        // NEVER exit while
      } // END WHILE(1)
    PT_END(pt);

} // animation thread



static PT_THREAD (protothread_piezoUnlocked(struct pt *pt))
{
    static int i;
    PT_BEGIN(pt);
    tft_fillScreen(ILI9340_BLACK);
    tft_setCursor(0, 10);
    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
    tft_writeString("PIEZO IS UNLOCKED");
    tft_setCursor(0,30);
    tft_writeString("MOVING ONTO FPS IN:");
    i=5;
    char myBuf[60];
    while (i>-1){
        tft_fillRect(100,50,100,30,ILI9340_BLACK);
        tft_setCursor(100,50);
        sprintf(myBuf,"%d",i);
        tft_writeString(myBuf);
        PT_YIELD_TIME_msec(1000);
        i=i-1;
    }
    tft_fillScreen(ILI9340_BLACK);
    piezoUnlocked=1;
    tft_setCursor(0, 10);
    tft_setTextColor(ILI9340_YELLOW); 
    tft_setTextSize(2);
    tft_writeString("Place finger on");
    tft_setCursor(0,30);
    tft_writeString("fingerprint scanner");
    PT_END(pt);
}

static PT_THREAD (protothread_piezo(struct pt *pt))
{ 
    PT_BEGIN(pt);
    static int firstKnock = 0;
    static int secondKnockValid = 0;
    static int firstKnockTime;
    static int firstLowerBound;
    static int firstUpperBound;

    static int second_piezo_val;
    static int secondKnockTime;
    static int secondLowerBound;
    static int secondUpperBound;
    static int thirdKnockValid=0;
    static int third_piezo_val;
    static int thirdKnockTime;
    static int holdBetweenKnock = 50;
    static int knockThreshold = 25;//6;
    
    static int secondHitInInterval=0;
    static int thirdHitInInterval = 0;
    piezo_start=1;
    static char myFavBuf3[60];
    
    static int elapsedTime;
    static int readTime;
    num_knocks =0;
    static int adc_piezo;
    static int diff;
    
    while(1){
        while(piezoChanged){
            num_knocks=0;
            //PT_YIELD_TIME_msec(5000);
            piezoChanged =0;
        }
        firstKnock =0;
        secondKnockValid =0;
        secondHitInInterval =0;
        thirdHitInInterval =0;
        thirdKnockValid = 0;
        SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11);
        mPORTBSetPinsDigitalOut(BIT_5 | BIT_15);
        mPORTBClearBits(BIT_5 | BIT_15);
        mPORTBSetBits(BIT_5 | BIT_15);
        average_piezo = 30;//20;
        tft_fillRect(10,300,200,15,ILI9340_BLACK); //x,y,w,h,color
        sprintf(buffer,"Avg: %d", average_piezo);
        tft_setCursor(10,300);
        //tft_writeString(buffer);
        AcquireADC10();
        PT_YIELD_TIME_msec(1);
        adc_piezo = ReadADC10(0);
        tft_fillRect(10,100,200,15,ILI9340_BLACK); //x,y,w,h,color
        sprintf(buffer,"Piz: %d", adc_piezo);
        tft_setCursor(10,100);
        //tft_writeString(buffer);
        //int diff = adc_piezo - average_piezo;
        diff = adc_piezo - average_piezo;
        if (abs(diff)>knockThreshold || adc_piezo ==0){
            num_knocks++;
            tft_fillRect(10,250,200,15,ILI9340_BLACK); //x,y,w,h,color
            sprintf(buffer,"K #: %d", num_knocks);
            tft_setCursor(10,250);
            //tft_writeString(buffer); 
            firstKnock = 1;
            PT_YIELD_TIME_msec(holdBetweenKnock);
        }
        else{
            firstKnock=0;
        }
        if (firstKnock){
            firstKnockTime = PT_GET_TIME();
            firstLowerBound = firstKnockTime + firstLowerBoundTime;
            firstUpperBound = firstKnockTime + firstUpperBoundTime;
            //int readTime = firstUpperBound - firstKnockTime;
            //int elapsedTime =0;
            readTime = firstUpperBound - firstKnockTime;
            elapsedTime = 0;
            tft_fillRect(10,50,200,15,ILI9340_BLACK); //x,y,w,h,color
            tft_setCursor(10,50);
            //tft_writeString("Waiting for 2nd knock");
            while (readTime >elapsedTime && !secondKnockValid){
                AcquireADC10();
                PT_YIELD_TIME_msec(1);
                second_piezo_val = ReadADC10(0);
                diff = second_piezo_val - average_piezo;
                if (abs(diff)>knockThreshold){
                    secondKnockTime = PT_GET_TIME();
                    num_knocks++;
                    tft_fillRect(10,250,200,15,ILI9340_BLACK); //x,y,w,h,color
                    sprintf(buffer,"K #: %d", num_knocks);
                    tft_setCursor(10,250);
                    //tft_writeString(buffer); 
                    if (secondKnockTime >= firstLowerBound){ //knock occured at valid time range
                        tft_fillRect(10,150,200,15,ILI9340_BLACK); //x,y,w,h,color
                        tft_setCursor(10,150);
                        //tft_writeString("2nd knock good"); 
                        secondKnockValid = 1;
                        tft_fillRect(10,50,200,15,ILI9340_BLACK); //x,y,w,h,color //erase waiting for 2nd lock
                        PT_YIELD_TIME_msec(holdBetweenKnock);
                        //while(1);
                        break;
                    }
                    else if (secondKnockTime < firstLowerBound){
                        secondKnockValid =0;
                        secondHitInInterval =1;
                        firstKnock = 0;
                        tft_fillRect(10,150,200,15,ILI9340_BLACK); //x,y,w,h,color
                        tft_setCursor(10,150);
                        //tft_writeString("Too early 2nd knock"); 
                        tft_fillRect(10,50,200,15,ILI9340_BLACK); //x,y,w,h,color //erase waiting for 2nd lock
                        PT_YIELD_TIME_msec(holdBetweenKnock);
                        break;
                    }
                }
                elapsedTime = PT_GET_TIME() - firstKnockTime;
            }
            if (!secondHitInInterval && !secondKnockValid){ //there was no 2nd knock in the time range
                tft_fillRect(10,150,200,15,ILI9340_BLACK); //x,y,w,h,color
                tft_setCursor(10,150);
                //tft_writeString("Too late 2nd knock"); 
                tft_fillRect(10,50,200,15,ILI9340_BLACK); //x,y,w,h,color //erase waiting for 2nd lock
                firstKnock = 0;
            }
            if (secondKnockValid){
                secondKnockTime=PT_GET_TIME();
                secondLowerBound = secondKnockTime + secondLowerBoundTime;
                secondUpperBound = secondKnockTime + secondUpperBoundTime;
                readTime = secondUpperBound - secondKnockTime;
                elapsedTime =0;
                tft_fillRect(10,50,200,15,ILI9340_BLACK); //x,y,w,h,color
                tft_setCursor(10,50);
                //tft_writeString("Waiting for 3rd knock");
                while (readTime >elapsedTime && !thirdKnockValid){
                    AcquireADC10();
                    PT_YIELD_TIME_msec(1);
                    third_piezo_val = ReadADC10(0);
                    diff = third_piezo_val - average_piezo;
                    if (abs(diff)>knockThreshold){
                        thirdKnockTime = PT_GET_TIME();
                        num_knocks++;
                        tft_fillRect(10,250,200,15,ILI9340_BLACK); //x,y,w,h,color
                        sprintf(buffer,"K #: %d", num_knocks);
                        tft_setCursor(10,250);
                        //tft_writeString(buffer); 
                        if (thirdKnockTime >= secondLowerBound){ //knock occured at valid time range
                            tft_fillRect(10,280,200,15,ILI9340_BLACK); //x,y,w,h,color
                            tft_setCursor(10,280);
                            //tft_writeString("Knock Unlocked"); 
                            thirdKnockValid = 1;
                            num_knocks = 0;
                            secondKnockValid =0;
                            firstKnock=0;
                            piezo_start=1;
                            PT_SPAWN(pt,&pt_piezoUnlocked,protothread_piezoUnlocked(&pt_piezoUnlocked)); 
                            break;
                        }
                        else if (thirdKnockTime < secondLowerBound){
                            thirdKnockValid =0;
                            firstKnock = 0;
                            secondKnockValid = 0;
                            thirdHitInInterval = 1;
                            //sum = 0;
                            
                            tft_fillRect(10,280,200,15,ILI9340_BLACK); //x,y,w,h,color
                            tft_setCursor(10,280);
                            //tft_writeString("Too early 3rd knock"); 
                            
                            tft_fillRect(10,50,200,15,ILI9340_BLACK); //x,y,w,h,color //erase waiting for 3rd knock

                            PT_YIELD_TIME_msec(holdBetweenKnock);
                        }
                    }
                    elapsedTime = PT_GET_TIME() - secondKnockTime;
                }
                if (!thirdHitInInterval && !thirdKnockValid){ //no 3rd knock in range
                    tft_fillRect(10,280,200,15,ILI9340_BLACK); //x,y,w,h,color
                    tft_setCursor(10,280);
                    //tft_writeString("Too late 3rd knock"); 
                    tft_fillRect(10,50,200,15,ILI9340_BLACK); //x,y,w,h,color //erase waiting for 3rd knock
                    
                    firstKnock = 0;
                    secondKnockValid = 0;
                    thirdKnockValid = 0;
                }
            }
        } 
    }
    PT_END(pt);
}

static PT_THREAD (protothread_changePiezo(struct pt *pt))
{
    PT_BEGIN(pt);
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN11);
    mPORTBSetPinsDigitalOut(BIT_5 | BIT_15);
    mPORTBClearBits(BIT_5 | BIT_15);
    mPORTBSetBits(BIT_5 | BIT_15);
    static int knockThreshold2 = 25;//6;
    static int adc_piezo2;
    static int average_piezo2 = 30;
    static int diff2;
    static int firstKnockTime;
    static int secondKnockTime;
    static int thirdKnockTime;
    static int firstDiff;
    static int secondDiff;
    static int holdBetweenKnock2 = 50;
    tft_fillRect(10,300,200,15,ILI9340_BLACK); //x,y,w,h,color
    sprintf(buffer,"Avg: %d", average_piezo);
    tft_setCursor(10,300);

    AcquireADC10();
    PT_YIELD_TIME_msec(1);
    adc_piezo2 = ReadADC10(0);
    diff2 = adc_piezo2 - average_piezo2;
    while(!(abs(diff2)>knockThreshold2 || adc_piezo2 ==0)){
        //wait until we get a knock
        AcquireADC10();
        PT_YIELD_TIME_msec(1);
        adc_piezo2 = ReadADC10(0);
        diff2 = adc_piezo2 - average_piezo2;
    }
    firstKnockTime = PT_GET_TIME();
    
    PT_YIELD_TIME_msec(holdBetweenKnock2);
    AcquireADC10();
    PT_YIELD_TIME_msec(1);
    adc_piezo2 = ReadADC10(0);
    diff2 = adc_piezo2 - average_piezo2;
    while(!(abs(diff2)>knockThreshold2 || adc_piezo2 ==0)){
        //wait until we get 2nd knock
        AcquireADC10();
        PT_YIELD_TIME_msec(1);
        adc_piezo2 = ReadADC10(0);
        diff2 = adc_piezo2 - average_piezo2;
    }
    secondKnockTime = PT_GET_TIME();
    
    PT_YIELD_TIME_msec(holdBetweenKnock2);
    AcquireADC10();
    PT_YIELD_TIME_msec(1);
    adc_piezo2 = ReadADC10(0);
    diff2 = adc_piezo2 - average_piezo2;
    while(!(abs(diff2)>knockThreshold2 || adc_piezo2 ==0)){
        //wait until we get 3rd knock
        AcquireADC10();
        PT_YIELD_TIME_msec(1);
        adc_piezo2 = ReadADC10(0);
        diff2 = adc_piezo2 - average_piezo2;
    }
    thirdKnockTime = PT_GET_TIME();
    firstDiff = secondKnockTime - firstKnockTime;
    secondDiff = thirdKnockTime - secondKnockTime;
    firstLowerBoundTime = firstDiff - 300;
    if (firstLowerBoundTime <50){
        firstLowerBoundTime = 50;
    }
    firstUpperBoundTime = firstDiff + 300;
    secondLowerBoundTime = secondDiff - 300;
    if (secondLowerBoundTime < 50){
        secondLowerBoundTime =50;
    }
    secondUpperBoundTime = secondDiff + 300;
    tft_fillScreen(ILI9340_BLACK);
    tft_setCursor(50, 10);
    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
    tft_writeString("Knock Pattern");
    tft_setCursor(50,40);
    tft_writeString("Registered");
    PT_YIELD_TIME_msec(2000);
    PT_END(pt);
}


static PT_THREAD (protothread_printConfigScreen(struct pt *pt)) //THE THREAD W/ STUFF
{
    PT_BEGIN(pt);
    tft_fillScreen(ILI9340_BLACK);
    tft_setCursor(10, 10);
    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
    tft_writeString("BOX IS UNLOCKED");

    tft_setCursor(10,50);
    tft_writeString("Choose an option:");

    tft_setTextSize(1);

    tft_setCursor(10,80);
    tft_writeString("1: Unlock box");

    tft_setCursor(10,110);
    tft_writeString("2: Change keypad password");

    tft_setCursor(10,140);
    tft_writeString("3: Change pot range");

    tft_setCursor(10,170);
    tft_writeString("4: Change knock pattern");
    PT_END(pt);
}

static PT_THREAD (protothread_keypadConfigureState(struct pt *pt)) //THE THREAD W/ STUFF
{
    PT_BEGIN(pt);
    
    //piezo_start=1;
    if (changeKeyState){
        tft_fillScreen(ILI9340_BLACK);
        tft_setCursor(10, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
        tft_writeString("Enter New 4-Digit Code");
    }
    int res;
    while(1) {
        if (configureState && configRun){
            PT_SPAWN(pt,&pt_printConfigScreen,protothread_printConfigScreen(&pt_printConfigScreen));
        }
        configRun=0;
        //========================scan the keypad=============================
        PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
        //==============run the debounce state machine=========================
        PT_YIELD_TIME_msec(30);
        
        switch (PushState) {   
        case 0: //NoPush
           if (keycode==-1) {
                PushState=NoPush;
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
           }
           else //keycode is not -1, so there was some push
           {
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
                PushState=MaybePush;
                possible = keycode;
           }
           break;
        case 1: //MaybePush
            if (keycode==possible) {
                PushState=Pushed;  
                digit=possible;
                char buf3[2];
                //add digit to buffer
                if(digit < 10 && digit > -1) {
                    if (configureState){
                        if (digit == 1){ //lock everything up
                            keypadUnlocked =0;
                            potUnlocked =0;
                            piezoUnlocked =0;
                            configureState = 0;
                            fingerprintUnlocked =0;
                            boxLocked =1;
                            codeCounter=0;
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            tft_writeString("Enter 4-Digit Code"); 
                        }
                        else if (digit == 2){ //change keypad password
                            changeKeyState=1;
                            codeCounter = 0;
                            configureState = 0;
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
                            tft_writeString("Enter New 4-Digit Code");
                        }
                        else if (digit == 3){ //change pot range
                            changePiezoState = 1;
                            configureState =0;
                            codeCounter = 0;
                            changePiezoLeft = 1;
                            changePiezoCenter= 0;
                            changePiezoRight = 0;
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10,100);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            tft_writeString("Enter Left");
                            tft_setCursor(10,120);
                            tft_writeString("Pot Value");
                        }
                        else if (digit == 4){ //change knock pattern
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
                            tft_writeString("Knock Box 3 Times");
                            PT_SPAWN(pt,&pt_changePiezo,protothread_changePiezo(&pt_changePiezo));
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            tft_writeString("BOX IS UNLOCKED");
    
                            tft_setCursor(10,50);
                            tft_writeString("Choose an option:");
    
                            tft_setTextSize(1);
                            tft_setCursor(10,80);
                            tft_writeString("1: Lock box");
    
                            tft_setCursor(10,110);
                            tft_writeString("2: Change keypad password");
    
                            tft_setCursor(10,140);
                            tft_writeString("3: Change pot range");
    
                            tft_setCursor(10,170);
                            tft_writeString("4: Change knock pattern");
                        
                        }
                    }
                    else if (changeKeyState){ //user is inputting new key code
                        tft_fillScreen(ILI9340_BLACK);
                        tft_setCursor(120, 160);
                        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(5);
                        sprintf(buf,"%d",digit);
                        tft_writeString(buf);

                        codeBuf[codeCounter] = digit;

                        if (codeCounter < 4){
                            codeCounter++;
                        }
                        if (codeCounter == 4){ //user has input 4 digit key code
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            tft_writeString("You entered:"); //display what the user input
                            
                            tft_setCursor(40,40);                        
                            sprintf(buf,"%d",codeBuf[0]);
                            tft_writeString(buf);
                        
                            tft_setCursor(55,40);
                            sprintf(buf,"%d",codeBuf[1]);
                            tft_writeString(buf);
                       
                            tft_setCursor(70,40);
                            sprintf(buf,"%d",codeBuf[2]);
                            tft_writeString(buf);
                        
                            tft_setCursor(85,40);
                            sprintf(buf,"%d",codeBuf[3]);
                            tft_writeString(buf);
                            
                            //ask if user wants to keep the new code or not
                            tft_setCursor(10,60);
                            tft_writeString("Keep this code?");
                                
                            tft_setCursor(10,90);
                            tft_writeString("1: Yes");
                        
                            tft_setCursor(10,115);
                            tft_writeString("2: Retry");
                        
                            tft_setCursor(10,140);
                            tft_writeString("3: Cancel. Keep previous");
                            //go to confirm key state
                            confirmKeyState = 1;
                            changeKeyState =0;
                            codeCounter =0;                            
                        }
                    }
                    else if (confirmKeyState){
                        if (digit ==1){ //yes, keep code, go back to configure state

                            firstNumber = codeBuf[0];
                            secondNumber = codeBuf[1];
                            thirdNumber = codeBuf[2];
                            fourthNumber = codeBuf[3];
                            
                            keyCodeChanged = 1;
                            confirmKeyState = 0;
                            configureState = 1;
                            
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            tft_writeString("BOX IS UNLOCKED");
    
                            tft_setCursor(10,50);
                            tft_writeString("Choose an option:");
    
                            tft_setTextSize(1);
                            tft_setCursor(10,80);
                            tft_writeString("1: Lock box");
    
                            tft_setCursor(10,110);
                            tft_writeString("2: Change keypad password");
    
                            tft_setCursor(10,140);
                            tft_writeString("3: Change pot range");
    
                            tft_setCursor(10,170);
                            tft_writeString("4: Change knock pattern");
                        }
                        else if (digit==2){ //retry
                            confirmKeyState = 0;
                            changeKeyState = 1;
                            codeCounter = 0;
                            configureState = 0;
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
                            tft_writeString("Enter New 4-Digit Code");
                        }
                        else if (digit == 3){ //cancel, go back to configure state
                            confirmKeyState = 0;
                            changeKeyState = 0;
                            codeCounter = 0;
                            configureState = 1;
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            tft_writeString("BOX IS UNLOCKED");
    
                            tft_setCursor(10,50);
                            tft_writeString("Choose an option:");
    
                            tft_setTextSize(1);
                            tft_setCursor(10,80);
                            tft_writeString("1: Lock box");
    
                            tft_setCursor(10,110);
                            tft_writeString("2: Change keypad password");
    
                            tft_setCursor(10,140);
                            tft_writeString("3: Change pot range");
    
                            tft_setCursor(10,170);
                            tft_writeString("4: Change knock pattern");
                        }
                    }
                    else if (changePiezoState){
                        tft_fillScreen(ILI9340_BLACK);
                        tft_setCursor(120, 160);
                        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(5);
                        sprintf(buf,"%d",digit);
                        tft_writeString(buf);

                        codeBuf[codeCounter] = digit;
                        potBuf[codeCounter]=digit;
                        
                        if (codeCounter < 3){
                            codeCounter++;
                        }
                        if (codeCounter ==3){
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            tft_writeString("You entered:"); //display what the user input

                            tft_setCursor(40,40);                        
                            sprintf(buf,"%d",codeBuf[0]);
                            tft_writeString(buf);

                            tft_setCursor(55,40);
                            sprintf(buf,"%d",codeBuf[1]);
                            tft_writeString(buf);

                            tft_setCursor(70,40);
                            sprintf(buf,"%d",codeBuf[2]);
                            tft_writeString(buf);

                            if (changePiezoLeft){
                                tft_setCursor(10,100);
                                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                                tft_writeString("Enter Center");
                                tft_setCursor(10,120);
                                tft_writeString("Pot Value");
                                changePiezoLeft = 0;
                                changePiezoCenter =1;
                                changePiezoRight = 0;
                                codeCounter =0;
                                leftPiezoValue = potBuf[0]*100+potBuf[1]*10+potBuf[2];
                                
                                if (leftPiezoValue < 130){
                                    leftMinV2temp  = 100;
                                }
                                else{
                                    leftMinV2temp = leftPiezoValue - 30;
                                }
                                if (leftPiezoValue > 970){
                                    leftMaxV2temp = 999;
                                }
                                else{
                                    leftMaxV2temp = leftPiezoValue +30;
                                }
                            }
                            else if (changePiezoCenter){
                                tft_setCursor(10,100);
                                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                                tft_writeString("Enter Right");
                                tft_setCursor(10,120);
                                tft_writeString("Pot Value");
                                
                                changePiezoLeft = 0;
                                changePiezoCenter =0;
                                changePiezoRight = 1;
                                codeCounter =0;
                                centerPiezoValue = potBuf[0]*100+potBuf[1]*10+potBuf[2];
                                if (centerPiezoValue < 130){
                                    centerMinV2temp  = 100;
                                }
                                else{
                                    centerMinV2temp = centerPiezoValue - 30;
                                }
                                if (centerPiezoValue > 970){
                                    centerMaxV2temp = 999;
                                }
                                else{
                                    centerMaxV2temp = centerPiezoValue +30;
                                }
                            }
                        }
                        if (changePiezoRight && codeCounter == 3){

                            rightPiezoValue = potBuf[0]*100+potBuf[1]*10+potBuf[2];
                            if (rightPiezoValue < 130){
                                  rightMinV2temp  = 100;
                              }
                            else{
                                rightMinV2temp = rightPiezoValue - 30;
                            }
                            if (rightPiezoValue > 970){
                                rightMaxV2temp = 999;
                            }
                            else{
                                rightMaxV2temp = rightPiezoValue +30;
                            }
                            char potBuf[3];
                            potBuf[0]=leftPiezoValue;
                            potBuf[1]=centerPiezoValue;
                            potBuf[2]=rightPiezoValue;

                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            tft_writeString("You entered:"); //display what the user input

                            tft_setCursor(40,40);                        
                            sprintf(buf,"Left Pot: %d",leftPiezoValue);
                            tft_writeString(buf);

                            tft_setCursor(40,60);
                            sprintf(buf,"Center Pot: %d",centerPiezoValue);
                            tft_writeString(buf);

                            tft_setCursor(40,80);
                            sprintf(buf,"Right Pot: %d",rightPiezoValue);
                            tft_writeString(buf);

                            tft_setCursor(10,110);
                            tft_writeString("Your Ranges:");
                            
                            tft_setCursor(10,135);
                            sprintf(buf,"Left Pot: %d-%d", leftMinV2temp, leftMaxV2temp);//leftPiezoValue -50, leftPiezoValue + 50);
                            tft_writeString(buf);
                            
                            tft_setCursor(10,155);
                            sprintf(buf,"Center Pot: %d-%d", centerMinV2temp, centerMaxV2temp);//centerPiezoValue -50, centerPiezoValue + 50);
                            tft_writeString(buf);
                            
                            tft_setCursor(10,175);
                            sprintf(buf,"Right Pot: %d-%d", rightMinV2temp, rightMaxV2temp);//rightPiezoValue -50, rightPiezoValue + 50);
                            tft_writeString(buf);
                            
                            tft_setTextSize(1);
                            tft_setCursor(10,220);
                            tft_writeString("Keep these Pot Values?");

                            tft_setCursor(10,245);
                            tft_writeString("1: Yes");

                            tft_setCursor(10,270);
                            tft_writeString("2: Retry");

                            tft_setCursor(10,295);
                            tft_writeString("3: Cancel. Keep previous");
                            
                            changePiezoLeft = 0;
                            changePiezoCenter =0;
                            changePiezoRight = 0;
                            codeCounter=0;
                            changePiezoState=0;
                            confirmPiezoState = 1;
                        }
                    }
                    else if (confirmPiezoState){
                        if (digit ==1){ //yes, keep pot values, go back to configure state
                            
                            potCodeChanged = 1;
                            
                            leftMinV2 = leftMinV2temp;//leftPiezoValue -50;
                            leftMaxV2 = leftMaxV2temp;//leftPiezoValue +50;
                            centerMinV2 = centerMinV2temp;//centerPiezoValue - 50;
                            centerMaxV2 = centerMaxV2temp;//centerPiezoValue + 50;
                            rightMinV2 = rightMinV2temp;//rightPiezoValue - 50;
                            rightMaxV2 = rightMaxV2temp;//rightPiezoValue + 50;
                            diffTimeChanged =1;
                            confirmPiezoState = 0;
                            configureState = 1;
                            codeCounter=0;
                            
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            tft_writeString("BOX IS UNLOCKED");
    
                            tft_setCursor(10,50);
                            tft_writeString("Choose an option:");
    
                            tft_setTextSize(1);
                            tft_setCursor(10,80);
                            tft_writeString("1: Lock box");
    
                            tft_setCursor(10,110);
                            tft_writeString("2: Change keypad password");
    
                            tft_setCursor(10,140);
                            tft_writeString("3: Change pot range");
    
                            tft_setCursor(10,170);
                            tft_writeString("4: Change knock pattern");
                        }
                        else if (digit==2){ //retry
                            confirmPiezoState = 0;
                            codeCounter = 0;
                            changePiezoState = 1;
                            configureState =0;
                            changePiezoLeft = 1;
                            changePiezoCenter= 0;
                            changePiezoRight = 0;
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
                            tft_writeString("Enter Left Pot Value");
                        }
                        else if (digit == 3){ //cancel, go back to configure state
                            confirmPiezoState = 0;
                            codeCounter = 0;
                            changePiezoState = 0;
                            configureState =1;
                            changePiezoLeft = 0;
                            changePiezoCenter= 0;
                            changePiezoRight = 0;
                            tft_fillScreen(ILI9340_BLACK);
                            tft_setCursor(10, 10);
                            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                            tft_writeString("BOX IS UNLOCKED");
    
                            tft_setCursor(10,50);
                            tft_writeString("Choose an option:");
    
                            tft_setTextSize(1);
                            tft_setCursor(10,80);
                            tft_writeString("1: Lock box");
    
                            tft_setCursor(10,110);
                            tft_writeString("2: Change keypad password");
    
                            tft_setCursor(10,140);
                            tft_writeString("3: Change pot range");
    
                            tft_setCursor(10,170);
                            tft_writeString("4: Change knock pattern");
                        }
                    } 
                }
            }
           else 
           {
               PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
               PushState=NoPush;
           }
           break;
        case 2: //Pushed
           if (keycode==possible)
           {
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
                PushState=Pushed; 
           }
           else 
           {
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
                PushState=MaybeNoPush;
           }
           break;
        case 3: //MaybeNoPush
           if (keycode==possible) 
           {
                PushState=Pushed;
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
           }
           else 
           {
               PushState=NoPush;
               PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
           }
           break;
        } 
    }
    PT_END(pt);
}
static PT_THREAD (protothread_cmd2(struct pt *pt))
{
    PT_BEGIN(pt);
        static char cmd[16];
        static int pinValue;
        static char pinBuf[60];
        tft_fillScreen(ILI9340_BLACK);
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); 
        tft_setTextSize(2);
        tft_writeString("Place finger on");
        tft_setCursor(0,30);
        tft_writeString("fingerprint scanner");
      while(1) {
        mPORTBSetPinsDigitalIn(BIT_11);    //Set port as input3
        pinValue=mPORTBReadBits(BIT_11);
        if (pinValue ==2048){
            tft_fillScreen(ILI9340_BLACK);
            tft_setCursor(0, 30);
            tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
            tft_setCursor(0,30);
            tft_writeString("Fingerprint Accepted");
            tft_setCursor(0,70);
            tft_writeString("Box Unlocked");
            PT_YIELD_TIME_msec(1000);
            fingerprintUnlocked = 1;
        }
        PT_YIELD_TIME_msec(100);
      } // END WHILE(1)
  PT_END(pt);
} //UART Thread

// === Main  ======================================================
void main(void) {
  SYSTEMConfigPerformance(PBCLK);
  
  ANSELA = 0; ANSELB = 0; CM1CON = 0; CM2CON = 0;

  // === config threads ==========
  // turns OFF UART support and debugger pin
  PT_setup();
  // The PBDIV value is already set via the pragma FPBDIV option above..
	SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

  // set up the Vref pin and use as a DAC
  // enable module| eanble output | use low range output | use internal reference | desired step
  //CVREFOpen( CVREF_ENABLE | CVREF_OUTPUT_ENABLE | CVREF_RANGE_LOW | CVREF_SOURCE_AVDD | CVREF_STEP_0 );
  // And read back setup from CVRCON for speed later
  // 0x8060 is enabled with output enabled, Vdd ref, and 0-0.6(Vdd) range
  //CVRCON_setup = CVRCON; //CVRCON = 0x8060 from Tahmid http://tahmidmc.blogspot.com/

  // timer interrupt //////////////////////////
  // Set up timer2 on,  interrupts, internal clock, prescalar 1, toggle rate
  // at 30 MHz PB clock 60 counts is two microsec
  // 400 is 100 ksamples/sec
  // 2000 is 20 ksamp/sec
  OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, 2500); //for Fs = 16e3

  // set up the timer interrupt with a priority of 2
  ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
  mT2ClearIntFlag(); // and clear the interrupt flag

  // SCK2 is pin 26 
  // SDO2 (MOSI) is in PPS output group 2, could be connected to RB5 which is pin 14
  PPSOutput(2, RPB5, SDO2);

  // control CS for DAC
  mPORTBSetPinsDigitalOut(BIT_3);
  mPORTBSetBits(BIT_3);
  
  // divide Fpb by 2, configure the I/O ports. Not using SS in this example
  // 16 bit transfer CKP=1 CKE=1
  // possibles SPI_OPEN_CKP_HIGH;   SPI_OPEN_SMP_END;  SPI_OPEN_CKE_REV
  // For any given peripherial, you will need to match these
  
  //SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);
      
  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();
  // the ADC ///////////////////////////////////////
  // configure and enable the ADC
	CloseADC10();	// ensure the ADC is off before setting the configuration
	// define setup parameters for OpenADC10
	// Turn module on | ouput in integer | trigger mode auto | enable autosample
  // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
  // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
  // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
  #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON//| ADC_AUTO_SAMPLING_OFF //
	// define setup parameters for OpenADC10
	// ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_1 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
  
	// Define setup parameters for OpenADC10
  // use peripherial bus clock | set sample time | set ADC clock divider
  // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
  // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
  #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_5 | ADC_CONV_CLK_Tcy2 //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

	// define setup parameters for OpenADC10
	// set AN4 and  as analog inputs
	#define PARAM4_9	ENABLE_AN9_ANA // pin 26
  #define PARAM4_11 ENABLE_AN11_ANA  //pin 24

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_ALL

	// use ground as neg ref for A | use AN4 for input A     
	// configure to sample AN9 
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN9 ); // configure to sample AN4 
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4_9, PARAM5 ); // configure ADC using the parameters defined above
    
	EnableADC10(); // Enable the ADC
  ///////////////////////////////////////////////////////
  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_color);
  PT_INIT(&pt_anim);
  PT_INIT(&pt_scanKey);
  PT_INIT(&pt_keypad);
  PT_INIT(&pt_adc);
  PT_INIT(&pt_keypadConfigureState);
  PT_INIT(&pt_piezo);
  PT_INIT(&pt_potUnlocked);
  PT_INIT(&pt_printConfigScreen);
  PT_INIT(&pt_changePiezo);
  PT_INIT(&pt_cmd2);
  PT_INIT(&pt_piezoUnlocked);
  // init the display
  tft_init_hw();
  tft_begin();
  tft_fillScreen(ILI9340_BLACK);
  //240x320 vertical display
  tft_setRotation(0); // Use tft_setRotation(1) for 320x240

  // seed random color
  srand(1);
  
  // build the sine lookup table
   // scaled to produce values between 0 and 4096
   int i;
   for (i = 0; i < sine_table_size; i++){
         sin_table[i] = (int)(1023*sin((float)i*6.283/(float)sine_table_size));
    }
    while (1){
        while (boxLocked){ 
            while (!keypadUnlocked){
                PT_SCHEDULE(protothread_keypad(&pt_keypad));
            }
            while (keypadUnlocked && !potUnlocked){
                PT_SCHEDULE(protothread_adc(&pt_adc));
                num_knocks=0;
            }
            while (keypadUnlocked && potUnlocked && !piezoUnlocked){
                PT_SCHEDULE(protothread_piezo(&pt_piezo));
            }
            while (keypadUnlocked && potUnlocked && piezoUnlocked &&!fingerprintUnlocked){
                PT_SCHEDULE(protothread_cmd2(&pt_cmd2));
            }
            boxLocked = 0; //box is no longer locked
            configureState = 1; //go to configure state
            configRun=1;
        }
        while (!boxLocked){
            while (configureState){
                PT_SCHEDULE(protothread_keypadConfigureState(&pt_keypadConfigureState));
            }
            while (!configureState && (changeKeyState||confirmKeyState || changePiezoState || confirmPiezoState)){
                PT_SCHEDULE(protothread_keypadConfigureState(&pt_keypadConfigureState));
            }
            piezoChanged =1;
            num_knocks=0;
        }
      }
  }// main

// === end  ======================================================
