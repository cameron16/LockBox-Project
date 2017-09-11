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

// string buffer
char buffer[12];
static char buf[12];
static char key_buf[12];
static char freq_buf[12];
char cleared_buf[12];
char restart_buf[12];
char digit_buf[12];
char isr_buf[12];

volatile unsigned int vi, vj, packed, DAC_value2; // voice variables
//volatile unsigned /*?*/int CVRCON_setup; // stores the voltage ref config register after it is set up

static int keycode, keypad, i, restart_key, pound_key, curr_key, test_mode_key;
static int possible = -1;
static int digit = -1;
static int pattern, PushState;
static int buf_count = 0;
volatile unsigned int isr_counter = 0;
static int dual_tone = 0; //conditional variable to play DT freq's
static int start_voice, voice = 0; //conditional variable to play voice
//static int keypad, i, pattern;
//static int keytable[12]={0x108, 0x81, 0x101, 0x201, 0x82, 0x102, 0x202, 0x84, 0x104, 0x204, 0x88, 0x208};

// === thread structures ============================================
// thread control structs
// note that UART input and output are threads
static struct pt pt_timer, pt_color, pt_anim, pt_scanKey,pt_keypad,pt_testModes,pt_setFreq, pt_setVoice;

// system 1 second interval tick
int sys_time_seconds ;
//== Timer 2 interrupt handler ===========================================
volatile unsigned int DAC_data ;// output value
volatile SpiChannel spiChn = SPI_CHANNEL2 ;	// the SPI channel to use
volatile int spiClkDiv = 2 ; // 20 MHz max speed for this DAC
// the DDS units:
volatile unsigned int freq1, freq2;
static int idx1, idx2; //voice indicies
volatile unsigned int phase_accum1, phase_incr1;//[60];
volatile unsigned int phase_accum2, phase_incr2;//[60];
volatile unsigned int sine1_idx, sine2_idx;
volatile int sin_table_data;
int buffer_ready = 0;
int phase_count = 0;
int test_mode = 0;

int j = 0;

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
   
    phase_accum1 += phase_incr1;//[j];//[i];//[phase_count];
    phase_accum2 += phase_incr2;//[j];//[i];//[phase_count];
    sine1_idx = phase_accum1>>24;
    sine2_idx = phase_accum2>>24;
//    tft_fillRoundRect(30,200, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//    tft_setCursor(30, 200);
//                    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//                    sprintf(digit_buf,"%d",isr_counter);
//                    tft_writeString(digit_buf);
    sin_table_data = sin_table[phase_accum1>>24] + sin_table[phase_accum2>>24];
    if(dual_tone) {
//        tft_fillRoundRect(10,200, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(10, 200);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(isr_buf,"%d", idx1);
//        tft_writeString(isr_buf);

        //DAC_data = sin_table[phase_accum1>>24] + sin_table[phase_accum2>>24] + 2048;
        //DAC_data = sin_table_data;
        if(isr_counter < 2*five_ms)
        {
            DAC_data = (((255*isr_counter)/(2*five_ms))*(sin_table_data))>>8 + 2047; //converts 2's C to offset binary
        } 
        else if (isr_counter >= 2*five_ms && isr_counter < (2*five_ms+sixtyfive_ms))
        {
            DAC_data = sin_table_data+2047; //converts 2's C to offset binary
        }
        else if (isr_counter >= (2*five_ms+sixtyfive_ms) && isr_counter < (4*five_ms+sixtyfive_ms))
        {
            if(test_mode)
            {
                DAC_data = sin_table_data;
            }
            else
            {
                DAC_data = ((255*(4*five_ms+sixtyfive_ms-isr_counter))/(2*five_ms))*(sin_table_data)>>8+2047; //converts 2's C to offset binary
            }
        }
        else
        {
            if(!test_mode){
                dual_tone = 0;
                isr_counter=0;
                DAC_data=0;
            }
            else
            {
                DAC_data = sin_table_data+2047;
            }
            
        }
        isr_counter++;
    }

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
     if(voice)
     {
         if(start_voice)
         {
//             tft_fillRoundRect(10,200, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(10, 200);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(isr_buf,"%d", idx1);
//        tft_writeString(isr_buf);
//         tft_fillRoundRect(10,250, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(10, 250);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(isr_buf,"%d", idx2);
//        tft_writeString(isr_buf);
            vi = idx1;
            start_voice=0;
         }
        vj = vi>>1;
        if (~(vi & 1)) packed = AllDigits[vj] ;
        if (vi & 1) DAC_value2 = packed>>4 ; // upper 4 bits
        else  DAC_value2 = packed & 0x0f ; // lower 4 bits
        DAC_value2 = (DAC_value2 << 8);
        //CVRCON = CVRCON_setup | DAC_value2 ;
        vi++ ;
        if (vi/*vj*/>idx2) {
            voice=0;
            vi = 0;
            
        }
        
     }
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
     tft_setCursor(0, 0);
     tft_setTextColor(ILI9340_WHITE);  tft_setTextSize(1);
     tft_writeString("Time in seconds since boot\n");
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(1000) ;
        sys_time_seconds++ ;
        
        // draw sys_time
        tft_fillRoundRect(0,10, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 10);
        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
        sprintf(buffer,"%d", sys_time_seconds);
        tft_writeString(buffer);
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // timer thread

// === Color Thread =================================================
// draw 3 color patches for R,G,B from a random number
//static int color ;
//static int i;
static PT_THREAD (protothread_color(struct pt *pt))
{
    PT_BEGIN(pt);
      static int color ;
      static int i;
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(2000) ;

        // choose a random color
        color = rand() & 0xffff ;
       
        // draw color string
        tft_fillRoundRect(0,50, 150, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(0, 50);
        tft_setTextColor(ILI9340_WHITE); tft_setTextSize(1);
        sprintf(buffer," %04x  %04x  %04x  %04x", color & 0x1f, color & 0x7e0, color & 0xf800, color);
        tft_writeString(buffer);

        // draw the actual color patches
        tft_fillRoundRect(5,70, 30, 30, 1, color & 0x1f);// x,y,w,h,radius,blues
        tft_fillRoundRect(40,70, 30, 30, 1, color & 0x7e0);// x,y,w,h,radius,greens
        tft_fillRoundRect(75,70, 30, 30, 1, color & 0xf800);// x,y,w,h,radius,reds
        // now draw the RGB mixed color
        tft_fillRoundRect(110,70, 30, 30, 1, color);// x,y,w,h,radius,mix color
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // color thread

// === Animation Thread =============================================
// move a disk
static PT_THREAD (protothread_anim(struct pt *pt))
{
    PT_BEGIN(pt);
    static int xc=10, yc=150, vxc=2, vyc=0;
      while(1) {
        // yield time 1 second
        PT_YIELD_TIME_msec(32);

        // erase disk
         tft_fillCircle(xc, yc, 4, ILI9340_BLACK); //x, y, radius, color
        // compute new position
         xc = xc + vxc;
         if (xc<5 || xc>235) vxc = -vxc;         
         //  draw disk
         tft_fillCircle(xc, yc, 4, ILI9340_GREEN); //x, y, radius, color
        // NEVER exit while
      } // END WHILE(1)
  PT_END(pt);
} // animation thread

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
        
        // yield time
        //PT_YIELD_TIME_msec(30);
   
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
                if (keytable[i]==keypad){
                    
                    break;
                    
                }
            }
        }
        else i = -1; // no button pushed
        keycode = i;
        
        if(keycode==11)
        {
            pound_key = 1;
        }
        
        if(keycode==10)
        {
            restart_key = 1;
        }
        
       
        // draw key number
//        tft_fillRoundRect(30,200, 100, 14, 1, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(30, 200);
//        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//        sprintf(buffer,"%d", i);
//        if (i==10)sprintf(buffer,"*");
//        if (i==11)sprintf(buffer,"#");
//        tft_writeString(buffer);
        

        // NEVER exit while
      
  PT_END(pt);
} // keypad thread

static PT_THREAD (protothread_setFreq(struct pt *pt))
{
    //PT_BEGIN(pt);
    switch(curr_key) {
        case 0: 
            freq1 = 941;
            freq2 = 1336;
            break;
        case 1: 
            freq1 = 697;
            freq2 = 1209;
            break;
        case 2: 
            freq1 = 697;
            freq2 = 1336;
            break;
        case 3:
            freq1 = 697;
            freq2 = 1477;
            break;
        case 4: 
            freq1 = 770;
            freq2 = 1209;
            break;
        case 5:
            freq1 = 770;
            freq2 = 1336;
            break;
        case 6: 
            freq1 = 770;
            freq2 = 1477;
            break;
        case 7:
            freq1 = 852;
            freq2 = 1209;
            break;
        case 8: 
            freq1 = 852;
            freq2 = 1336;
            break;
        case 9:  
            freq1 = 852;
            freq2 = 1477;
            break;
        case 10:
            freq1 = 941;
            freq2 = 1209;
            break;
        case 11: 
            freq1 = 941;
            freq2 = 1477;
            break;
        default:
            freq1 = 0;
            freq2 = 0;
            break;
    //PT_END();      
    }
}
    
    static PT_THREAD (protothread_setVoice(struct pt *pt))
    {
    //PT_BEGIN(pt);
    switch(digit) {
        case 0: 
            idx1 = 410*4;
            idx2 = 1929*4;
            break;
        case 1: 
            idx1 = 2980*4;
            idx2 = 3838*4;
            break;
        case 2: 
            idx1 = 5005*4;
            idx2 = 6040*4;
            break;
        case 3:
            idx1 = 7475*4;
            idx2 = 8640*4;
            break;
        case 4: 
            idx1 = 9785*4;
            idx2 = 10930*4;
            break;
        case 5:
            idx1 = 12195*4;
            idx2 = 13870*4;
            break;
        case 6: 
            idx1 = 14645*4;
            idx2 = 16265*4;
            break;
        case 7:
            idx1 = 17060*4;
            idx2 = 18935*4;
            break;
        case 8: 
            idx1 = 19755*4;
            idx2 = 20740*4;
            break;
        case 9:  
            idx1 = 21515*4;
            idx2 = 23375*4;
            break;
        default:
            idx1 = 0;
            idx2 = 0;
            break;
            
    }
    //PT_END(pt); 
    }
  

static PT_THREAD (protothread_testModes(struct pt *pt)) //SCAN KEYPAD
{
    PT_BEGIN(pt);
    while(1){
        PT_YIELD_TIME_msec(30);
    mPORTBSetPinsDigitalIn(BIT_10);  //* key - test mode key
    //mPORTBSetPinsDigitalOut(BIT_13);
    mPORTBClearBits(BIT_10);    
    //mPORTBSetBits(1);
    test_mode_key = mPORTBReadBits(BIT_10);
    
        while(test_mode_key==0) //active low
        {   
            PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
            curr_key = keycode;
            tft_fillRoundRect(30,150, 200, 10, 2, ILI9340_BLACK);// x,y,w,h,radius,color
             tft_setCursor(30, 150);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
                sprintf(buf,"%d",curr_key);
                tft_writeString(buf);
            switch(curr_key){
            case 1:
                freq1=697;
                freq2=0;
                break;
            case 2:
                freq1=770;
                freq2=0;
                break;
            case 3:
                freq1=852;
                freq2=0;
                break;
            case 4:
                freq1=941;
                freq2=0;
                break;
            case 5:
                freq1=1209;
                freq2=0;
                break;
            case 6:
                freq1=1336;
                freq2=0;
                break;
            case 7:
                freq1=1477;
                freq2=0;
                break;    
            default:
                freq1=0;
                freq2 = 0;
            }
            //PT_SPAWN(pt,&pt_setFreq,protothread_setFreq(&pt_setFreq));
            phase_incr1=freq1*two32/Fs;
            phase_incr2=freq2*two32/Fs;
            dual_tone = 1;   
            test_mode_key = mPORTBReadBits(BIT_10);
            test_mode=1;
        }
//                tft_fillRoundRect(5,5, 200, 10, 2, ILI9340_BLACK);// x,y,w,h,radius,color
//        tft_setCursor(5, 5);
//                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
//                sprintf(buf,"%d",buf_count);
//                tft_writeString(buf);
                test_mode=0;
                //phase_incr1 = 0;
                test_mode_key = mPORTBReadBits(BIT_10);
           // dual_tone = 0;
 
    }
            PT_END(pt);
}

static PT_THREAD (protothread_keypad(struct pt *pt)) //THE THREAD W/ STUFF
{
    PT_BEGIN(pt);
    int res;
    while(1) {
        //========================scan the keypad=============================
        PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
        //==============run the debounce state machine=========================
        PT_YIELD_TIME_msec(30);
        //tft_fillScreen(ILI9340_BLACK);
        tft_fillRoundRect(5,5, 200, 10, 2, ILI9340_BLACK);// x,y,w,h,radius,color
        tft_setCursor(5, 5);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
                sprintf(buf,"%d",digit);
                tft_writeString(buf);
        switch (PushState) {   
        case 0: //NoPush
           if (keycode==-1) {
               //tft_fillRoundRect(30,100, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_fillRoundRect(30,50, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_setCursor(30, 50);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                tft_writeString("nopush");
               PushState=NoPush;
               PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
           }
           else 
           {
               //tft_fillRoundRect(30,100, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_fillRoundRect(30,50, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_setCursor(30, 50);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                tft_writeString("nopush(else)");
               
               PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
               PushState=MaybePush;
               possible = keycode;
           }
           break;
        case 1: //MaybePush
           if (keycode==possible) {
               tft_fillRoundRect(30,50, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_setCursor(30, 50);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                tft_writeString("maybe");
              PushState=Pushed;  
              digit=possible;
              char buf3[2];
              //add digit to buffer
              if(digit < 10 && digit > -1) {
                  
//                    tft_fillRoundRect(30,150, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
//                    tft_setCursor(30, 150);
//                    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//                    sprintf(buf3,"%d",digit);
//                    tft_writeString(buf3);
                  if(buf_count<12) {
                    key_buf[buf_count] = digit;
                    PT_SPAWN(pt,&pt_setVoice,protothread_setVoice(&pt_setVoice));
                    start_voice = 1;
                    //tft_fillRoundRect(30,200, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
//                    tft_setCursor(30, 200);
//                    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//                    sprintf(buf3,"%d",idx1);
//                    tft_writeString(buf3);
                    //tft_fillRoundRect(30,250, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
//                    tft_setCursor(30, 250);
//                    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//                    sprintf(buf3,"%d",idx2);
//                    tft_writeString(buf3);
                    buf_count++;
                    voice = 1;  
                  }
                }
              //voice = 0;
              
              if(restart_key==1)
              {
                  tft_fillRoundRect(30,100, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
                  tft_fillRoundRect(30,300, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
                    tft_setCursor(30, 300);
                    tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                    //sprintf(buf3,"%d",key_buf[buf_count]);
                    tft_writeString("in restart");
                    //PT_YIELD_TIME_msec(500);
                    tft_fillRoundRect(30,300, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
                  //key_buf = 0;
                    //memset(key_buf,0,sizeof(key_buf));           
                    buf_count = 0;
                    //digit=0;
                    //digit = -1;
                    restart_key=0;
                    voice = 0;
                    
              }
              
              if(pound_key) //if playback key pressed
                {
                  char buf2[2];
                  //tft_fillRoundRect(30,100, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
                  //j=0;
                  j=0;   
//                  tft_fillRoundRect(30,150, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
//                    tft_setCursor(30, 150);
//                        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
//                        sprintf(buf3,"%d", buf_count);
//                        tft_writeString(buf3);
                    //loop through buffer to each digit
                  
                    while(j<buf_count)
                    {    
//                        tft_fillRoundRect(30,250, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
//                        tft_setCursor(30, 250);
//                        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//                        sprintf(buf3,"j = %d",j);
//                        tft_writeString(buf3);
                        tft_setCursor(30+20*j, 100);
                        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
                        sprintf(buf2,"%d", key_buf[j]);
                        tft_writeString(buf2);
                        curr_key = key_buf[j];
                        PT_SPAWN(pt,&pt_setFreq,protothread_setFreq(&pt_setFreq));
                        phase_incr1=freq1*two32/Fs;
                        phase_incr2=freq2*two32/Fs;
//                        tft_fillRoundRect(30,150, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
//                        tft_setCursor(30, 150);
//                        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//                        sprintf(buf3,"phincr1 = %d",phase_incr1);
//                        tft_writeString(buf3);
//                        tft_fillRoundRect(30,200, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
//                        tft_setCursor(30, 200);
//                        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
//                        sprintf(buf3,"phincr2 = %d",phase_incr2);
//                        tft_writeString(buf3);
                        dual_tone = 1;
                        PT_YIELD_TIME_msec(500);
                        phase_incr1=0;
                        phase_incr2 = 0;
                        j++;
                    }
                    //buffer_ready = 1;
                    //start voice
                }
              
              
//              tft_fillRoundRect(30,150, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
//              tft_setCursor(30, 150);
//                        tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(1);
//                        sprintf(buf3,"%d", restart_key);
//                        tft_writeString(buf3);
//              [more code to record a digit]
//              [or start a tone, or other event]
           }
           else 
           {
               //tft_fillRoundRect(30,100, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_fillRoundRect(30,50, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_setCursor(30, 50);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                tft_writeString("maybe (else)");
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
               PushState=NoPush;
               
           }
           break;
        case 2: //Pushed
           if (keycode==possible)
           {
               //tft_fillRoundRect(30,100, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_fillRoundRect(30,50, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_setCursor(30, 50);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                tft_writeString("pushed");
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
               PushState=Pushed; 
               
           }
           else 
           {
               //tft_fillRoundRect(30,100, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_fillRoundRect(30,50, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_setCursor(30, 50);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                tft_writeString("pushed (else)");
                PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
               PushState=MaybeNoPush;
               
           }
           break;
        case 3: //MaybeNoPush
           if (keycode==possible) 
           {
               //tft_fillRoundRect(30,100, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_fillRoundRect(30,50, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_setCursor(30, 50);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                tft_writeString("maybe no push");
               PushState=Pushed;
               PT_SPAWN(pt,&pt_scanKey,protothread_scanKey(&pt_scanKey));
           }
           else 
           {
               //tft_fillRoundRect(30,100, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_fillRoundRect(30,50, 400, 14, 2, ILI9340_BLACK);// x,y,w,h,radius,color
               tft_setCursor(30, 50);
                tft_setTextColor(ILI9340_YELLOW); tft_setTextSize(2);
                tft_writeString("maybe no push (else)");
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
        SpiChnOpen(spiChn, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , spiClkDiv);
  
  // === setup system wide interrupts  ========
  INTEnableSystemMultiVectoredInt();

  // init the threads
  PT_INIT(&pt_timer);
  PT_INIT(&pt_color);
  PT_INIT(&pt_anim);
  PT_INIT(&pt_scanKey);
  PT_INIT(&pt_keypad);
  PT_INIT(&pt_testModes);

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

  // round-robin scheduler for threads
  while (1){
      //PT_SCHEDULE(protothread_timer(&pt_timer));
      //PT_SCHEDULE(protothread_color(&pt_color));
      //(protothread_anim(&pt_anim));
      PT_SCHEDULE(protothread_keypad(&pt_keypad));
      PT_SCHEDULE(protothread_testModes(&pt_testModes));
      }
  }// main

// === end  ======================================================
