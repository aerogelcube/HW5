/*
 * File:   main.c
 * Author: Sandra Fang
 *
 * April 15, 2015, 4:30 PM
 *
 * Description: OLED display to print accelerometer directions
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include<xc.h> // processor SFR definitions
#include<sys/attribs.h> // __ISR macro
#include "i2c_master_int.h" //use "" to search for the proper directory
#include "i2c_display.h"
#include "accel.h"

// DEVCFGs here
// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins - ?
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // slowest wdt - PS1048576?
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 40MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz; 8MHz/2=4Mhz
#pragma config FPLLMUL =  MUL_20 // multiply clock after FPLLIDIV: multiply 4MHz*20 = 80MHz
#pragma config UPLLIDIV =  DIV_2 // divide clock after FPLLMUL: 80Mhz/2 = 40Mhz
#pragma config UPLLEN = ON // USB clock on
#pragma config FPLLODIV = DIV_2 // divide clock by 2 to output on pin

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid; preset to 0
#pragma config PMDL1WAY = ON // not multiple reconfiguration, check this
#pragma config IOL1WAY = ON // not multimple reconfiguration, check this
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // controlled by USB module

void printOLED(int ascii, int startposx, int startposy);

int main() {
    // Startup code to run as fast as possible and get pins back from bad defaults
    __builtin_disable_interrupts();
    // set the CP0 CONFIG register to indicate that
    // kseg0 is cacheable (0x3) or uncacheable (0x2)
    // see Chapter 2 "CPU for Devices with M4K Core"
    // of the PIC32 reference manual
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);
    // no cache on this chip!
    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;
    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;
    // disable JTAG to be able to use TDI, TDO, TCK, TMS as digital
    DDPCONbits.JTAGEN = 0;
    //accelerometer setup
    acc_setup();
    __builtin_enable_interrupts();
//-------------------------------------------------------------
    int delay = 0;
    while(delay<10000) { //for button debouncing
        delay++;
    }
    // set up B4 digital output for powering OLED screen
    TRISBbits.TRISB4 = 0; //digital output
    LATBbits.LATB4 = 1; //output on=0 initially

    delay = 0;
    while(delay<4000) {
        delay++;
    }

    //ACCELEROMETER
    short accels[3]; // accelerations for the 3 axes
    short mags[3]; // magnetometer readings for the 3 axes
    short temp;
    
    display_init();

    while(1) {

        // read the accelerometer from all three axes
        // the accelerometer and the pic32 are both little endian by default
        //(the lowest address has the LSB)
        // the accelerations are 16-bit twos compliment numbers, the same as a short
        acc_read_register(OUT_X_L_A, (unsigned char *) accels, 6);
        // need to read all 6 bytes in one transaction to get an update.
        acc_read_register(OUT_X_L_M, (unsigned char *) mags, 6);
        // read the temperature data. Its a right justified 12 bit two's compliment number
        acc_read_register(TEMP_OUT_L, (unsigned char *) &temp, 2);

        /*
        char message[25];
        sprintf(message, "%3.2f,%3.2f,%3.2f", accels[0]/16000.0,accels[1]/16000.0,accels[2]/16000.0);
        int i=0;
        while(message[i])
        {
            printOLED(message[i],28,32+5*i);
            i++;
        }
         */
        display_clear();
        int startrow = 32;
        int startcol = 64;
        int d;
        //x direction
        for (d = 0; d<1+abs(20*accels[0]/16000.0); d++) {
            if (accels[0]>0){
                display_pixel_set(startrow,startcol-d,1);
                display_pixel_set(startrow+1,startcol-d,1);
            }
            else {
                display_pixel_set(startrow,startcol+d,1);
                display_pixel_set(startrow+1,startcol+d,1);
            }
        }

        //y direction
        for (d = 0; d<1+abs(20*accels[1]/16000.0); d++) {
            if (accels[1]>0){
                display_pixel_set(startrow-d,startcol,1);
                display_pixel_set(startrow-d,startcol+1,1);
            }
            else {
                display_pixel_set(startrow+d,startcol,1);
                display_pixel_set(startrow+d,startcol+1,1);
            }
        }
        display_draw();
    }
    
    
    return 0;
}

void printOLED(int ascii, int startrow, int startcol) {
    int row, col;
    int ascii_num = ascii-0x20;
    for (col = 0; col < 5; col++){
        for (row = 0; row < 8; row++) {
            int totrow = startrow+row;
            int totcol = startcol+col;
            if(totrow<HEIGHT && totcol<WIDTH) { //
                int digit = !!((1 << row) & ASCII[ascii_num][col]); //1000 1000 = 136
                display_pixel_set(totrow,totcol,digit);
            }
         }
    }
}