#ifndef UBOT_H
#define UBOT_H

#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/eeprom.h>
//#include <stdlib.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <inttypes.h>
#include "lcd.h"
typedef unsigned char UCHAR;
#define IsOff(PIN, BIT)    ((PIN & (1<<BIT)) == 0)
#define IsOn(PIN, BIT)     ((PIN & (1<<BIT)) != 0)
#define SetOn(PORT, BIT)     PORT |= (1<<BIT)  //Give Pos
#define SetOff(PORT, BIT)    PORT &= ~(1<<BIT) //Give Ground




//sensor pins digital inputs
#define SENSORPORT PORTC
#define SENSORPIN PINC
#define SENSORDDR DDRC
#define S1 PC0
#define S2 PC1
#define S3 PC2
#define S4 PC3
#define S5 PC4
#define S6 PC5
#define IsSensor(S) (ReadADC(S)<lconfig.Threshold[S])

#define KEYBOARDPORT PORTD
#define KEYBOARDPIN PIND
#define KEYBOARDDDR DDRD
#define K1 PD1
#define K2 PD2
#define K3 PD3
#define K4 PD4
#define KEYPRESS(KEY) (IsOff(KEYBOARDPIN,KEY))
#define NOKEYPRESS (   (  !KEYPRESS(K1) && !KEYPRESS(K2) && !KEYPRESS(K3) && !KEYPRESS(K4) )     )


//motor pins out
#define MRPORT PORTB
#define MRDDR DDRB
#define MLPORT PORTB
#define MLDDR DDRB

#define MR1 PB3
#define MR2 PB0
#define ML1 PB5
#define ML2 PB4




//motor enables
#define MEPORT PORTB
#define MEDDR DDRB
#define MLE PB1
#define MRE PB2

//eeprom variables
typedef uint8_t PIDType;
typedef uint8_t SensorType;

typedef struct env_var{
PIDType P_K;
PIDType D_K;
PIDType I_K;
UCHAR basespeed;
UCHAR maxspeed;
SensorType Threshold[6];
} EnvVar;

EnvVar lconfig;
EnvVar EEMEM rconfig;//stores in EEPROME

UCHAR sensor[6];//holds the current value of sensors True or False
char last='r';
float prev_error=0, I=0;
void load_eeprom_info(void)
{
    eeprom_read_block((void*) &lconfig, (void*) &rconfig, sizeof(lconfig));
}
void write_eeprom_info(void)
{
   eeprom_write_block((void*) &lconfig, (void*) &rconfig, sizeof(rconfig));
}






void init_pwm(void);
void init_motors(void);
void init_sensors(void);
void init_leds(void);
void ml_fw(UCHAR speed);
void ml_rv(UCHAR speed);
void mr_fw(UCHAR speed);
void mr_rv(UCHAR speed);
void set_led(UCHAR num);



void init_pwm(void)
{
        SetOn(MEDDR,MLE);
        SetOn(MEDDR,MRE);
        ///SetOn(MEPORT,MLE);
        //SetOn(MEPORT,MRE);
        TCCR1A=TCCR1B = 0;
        TCCR1A |= (1<<WGM10);
        TCCR1B |= (1<<CS10);
        TCCR1A |= (1<<COM1A1);
        TCCR1A |= (1<<COM1B1);
}
void init_motors(void)
{
        SetOn(MLDDR,ML1);
        SetOn(MLDDR,ML2);
        SetOn(MRDDR,MR1);
        SetOn(MRDDR,MR2);
}
        
void init_sensors(void)
{
        SENSORPORT=0xFF;
}
void ml_fw(UCHAR speed)
{
        SetOff(MLPORT,ML1);
        SetOn(MLPORT,ML2);
        OCR1B=speed;
}
void mr_fw(UCHAR speed)
{
        SetOff(MRPORT,MR1);
        SetOn(MRPORT,MR2);
        OCR1A=speed;
} 
void ml_rv(UCHAR speed)
{
        SetOn(MLPORT,ML1);
        SetOff(MLPORT,ML2);
        OCR1B=speed;
}
void mr_rv(UCHAR speed)
{
        SetOn(MRPORT,MR1);
        SetOff(MRPORT,MR2);
        OCR1A=speed;
}

void init_adc(void)
{
    ADMUX=(1<<REFS0);                         // For Aref=AVcc;
    ADCSRA=(1<<ADEN)|(1<<ADPS1)|(1<<ADPS0); //Rrescalar div factor =8
}


void init_lcd(void)
{
    //init lcd library
    lcd_init(LCD_DISP_ON_CURSOR);

}

#define flash_lcd(STRING,DELAY) lcd_clrscr();lcd_puts(STRING);_delay_ms(DELAY);lcd_clrscr();
#define write_till_lcd(STRING,DELAY) lcd_clrscr();lcd_puts(STRING);_delay_ms(DELAY);
char BUFF[16];

void welcome_message(void)
{
    flash_lcd("     uBot",1000);
    flash_lcd("   Micro",500);
    flash_lcd("   Scale",500);
    flash_lcd("   Bot",500);
}


char* itoa(int value, char* result, int base) {
    // check that the base if valid
    if (base < 2 || base > 36) {
        *result = '\0';
        return result;
    }

    char* ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

uint8_t ReadADC(uint8_t ch)
{
    //Select ADC Channel ch must be 0-7
    ch&=0b00000111;
    SetOff(ADMUX,MUX0);SetOff(ADMUX,MUX1);SetOff(ADMUX,MUX2);
    ADMUX|=ch;
    ADMUX|=(1<<ADLAR);//Left Shift

    
    ADCSRA|=(1<<ADSC);//Start Single conversion
    while (!(ADCSRA & (1<<ADIF)));//Wait for conversion to complete
    ADCSRA|=(1<<ADIF);//Clear ADIF by writing one to it
    //starting single conversion again for better resolution (bsdfox's advice)
    ADCSRA|=(1<<ADSC);//Start Single conversion
    while (!(ADCSRA & (1<<ADIF)));//Wait for conversion to complete
    ADCSRA|=(1<<ADIF);//Clear ADIF by writing one to it


    return(ADCH);
}

void getsensor(void)
{
        UCHAR i;
        for(i=0;i<6;i++) //read the value of all but the extreame sensors
        {
                sensor[i]=IsSensor(i);
        }
        if(sensor[0]) last='l';
        else if(sensor[5]) last='r';

        
}
float calc_error(void)
{
        getsensor();
        float error=0;
        UCHAR i,sum=0;
        for(i=0;i<6;i++)
        {
                error+=i*sensor[i];
                sum+=sensor[i];
        }
        
        error/=sum;
        error-=2.5;
        return error;
}
float calc_correction(void)
{
        float error=calc_error();
        float P=error * lconfig.P_K;
        I+=error;
        I*=lconfig.I_K; 
        float D=error-prev_error;
        D*=lconfig.D_K;
        prev_error=error;
        return (P+I+D);
}         
        
#endif
