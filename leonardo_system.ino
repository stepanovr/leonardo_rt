/*
//const byte ledPin = 13;
const byte  ledPin = LED_BUILTIN;
const byte interruptPin = 2;
volatile byte state = LOW;
int toggle0 = 0;
*/

#include <Wire.h>
 
#define  SLAVE_ADDRESS           0x2A  //slave address,any number from 
                                       // 0x01 to 0x7F

//timer interrupts
//by Amanda Ghassaei
//June 2012
//https://www.instructables.com/id/Arduino-Timer-Interrupts/

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
*/

//timer setup for timer0, timer1, and timer2.
//For arduino uno or any board with ATMEL 328/168.. diecimila, duemilanove, lilypad, nano, mini...

//this code will enable all three arduino timer interrupts.
//timer0 will interrupt at 2kHz
//timer1 will interrupt at 1Hz
//timer2 will interrupt at 8kHz

#define CHANNELS_NUM 4
#define CTRL_REGS_NUM 10

typedef struct _measure_t{
  unsigned short result[CHANNELS_NUM];
  unsigned char receivedCommands[CTRL_REGS_NUM];
  unsigned char pos;
}measure_t;

measure_t a2d;

//storage variables
boolean toggle0 = 0;
boolean toggle1 = 0;
boolean toggle2 = 0;

const int mainsMonitorRefPin = 0;
static volatile unsigned char currentAdcChannel;
unsigned int adcVal;

volatile int RtCnt = 0;

void setup(){
  
  //set pins as outputs
//  pinMode(8, OUTPUT);
//  pinMode(9, OUTPUT);
  pinMode(13, OUTPUT);

//I2C slave setup
   Wire.begin(SLAVE_ADDRESS); 
   Wire.onRequest(requestEvent);
   Wire.onReceive(receiveEvent);


  a2d.pos = 0;
cli();//stop interrupts
 // Set up the ADC. We need to read the mains voltage frequently in order to get a sufficiently accurate RMS reading.
 // To avoid using too much CPU time, we use the conversion-complete interrupt. This means we can't use analogRead(),
 // we have to access the ADC ports directly. 
 currentAdcChannel = mainsMonitorRefPin;      // set up which analog input we will read first
 ADMUX = B01000000 | currentAdcChannel;
 ADCSRB = B00000000; // Analog Input bank 1
 ADCSRA = B10011111; // ADC enable, manual trigger mode, ADC interrupt enable, prescaler = 128


//set timer0 interrupt at 2kHz
  TCCR0A = 0;// set entire TCCR2A register to 0
  TCCR0B = 0;// same for TCCR2B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 2khz increments
  OCR0A = 124*2;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);   
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
//  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
OCR1A = 1562;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

/*
//set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
*/
  Serial.begin(115200);
sei();//allow interrupts
 // Kick off a new ADC conversion. We already set the multiplexer to the correct channel when the last conversion finished.
 ADCSRA = B11001111;   // ADC enable, ADC start, manual trigger mode, ADC interrupt enable, prescaler = 128

}//end setup


ISR(TIMER0_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
//generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
  static int cnt = 10;
  // Kick off a new ADC conversion. We already set the multiplexer to the correct channel when the last conversion finished.
//  ADCSRA = B11001111;   // ADC enable, ADC start, manual trigger mode, ADC interrupt enable, prescaler = 128
  if(--cnt){
    return;
  }else {
    cnt = 10;
  }
  RtCnt++;
  /*
  if (toggle0){
    digitalWrite(13,HIGH);
    toggle0 = 0;
  }
  else{
    digitalWrite(13,LOW);
    toggle0 = 1;
  }
  */
}


ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle1){
    digitalWrite(8,HIGH);
    toggle1 = 0;
  }
  else{
    digitalWrite(8,LOW);
    toggle1 = 1;
  }
}

/*
ISR(TIMER2_COMPA_vect){//timer1 interrupt 8kHz toggles pin 9
//generates pulse wave of frequency 8kHz/2 = 4kHz (takes two cycles for full wave- toggle high then toggle low)
  if (toggle2){
    digitalWrite(9,HIGH);
    toggle2 = 0;
  }
  else{
    digitalWrite(9,LOW);
    toggle2 = 1;
  }
}
*/

// Interrupt service routine for ADC conversion complete
ISR(ADC_vect) 
{
 // The mcu requires us to read the ADC data in the order (low byte, high byte)
 unsigned char adcl = ADCL;
 unsigned char adch = ADCH;
 unsigned char index;
 //adcVal = (adch << 8) | adcl;

 a2d.result[a2d.pos] = (adch << 8) | adcl;
 a2d.pos++;
 if(a2d.pos >= CHANNELS_NUM){
  a2d.pos = 0;
 }
 
 switch(a2d.pos)
 {
   default:
   case 0:
     index = 0;
     break;

   case 1:
     index = 1;
     break;

   case 2:
     index = 4;
     break;

   case 3:
     index = 5;
     break;


   // add switch cases fort the other channels you want to read here...
 }
 
 // Set the ADC multiplexer to the channel we want to read next. Setting if here rather than in the tick ISR allows the input
 // to settle, which is required for ADC input sources with >10k resistance.
 ADMUX = (B01000000 | index);   // Vcc reference, select current channel
 // Kick off a new ADC conversion. We already set the multiplexer to the correct channel when the last conversion finished.
 ADCSRA = B11001111;   // ADC enable, ADC start, manual trigger mode, ADC interrupt enable, prescaler = 128
}

//I2C slave support
void requestEvent()
{
     Wire.write((char*)&a2d.result[0], sizeof(unsigned short)*CHANNELS_NUM);  //Set the buffer up to send all sizeof(unsigned short)xCHANNELS_NUM bytes of data
}

void receiveEvent(int bytesReceived)
{
     for (int a = 0; a < bytesReceived; a++)
     {
          if ( a < sizeof(unsigned short)*CHANNELS_NUM)
          {
               a2d.receivedCommands[a] = Wire.read();
          }
          else
          {
               Wire.read();  // if we receive more data then allowed just throw it away
          }
     }
}


#define SANTISEC_PER_SEC  100
#define SEC_PER_MIN  60
#define MIN_PER_HOUR  60

//#define SEC_PER_MIN  5
//#define MIN_PER_HOUR  5

boolean toggle_led = 0;
  static char seconds = 0;
  static char minutes = 0;
  static int hours = 0;

void each_sec()
{
char i;

for(i = 0; i < CHANNELS_NUM; i++){
  Serial.print(a2d.result[i]);
  Serial.print(" === ");
}
Serial.print((int)hours);
Serial.print(":");
Serial.print((int)minutes);
Serial.print(":");
Serial.println((int)seconds);
}

void each_min()
{

Serial.println("-----Minute----");
}

void each_hour()
{
Serial.println("-----Hour----");
}


void get_input()
{
  char incomingByte;
  if (Serial.available() > 0) {
    incomingByte = Serial.read(); // read the incoming byte:
    Serial.print(" I received:");
    Serial.println(incomingByte); 
  }
}

void blinkLED()
{
  static char counter = 10;
  static char toggle = 0;
  if(0 == --counter){
    counter = 5;
    if (toggle0){
      digitalWrite(13,HIGH);
      toggle0 = 0;
    }
    else{
      digitalWrite(13,LOW);
      toggle0 = 1;
    }
  }
  
}

void each_10_ms()
{
  get_input();
  blinkLED();
}


void loop(){
  //do other things here
  static char santisec = 0;


  while(0 == RtCnt){
  }

  RtCnt--;
  santisec++;
//    Serial.println((int)santisec);
//    Serial.println((int)seconds);
  each_10_ms();
  
  if(SANTISEC_PER_SEC == santisec){

      santisec = 0;

      seconds++;
            
      if(SEC_PER_MIN == seconds){
        seconds = 0;
        minutes++;

        if(MIN_PER_HOUR == minutes){
          minutes = 0;
          hours++;
          each_hour();
        }
        each_min();
      }
      each_sec();
    }
  
}
