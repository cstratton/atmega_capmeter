/**********************************************************/
/*                                                        */
/* Open re-implementation of firmware for the             */
/* atmega-based capacitance meter kit sold by             */
/* JYETECH, Sparkfun, etc                                 */
/*                                                        */
/* Based on published schematic and oscilloscope          */
/* probing but NOT on any attempt to readout or           */
/* decompile existing firmware.  Please note the          */
/* the basic algorithm is well known, and published       */
/* as an idea on the Arduino site.                        */                    
/*                                                        */
/* Copyright (C) 2011 Chris Stratton                      */
/*                                                        */
/* This program is free software; you can redistribute it */
/* and/or modify it under the terms of the GNU General    */
/* Public License as published by the Free Software       */
/* Foundation; either version 2 of the License, or        */
/* (at your option) any later version.                    */
/*                                                        */
/* This program is distributed in the hope that it will   */
/* be useful, but WITHOUT ANY WARRANTY; without even the  */
/* implied warranty of MERCHANTABILITY or FITNESS FOR A   */
/* PARTICULAR PURPOSE.  See the GNU General Public        */
/* License for more details.                              */
/*                                                        */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* You should have received a copy of the GNU General     */
/* Public License along with this program; if not, write  */
/* to the Free Software Foundation, Inc.,                 */
/* 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA */
/*                                                        */
/* Licence can be viewed at                               */
/* http://www.fsf.org/licenses/gpl.txt                    */
/*                                                        */
/* Installation:                                          */
/*                                                        */
/* The idea is that you pop the supplied ATMEGA48         */
/* out of its socket and replace it with an ATMEGA168     */
/* that has been programmed with an arduino-style         */
/* serial bootloader (taking into account 12 MHz xtal!)   */
/* and then use that to upload this replacement firmware. */
/* The only board modification is to install a serial     */
/* header and rig up the capacitor or resistor to         */
/* DTR or RTS so avrdude can assert the atmega reset,     */
/* though this is only necessary if using the bootloader  */
/* is desired for further development/experimentation.    */
/*                                                        */
/* The ISP header was avoided as it shares pins used      */
/* in the sensitive measurement                           */
/*                                                        */   
/* WARNING: This is EXPERIMENTAL software.  Measurement   */
/* accuracy may not match the supplied firmware. Please   */
/* do not reflash the supplied microcontroller with this  */
/* program, but instead store it in a safe place so that  */
/* you can switch back if desired                         */
/*                                                        */
/*   TODO: find some 1% components check calibration      */ 
/*                                                        */  
/*   TODO: verify full discharge                          */ 
/*                                                        */    
/*   TODO: audit scaling math for 32-bit overflow         */
/*                                                        */
/**********************************************************/

//T=RC Calibration measurements
//upper cal voltage = 4.22v
//supply =5.14v
//lower cal voltage = 0.86v
//Vt = Vin * (1-e^(-t/T))
//1 - Vt / vin1 = e^(-t/T)
//-t/T = ln (1-(vt/vin))
//so it's 1.72 time constants
//Farads = count / F_CPU / 1.72 / R
//pF = count * 1000000 / (12 * 1.72 * 3300) 
//pF = count / 12 / 1.72 / 3300
//with the extra /1000 if we are on range 0

#define NOISY 0
#define F_CPU 12000000L
#define BAUD_RATE 115200L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//segments are dp, center, then counterclockwise from upper left
#define LETTER_P 0b01110011
#define LETTER_N 0b01010100
#define LETTER_U 0b00011100
#define LETTER_C 0b00111001
#define LETTER_E 0b01111001
#define DECIMAL_POINT 0b10000000 
#define MINUS_SIGN 0b01000000; 
uint8_t digit_to_segments[] = {
  0b00111111, //0
  0b00000110, //1
  0b01011011, //2
  0b01001111, //3
  0b01100110, //4
  0b01101101, //5
  0b01111101, //6
  0b00000111, //7
  0b01111111, //8
  0b01101111, //9
};

void hexbyte(uint8_t val);

static uint8_t current_digit;
uint8_t displayval[4];

void delay_ms(uint16_t ms) {
  uint16_t j;
  while (ms--) 
    for (j=0; j<1550; j++) //approximate at 12 MHz
      asm("nop");
}

void serialbyte(char b) {
  while (!((UCSR0A) & (1<<UDRE0)));
  UDR0 = b;
}

void serialstr(char *s) {
  while (*s) serialbyte(*(s++));
}

//drive one digit of the display, called from interrupt
//this is for COMMON CATHODE
void display(char digit, char segments) {
  PORTD &= 0b11000011;
  PORTB &= 0b11110001;
  PORTC &= 0b11100000; //one segment and the digit selects
  PORTD |= (segments & 0xf) << 2;
  PORTB |= ((segments & 0xe0) >> 4);
  PORTC |= (segments & 0x10); 
  PORTC |= (~digit & 0xf);
}

void setup()
{
  //init serial - must use double speed if we have 12MHz xtal
  UCSR0A = (1<<U2X0); //Double speed mode USART0 //42
  UBRR0L = (uint8_t)(F_CPU/(BAUD_RATE*8L)-1); 
  UBRR0H = (F_CPU/(BAUD_RATE*8L)-1) >> 8;
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);          //0x98
  UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
 
/* Idea copied from arduino bootloader - Enable internal pull-up 
resistor on pin D0 (RX), in order to supress line noise */
  DDRD &= ~_BV(PIND0);
  PORTD |= _BV(PIND0);

  PORTC = 1;
  DDRD = 0b00111100 ; 
  DDRC = 0x1f; //digit selects and segment 4
  DDRB = 0b00001110 ; //highest three bits

  //Set up an interrupt to multiplex the display
  TCCR0A = 2;//<<WGM01; //0b00000010;	                    
  TCCR0B = 0b00000101; //clk / 1024, ie ~12 KHz
  OCR0A = 10; //1.2 khz?
  TIMSK0 = _BV(OCIE0A);
  sei();
}

ISR(TIMER0_COMPA_vect) {
  sei(); //no need to have them disabled
  current_digit = (current_digit + 1) & 3;
  display(1<<current_digit, displayval[current_digit]);
}

//serial print a byte in hex
void hexbyte(uint8_t val) {
  char len=2;
  while(len--) {
    char v = val >> 4;
    if (v > 9) v=(v + 'a'- 10);
    else v=(v + '0');
    serialbyte(v);
    val = val << 4;
  }
}

//serial print a decimal value relative to a power of ten
void printdec(uint32_t val, uint32_t place) {
  uint32_t dd;
  int16_t d;  
  while (place) {
    dd = val / place;
    d = dd;
    serialbyte(d+'0');
    val -= (place * dd);
    place = place / 10;
  }
}

//serial print 32 bits decimal formatted 0 000.000 000
void printeng32(uint32_t val) {
  uint32_t place = 1000000000L; 
  uint32_t dd;
  int d;  
  while (place) {
    dd = val / place;
    d = dd;
    serialbyte(d+'0');
    val -= (place * dd);
    if (place == 1000000000L) serialbyte(' ');
    if (place == 1000000) serialbyte('.');
    if (place == 1000) serialbyte(' ');
    place = place / 10;
  }
}


//Take a measurement given resistor range and preselector value
//returns the unscaled timer count, or 65535 on overflow

uint16_t measure(uint8_t range, uint8_t presel) {
  uint16_t count, temp;
  uint8_t charge_mask;
  // range 0 is the 3.3M resistor which we do not control
  // range 1 is the 3.3K resistor
  // range 2 would be the 330 ohm resistor but it's not installed
  //  hexbyte(range);
  //  hexbyte(presel);
  //  PRR &= ~ _BV(PRTIM1);  //power it up  
  if (range==1) charge_mask = 1<<4; //PB1 is the 3.3K resistor
  else if (range == 2) charge_mask = 1<<0; //PB0 is the 330 ohm resistor NOT INSTALLED!
  else charge_mask = 0;
  DDRB &= ~(1 << 0) ;//high Z PB0
  DDRB &= ~(1 << 4) ;//high Z PB4
  PORTB &= ~(1 << 0);
  PORTB &= ~(1 << 4);

  DDRB |= (1<<5) ; //make PB5 an output
  PORTB &= ~(1<<5); //make PB5 low to set threshold to 1/3 VCC

  DDRD |= 1<<6; //make the 120 ohm sense resistor an output
  PORTD &= ~(1<<6); //drive it low to discharge the cap

//readback via the 3.3K resistor and wait for the full discharge
  while (PINB & (1<<4)); 

  delay_ms(10); //just to be safe (is this enough?)

  TCCR1B = 0b11000000;   //Select noise cancel, rising edge, count disabled
  TCNT1H=0;              //reset count (probably not needed?)
  TCNT1L=0;
  ACSR = _BV(ACIC);      //choose the Analog compator as timer capture source
  TIFR1 |=  _BV(ICF1) | _BV(TOV1); //clear capture & oveflow flags
  if (NOISY) serialstr("TIFR1=");
  if (NOISY) hexbyte(TIFR1);

  DDRD &= ~(1<<6); //make the AIN0 an input (it's already low, so no pullup);
  
  cli(); //critical section as we kick it off, so not interrupts
  //Drive an appropriate charge resistor
  DDRB |= charge_mask; 
  PORTB |= charge_mask;

  TCCR1B |= presel; //set preselector value to enable counting
#if 0 //optionally wait until we pass the lower threshold to start measuring
  while (!(TIFR1 & _BV(ICF1))) {  //wait for capture event
    if (TIFR1 & _BV(TOV1)) {
      if (NOISY) serialstr("OVERFLOW!\n");
      return 65535; //OVERFLOW!!
    }
  }
  //Be careful - if there's no cap and we can get a race condition here
  //with clearing the flag and changing the threshold, and as a result
  //we can never realize we hit the threshold instantly
  //we should probably manually check the comparator output to be
  //sure we aren't waiting for something that already happened

  PORTB |= (1<<5); //switc to the 2/3 VCC threshold
  TIFR1 |=  _BV(ICF1) | _BV(TOV1); //clear capture & oveflow flags
#endif
  PORTB |= (1<<5); //switc AIN1 to the 2/3 VCC threshold
  sei(); //resume display interrupt
  while (!(TIFR1 & _BV(ICF1))) {  //wait for capture event
    if (TIFR1 & _BV(TOV1)) {
      if (NOISY) serialstr("OVERFLOW!\n");
      return 65535; //OVERFLOW!!
    }
  }

  TCCR1B &= ~7; //counting disabled
  if (NOISY) serialstr("TIFR1=");
  if (NOISY) hexbyte(TIFR1);
  if (NOISY) serialstr(" ACSR=");
  if (NOISY) hexbyte(ACSR);
  if (NOISY) serialstr(" ");

//Reading the 16-bit value in the Input Capture Register (ICR1) is done by first reading the low
//byte (ICR1L) and then the high byte (ICR1H).

  temp = ICR1L;
  count = ICR1H << 8;
  count += temp;
  if (NOISY) printdec(count, 10000);
  //  serialstr("c=");
  //  printdec(i, 10000);
  //  serialbyte(10);
  if (NOISY) serialbyte(10);
  return count;
}

int main(void)
{
  uint32_t calval;
  uint16_t i;
  uint32_t timebase, mcount, acc; 
  uint16_t range, presel;
  calval=0;
  DDRB=0;
  DDRD=0;
  PORTD=0;
  PORTB=0;
  setup();
  displayval[0] = digit_to_segments[0]; //dummy data
  displayval[1] = digit_to_segments[1]; 
  displayval[2] = digit_to_segments[2]; 
  displayval[3] = digit_to_segments[3]; 
  while (1) {
    //check serial for commands
    if (UCSR0A & _BV(RXC0)) {
      char cmd = UDR0;
      if (cmd == 'L') {
	serialstr("measure lower threshold voltage on pin 13");
	DDRB |= 1<<5;
	PORTB &= ~(1<<5);
	while (!(UCSR0A & _BV(RXC0)));
      }
      if (cmd == 'U') {
	serialstr("measure upper threshold voltage on pin 13");
	DDRB |= 1<<5;
	PORTB |= (1<<5);
	while (!(UCSR0A & _BV(RXC0)));
      }
      else serialstr("unkown command\n");
    }
  component_changed: 
    //Ranges: 2 330  ohm resistor controlled by PB0/***NOT INSTALLED****
    //        1 3.3K ohm resistor controlled by PB4
    //        0 3.3M ohm resistor always on
    range = 2; //will pre-decremewnt and start at range 1 (range 2 not installed)
    timebase = 0;
    if (1) {
      do {
	range--;
	//	serialstr("range=");
	//	printdec(range, 10);
	//  Find a preselector that doesn't overflow the 16-bit counter
	for (presel = 1; (presel <= 4) && ((mcount = measure(range, presel))==65535); presel++);
	//	serialstr(" count=");
	//	printdec(mcount, 1000000000);
	//	serialstr("\n");
      }
      //if the count is less than 65535/1000 decrease the resistor
      while (range && (mcount < 6000)); 
    }
    else 
      for (presel = 1; (presel <= 4) && ((mcount = measure(range=1, presel))==65535); presel++);

    //    serialstr("timebase = ");
    //    printdec(timebase, 100);
    //    serialstr(" count= ");
    //    printdec(mcount, 10000);
    //    serialstr(" multiplier=");
    //    printdec(timebase, 10000);
    acc = mcount;
#if 1 //average 10 readings
    uint16_t trials=10;
    if (presel==1) trials = 40;
    if (presel==2) trials = 20;
    for (i=1; i < trials; i++) {
      mcount= measure(range, presel);
      if (mcount == 65535) {
	displayval[3]=displayval[2]=displayval[1]=displayval[0] = MINUS_SIGN;
	goto component_changed;
      }
      else acc+= mcount;
    }
    timebase = 1 << (3*(presel - 1)); //divisor = 8 to the (presel-1)
    acc = acc * timebase;
    acc = acc/trials;
#else
    timebase = 1 << (3*(timebase - 1));
    acc = acc * timebase;
#endif
    serialstr("clocks=");
    printeng32(acc);
    serialstr(range ? " R=3.3K" : " R=3.3M");
    //the math says we should multiply counts by 14.6817 to get to pF
    //    acc = acc * 1468L;
    acc = acc * 1450L; //to make my 2400pf+/-2% cap read correct
    acc = acc / 10L;

    if (range == 0) acc = acc /1000; //3.3M resistor instead of 3.3K
    //TODO add scaling for the 330 ohm capacitor if it's installed
    
    //Crude volatile cal mechanism - record reading when the button is down
    if (!(PINC & (1<<5)))  
      calval = acc;
    /* 
       serialstr("\nvalue=");
       printeng32(acc);
       serialstr("cal=");
       printeng32(calval);
    */

    //at this point we have a value in tenths of pF
    serialstr(" value=");
    printeng32(acc);
    acc -= calval; 
    serialstr("\n");    

    //Subtracting a calibration means that we can produce a negative number
    if (acc > 4000000000L) { //FIXME unsigned warning
      displayval[3] = MINUS_SIGN;
      displayval[2]=displayval[1]= digit_to_segments[0];
      displayval[0]=LETTER_E;
    }
    else {
      uint32_t m=1000;

      //Now let's try to scale it and decimal point it
      //find the largest multiplier (>= 100pf) smaller than the value
      for (i=2; i<11; i++) {
	if (acc > m) m=m*10;
	else break;
      }
      m = m / 1000;  //move the decimal point to get some fractional digits
      acc += m/2 ;   //add .5 before truncating in order to round
      acc = acc / m; 
      uint16_t digit;
      /*
	serialstr("m=");printeng32(m);
	serialstr("acc=");printeng32(acc);
	serialstr("digit=");hexbyte(digit);
      */

      //Nobody has 800/900 nF caps, so for 
      //values close to 1, display 0.8xx instead of 8.xx
      if (acc >= 800) {
	acc=acc/10;
	i=i+1;
      }

      //assign the three digits
      //(display multiplexing in timer0 interrupt)
      digit = acc / 100;
      displayval[3] = digit_to_segments[digit];
      acc -= digit * 100;
    
      digit = acc / 10;
      displayval[2] = digit_to_segments[digit];
      acc -= digit * 10;
    
      digit = acc;
      displayval[1] = digit_to_segments[digit];
            
      //Assign the decimal point and the units
      if (i < 4) {
	displayval[0] = LETTER_P;
	displayval[4-i] |= DECIMAL_POINT;
      } else if (i < 7) { 
	displayval[0] = LETTER_N;
	displayval[7-i] |= DECIMAL_POINT;
      } else if (i < 10) {
	displayval[0] = LETTER_U;
	displayval[10-i] |= DECIMAL_POINT;
      } else
	displayval[0] = LETTER_E;  //ERROR, we don't handle them that big
    }
  }  
  return 0; //as if!
}
