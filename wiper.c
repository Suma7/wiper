#include<avr/io.h>
#include<avr/interrupt.h>
#include<stdint.h>
#define adcpin 0
struct{
  		volatile unsigned int FLAG:1;
}FLAG_BIT;
uint16_t value;
int main()
{
  
  DDRD|=((1<<PD6));
  DDRD|=((1<<PD7));   //Motor and LED
  DDRD&=~(1<<PB0);             //Switch
  PORTD&=~(1<<PB0);
  PORTD&=~(1<<PD6);            //Oscilloscope
  PCICR |= (1 << PCIE0);    // set PCIE0 to enable PCMSK0 scan
  PCMSK0 |= (1 << PCINT0);  // set PCINT8 to trigger an interrupt on state change

  sei();
  while(1)
  {
    while(FLAG_BIT.FLAG==1)
    {
      PORTD|=(1<<PD7);
      ADCSRA|=(1<<ADEN);
	  value=adc_read(adcpin);
      Serial.println(value);
      if(value<=50)
      {
        OCR0B=0;   //Wiper off,motor speed is 0,rain intensity is 0
      }
      else if(value>50 && value<=100)
       {
        OCR0B=32;  //DC motor at very low rpm,rain intensity is very low(Mist mode)
      } 
      else if(value>100 && value<=250)
      {
        OCR0B=64;  //DC motor at low rpm,rain intensity is low(Low mode)
      }
      else if(value>250 && value<=750)
      {
        OCR0B=128;   //DC motor at medium rpm,rain intensity is medium(Intermittent mode)
      }
      else //if(value>750)
      {
        OCR0B=192;  //DC motor at high rpm,rain intensity is high(High mode)
      }
    }
    PORTD&=~((1<<PD6)|(1<<PD7));
  }
}

uint16_t adc_read(uint8_t adc)
{
  ADMUX |= (1<<REFS0);
  ADMUX|=(adc & 0x0f);
  ADCSRA|=(1<<ADSC);
    while(ADCSRA &(1<<ADSC))
  {
  }
  return ADC;
}

ISR(PCINT0_vect)
{
  cli();
  if(FLAG_BIT.FLAG==1)
  {
    FLAG_BIT.FLAG=0;
  	TCCR0B=0x00;
  }
  else 
  {
    FLAG_BIT.FLAG=1;
    TCNT0=0x00;
  	TCCR0A|=0x00;
    TCCR0B|=((1<<CS02)|(1<<CS00));
    TIMSK0|=((1<<OCIE0A)|(1<<OCIE0B));
    OCR0A=255;
  }
}

ISR(TIMER0_COMPA_vect)
{
  PORTD|=(1<<PD6);
}

ISR(TIMER0_COMPB_vect)
{
  PORTD&=~(1<<PD6);
}





