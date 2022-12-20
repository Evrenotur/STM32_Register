#include<SPI.h>
#define inputbutton 2
#define outputLED 4

volatile boolean received;
volatile byte Slavereceived,Slavesend;
int buttonvalue;
int x;

void setup()

{
  Serial.begin(115200);
  
  pinMode(inputbutton,INPUT); 
  pinMode(outputLED,OUTPUT);               
  pinMode(MISO,OUTPUT);                  

  SPCR |= _BV(SPE);                  
  received = false;

  SPI.attachInterrupt();                  
  
}

ISR (SPI_STC_vect)                        
{
  Slavereceived = SPDR;         
  received = true;                        
}

void loop()
{  
 

      
      Serial.println(Slavereceived);
      
      
 // Slavesend=x;                             
  //SPDR = Slavesend;                       
  delay(1000);
}
