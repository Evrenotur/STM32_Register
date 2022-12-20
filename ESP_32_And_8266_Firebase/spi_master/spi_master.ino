#include<SPI.h>                  
#define push 2
#define LED 4           

int x;
int value;

void setup (void)

{
  Serial.begin(115200);               
  
  pinMode(push,INPUT);              
  pinMode(LED,OUTPUT);                  
  
  SPI.begin();                            
  SPI.setClockDivider(SPI_CLOCK_DIV8); 
  digitalWrite(SS,HIGH);                  
}

void loop(void)
{
  byte m_send,m_receive;          

for(int i=0;i<100;i++)
{
  x=i;

}
    delay(100);

 
  digitalWrite(SS, LOW);                  
  
  m_send = x;                            
  m_receive=SPI.transfer(m_send); 
  
 
}
