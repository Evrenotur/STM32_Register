#include <Wire.h>
int LED = 13;
int x = 0;

void setup() 
{
  pinMode (LED, OUTPUT);
  Wire.begin(9); 
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
}

void receiveEvent(int bytes) 
{
  x = Wire.read();
}

void loop() 
{
  Serial.println(x);
}
