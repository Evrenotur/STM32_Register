#include <Wire.h>
int x = 0;

void setup() 
{
  Wire.begin(); 
  Serial.begin(9600);
}

void loop() 
{
  Wire.beginTransmission(9);
  Wire.write(x);              
  Wire.endTransmission();
 

    x = 10;
  
}
