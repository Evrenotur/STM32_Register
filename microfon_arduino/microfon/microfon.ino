/*
  modified on Sep 8, 2020
  Modified by MohammedDamirchi from Arduino Examples
  Home
*/

#define led1   3
#define led2   4
#define led3   5
#define led4   6
#define button 7

int sen1,sen2,sen3,sen4,art;
int value1,value2,value3,value4;



void setup() {
 
  Serial.begin(9600);

  pinMode(led1,OUTPUT);
  pinMode(led2,OUTPUT);
  pinMode(led3,OUTPUT);
  pinMode(led4,OUTPUT);
  pinMode(button,INPUT);
}


void loop() {
if (!button) {
      delay(50); 
       art++;
      while(!button){ 
      }
      delay(10); 
      } 
      
 if(art==1)
 {
    digitalWrite(led1,HIGH);
    delay(500);
    digitalWrite(led1,LOW);
    delay(500);
  value1=analogRead(A4); 
  Serial.println(value1);
 }
 else if(art==2)
 {
   digitalWrite(led2,HIGH);
    delay(500);
    digitalWrite(led2,LOW);
    delay(500);
   value2=analogRead(A4);
     Serial.println(value2);
 }
 else if(art==3)
 {
   digitalWrite(led3,HIGH);
    delay(500);
    digitalWrite(led3,LOW);
    delay(500);
    value3=analogRead(A4);
      Serial.println(value3);
 }
 else if(art==4)
 {
   digitalWrite(led4,HIGH);
    delay(500);
    digitalWrite(led4,LOW);
    delay(500);
   value4=analogRead(A4);
     Serial.println(value4);
 }
 
  else 
 {
   art=0;
 }


 sen1 = analogRead(A0);
    sen2 = analogRead(A1);
    sen3 = analogRead(A2);
    sen4 = analogRead(A3);

   if(value1<sen1)
   {
    digitalWrite(led1,HIGH);
   }
   else
   {
    digitalWrite(led1,LOW);
   }
   if(value2<sen2)
   {
    digitalWrite(led2,HIGH);
   }
   else
   {
    digitalWrite(led2,LOW);
   }
    if(value3<sen3)
   {
    digitalWrite(led3,HIGH);
   }
   else
   {
    digitalWrite(led3,LOW);
   }
 if(value4<sen4)
   {
    digitalWrite(led4,HIGH);
   }
   else
   {
    digitalWrite(led4,LOW);
   }
  
 

      

  }
