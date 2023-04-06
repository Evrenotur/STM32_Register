
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
