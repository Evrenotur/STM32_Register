//Kullanıcıdan veri alalım
#include<Servo.h>

char veri;
int mesafe, tahmin, pot_deger, ldr_deger;

const int trigPin = 9;    //Ultrasonic sensörün trig pini
const int echoPin = 10;   //Ultrasonic sensörün echo pini
const int redPin = 2;     //Kırmızı led pin
const int greenPin = 3;   //Yeşil led pin
const int bluePin = 4;    //Mavi led pin
const int yellowPin = 5;  //Sarı led pin
const int buzzerPin = 6;  //Buzzer pin
const int servoPin = 7;   //Servo motor pin
const int potPin = A0;    //Potansiyometre pin
const int ldrPin = A1;    //LDR pin

Servo myservo;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  myservo.attach(servoPin);
  Serial.begin(9600);
}

void loop() {
  //Kullanıcıdan veri alalım
  Serial.println("a, b, c veya d girin:");
  while (!Serial.available());
  veri = Serial.read();

  if (veri == 'a') {
    //Mesafe ölçümü yapalım
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    mesafe = pulseIn(echoPin, HIGH) / 58.2;

    //Mesafe değerini ekrana yazdıralım
    Serial.print("Mesafe: ");
    Serial.print(mesafe);
    Serial.println(" cm");

    //Mesafe değerine göre işlemler yapalım
    if (mesafe >= 15 && mesafe <= 25) {
      for (int i = 0; i < 5; i++) {
        digitalWrite(bluePin, HIGH);
        myservo.write(0);
        tone(buzzerPin, 1000);
        delay(300);
        digitalWrite(bluePin, LOW);
        myservo.write(90);
        noTone(buzzerPin);
        delay(300);
      }
    } else if (mesafe >= 4 && mesafe <= 10) {
      for (int i = 0; i < 10; i++) {
        digitalWrite(redPin, HIGH);
        myservo.write(90);
        tone(buzzerPin, 2000);
        delay(100);
        digitalWrite(redPin, LOW);
        myservo.write(0);
        noTone(buzzerPin);
        delay(100);
      }
    } else {
      Serial.println("Mesafe aralığında değil.");
    }
  } else if (veri == 'b') {
    //Rastgele sayı üretelim
    tahmin = random(0,6);

   if(tahmin==2){
    digitalWrite(redPin, HIGH);
    Serial.println("kirmizi led yaniyor");
  }
else if(tahmin==3){
    digitalWrite(redPin, HIGH);
    delay(1000);
    digitalWrite(yellowPin, HIGH);
    delay(1000);
    digitalWrite(greenPin, HIGH);
    delay(1000);
    digitalWrite(bluePin, HIGH);
    
  }
  }

  else if(veri=='c'){
       int pot =analogRead(potPin);
        Serial.println(pot);
        delay(pot);
         digitalWrite(redPin, HIGH);
         for (int i = 0; i <= 90; i++) {
    myservo.write(i);
    delay(15);
  }
  delay(pot);
 digitalWrite(redPin, LOW);
  delay(pot);

    
  }
  else if(veri=='d'){
   int ldr= analogRead(ldrPin);
   if(ldr<500){
     digitalWrite(redPin, HIGH);
   }
   else{
    digitalWrite(redPin, LOW);
   }
  }
 
  else{
    Serial.println("lutfen a,b,c,d degerlerini giriniz");
  }
  
}

  
