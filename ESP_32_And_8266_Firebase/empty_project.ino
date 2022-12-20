#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <TimerOne.h>                          // Timer one interrupts

int sensorValue = 0;

int sensorValue1 = 0;
int i;
int sayac;
float a,pos;
//double kp = 2.01, ki = 0.011, kd = 0.0011; // CGO TETA 1
//double kp = 2.03, ki = 0.012, kd = 0.013;    // pso TETA 1
//double kp = 2.00, ki = 0.012, kd = 0.01092;  // Yaka TETA 1
double kp = 2.0, ki = 0.001, kd = 0.001;      // CGO TETA 2
//double kp = 2.02, ki = 0.011, kd = 0.0002;  // pso TETA 2
//double kp = 1.99, ki = 0.011, kd = 0.00092;  // YAKA TETA 2
String sdeger="";
double input, output, setpoint;
double iTerm = 0, lastInput = 0, dInput = 0, error = 0;
double outMin = -255, outMax = 255;
double sampleTime = 10;                        // in ms
volatile long encoderPos = 0;

#define encodPinA1      2                      // Quadrature encoder A pin
#define encodPinB1      8                       // Quadrature encoder B pin
#define M1              3                       // PWM outputs to L298N H bridge motor driver module
#define M2              11
#define led             13
 int art=0;
void setup(void)
{
  pinMode(7,INPUT);
  pinMode(12,INPUT);
  pinMode(encodPinA1, INPUT);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT);                  // quadrature encoder input B
  pinMode(led, OUTPUT);
  attachPCINT(digitalPinToPCINT(encodPinA1), encoder, FALLING); // update encoder position
  TCCR2B = TCCR2B & 0b11111000 | 1;                   // set 31Kh PWM to prevent motor whine (timer 2)
  Timer1.initialize(sampleTime * 1000);               // setup timer 1
  Timer1.attachInterrupt(Compute);
  Serial.begin(115200);                               // for debugging
}

void Compute()
{
  
 
  






  
 
      a=map(art,0,360,0,1023);
 
 
  setpoint = (a/1.5) * 10;                      // set position
  input = encoderPos;                                 // data from encoder
  error = setpoint - input;
  iTerm += ki * error * sampleTime;
  if (iTerm > outMax) iTerm = outMax;                 // prevent iTerm windup
  else if (iTerm < outMin) iTerm = outMin;
  dInput = (input - lastInput) / sampleTime;
  output = kp * error + iTerm - kd * dInput;          // compute PID Output
  if (output > outMax) output = outMax;               // limit output
  else if (output < outMin) output = outMin;
  lastInput = input;
  pwmOut(output);

}

void pwmOut(int out) {                                // to H-Bridge board
  
  if (out > 0) {
    analogWrite(M1, out);                             // drive motor CW
    analogWrite(M2, 0);
  }
  else {
    analogWrite(M1, 0);
    analogWrite(M2, abs(out));                        // drive motor CCW
  }
  
     // Serial.println("Encoder Pos");
   // Serial.println(pos);





   
    //Serial.println("\t");
   // Serial.println("PWM");
     // Serial.println(out);
}

void encoder()  {                                     // pulse and direction, direct port reading to save cycles
  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      encoderPos--;             // if (digitalRead(encodPinB1)==LOW)   count --;
}

void loop(void)
{
  if (!digitalRead(7)) {
      delay(50); /* bekleyelim */
           art +=30;
      while(!digitalRead(7)){ /* buton_saymaona basili olduğu surece bekle */
      }
      delay(10); /* dalgalanmalar için biraz bekleyelim */

      }    

  else if(!digitalRead(12))
  {
      delay(50); /* bekleyelim */
           art -=30;
      while(!digitalRead(7)){ /* buton_saymaona basili olduğu surece bekle */
      }
      delay(10); /* dalgalanmalar için biraz bekleyelim */

  }

 

  //digitalWrite(led, !digitalRead(led));               // blink led or do something else
delay(200);
   pos=map(encoderPos,0,6820,0,360);
sdeger+='*';

sdeger+= pos/100;               

sdeger+='*';

sdeger+=sensorValue1/100;

sdeger+='*';

Serial.println(sdeger );

sdeger=" ";
}
