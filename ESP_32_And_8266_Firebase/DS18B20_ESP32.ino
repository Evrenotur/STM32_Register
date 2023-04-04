#include <WiFi.h>
#include <FirebaseESP32.h>
#include <OneWire.h>
#include <DallasTemperature.h>
FirebaseData veritabanim;
#define FIREBASE_HOST "ds18b20-31b41-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "IUsnoIGUtbOFDIAc9MP6ofaGYRfzNjDbcc03icLg"
#define WIFI_SSID "Evren"
#define WIFI_PASSWORD "bilinmez"
 float temperatureC ;
  float temperatureF ;


const int oneWireBus = 4;     

OneWire oneWire(oneWireBus);


DallasTemperature sensors(&oneWire);
void setup()
{
Serial.begin(9600);

  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
  Serial.print(".\n");
  }
    delay(500);

     Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());

  Firebase.reconnectWiFi(true);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  sensors.begin();
}
void loop(){

  sensors.requestTemperatures(); 
  temperatureC = sensors.getTempCByIndex(0);
   temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureC);
 if(Firebase.setFloat(veritabanim, "TempatureC", temperatureC))
{
    //bağlantı başarılı ve veri geliyor ise
     Serial.println("Tempature tipinde veri gönderimi başarılı");
 
  }else{


   Serial.print("Tempatureftipindeki veri gönderilemedi, ");
   Serial.println(veritabanim.errorReason());
  }
  if(Firebase.setFloat(veritabanim, "TemperatureF", temperatureF))
{
    //bağlantı başarılı ve veri geliyor ise
     Serial.println("Tempaturef tipinde veri gönderimi başarılı");
 
  }else{


   Serial.print("Tempaturef tipindeki veri gönderilemedi, ");
   Serial.println(veritabanim.errorReason());
  }
  

}
