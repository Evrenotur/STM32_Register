
#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#endif

//1. Firebase veritabanı adresini, Token bilgisini ve ağ adresi bilgileri.
#define FIREBASE_HOST "espevren2-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "MfXEuyR4XvGNRjwUEIaxFGJDIIlQv0p38hwlOgSa"

#define WIFI_SSID "DESKTOP-5KOI0HL 9768"
#define WIFI_PASSWORD "D7209+o1"

FirebaseData veritabanim;
#define led1 17 //5
#define led2 5 
#define led3 18
#define led4 1
#define led5 19
int UPDATE_INTERVAL_SECS = 0; // x * 60 (dk cinsinden)
int c;
   String erisim;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);

  // connect to wifi.
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("connecting");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("connected: ");
  Serial.println(WiFi.localIP());
   // Firebase bağlantısı başlatılıyor
  // Ağ bağlantısı kesilirse tekrar bağlanmasına izin veriyoruz
  Firebase.reconnectWiFi(true);
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

   pinMode(led1,OUTPUT);
 pinMode(led2,OUTPUT);
}

void loop() {
   
  if(Firebase.getInt(veritabanim, "c1")) //Alınacak veri tipine göre getInt, getBool, getFloat, getDouble, getString olarak kullanılabilir.
  {
    //bağlantı başarılı ve veri geliyor ise
    Serial.print("String tipinde veri alımı başarılı, veri = ");
    Serial.println(veritabanim.stringData());
    if (veritabanim.intData()==1){
      digitalWrite(led1,HIGH);
    //  Serial.print("Led Yandi");
      delay(200);
    }
    else {
      
      digitalWrite(led1,LOW);
     // Serial.print("Led Söndü");
        delay(300);
      }
    
    

  }else{
    //hata varsa hata mesajı ve nedeni yazdırılıyor

    Serial.print("Str verisi çekilemedi, ");
    Serial.println(veritabanim.errorReason());
  }


  if(Firebase.getInt(veritabanim, "c2")) //Alınacak veri tipine göre getInt, getBool, getFloat, getDouble, getString olarak kullanılabilir.
  {
    //bağlantı başarılı ve veri geliyor ise
    Serial.print("int tipinde veri alımı başarılı, veri = ");
    Serial.println(veritabanim.intData());
    if (veritabanim.intData()==1){
      digitalWrite(led2,HIGH);
       // Serial.print("Led Yand ");
          delay(300);
    
    }
    else {
      
      digitalWrite(led2,LOW);
     // Serial.print("Led Söndü  ");
        delay(200);
      }
    
    

  }else{
    //hata varsa hata mesajı ve nedeni yazdırılıyor

    Serial.print("int verisi çekilemedi, ");
    Serial.println(veritabanim.errorReason());
  }





  if(Firebase.getInt(veritabanim, "c3")) //Alınacak veri tipine göre getInt, getBool, getFloat, getDouble, getString olarak kullanılabilir.
  {
    //bağlantı başarılı ve veri geliyor ise
    Serial.print("int tipinde veri alımı başarılı, veri = ");
    Serial.println(veritabanim.intData());
    if (veritabanim.intData()==1){
      digitalWrite(led3,HIGH);
        Serial.print("Led Yandi-3");
          delay(300);
    
    }
    else {
      
      digitalWrite(led3,LOW);
      Serial.print("Led Söndü-3");
        delay(200);
      }
    
    

  }else{
    //hata varsa hata mesajı ve nedeni yazdırılıyor

    Serial.print("int verisi çekilemedi, ");
    Serial.println(veritabanim.errorReason());
  }



    if(Firebase.getInt(veritabanim, "c4")) //Alınacak veri tipine göre getInt, getBool, getFloat, getDouble, getString olarak kullanılabilir.
  {
    //bağlantı başarılı ve veri geliyor ise
    Serial.print("int tipinde veri alımı başarılı, veri = ");
    Serial.println(veritabanim.intData());
    if (veritabanim.intData()==1){
      digitalWrite(led4,HIGH);
       //Serial.print("Led Yand ");
          delay(300);
    
    }
    else {
      
      digitalWrite(led4,LOW);
     // Serial.print("Led Söndü  ");
        delay(200);
      }
    
    

  }else{
    //hata varsa hata mesajı ve nedeni yazdırılıyor

    Serial.print("int verisi çekilemedi, ");
    Serial.println(veritabanim.errorReason());
  }



  if(Firebase.getInt(veritabanim, "c5")) //Alınacak veri tipine göre getInt, getBool, getFloat, getDouble, getString olarak kullanılabilir.
  {
    //bağlantı başarılı ve veri geliyor ise
    Serial.print("int tipinde veri alımı başarılı, veri = ");
    Serial.println(veritabanim.intData());
    if (veritabanim.intData()==1){
      digitalWrite(led5,HIGH);
     //   Serial.print("Led Yand ");
          delay(300);
    
    }
    else {
      
      digitalWrite(led5,LOW);
    //  Serial.print("Led Söndü  ");
        delay(200);
      }
    
    

  }else{
    //hata varsa hata mesajı ve nedeni yazdırılıyor

    Serial.print("int verisi çekilemedi, ");
    Serial.println(veritabanim.errorReason());
  }

if(Firebase.getString(veritabanim, "Erişim Tarih cihaz 1", erisim)){ //Alınacak veri tipine göre getInt, getBool, getFloat, getDouble, getString olarak kullanılabilir.
  
    //bağlantı başarılı ve veri geliyor ise
    Serial.print("String tipinde veri alımı başarılı, veri = ");
    Serial.print(erisim);

 
    
  
}
if(Firebase.setString(veritabanim, "erscihaz2", erisim))
{
    //bağlantı başarılı ve veri geliyor ise
     Serial.println("String tipinde veri gönderimi başarılı");
 
  }else{
  //hata varsa hata mesajı ve nedeni yazdırılıyor

   Serial.print("String tipindeki veri gönderilemedi, ");
   Serial.println(veritabanim.errorReason());
  }
    



  
////////////////////////////////////////////////////////////////////////////////////
// firebase veritabanına veri göndermek için Firebase.setInt komutu kullanılabilir.
String a = "300";

//cihaz id yi a değşkeninin içine yazınız
if(Firebase.setString(veritabanim, "cihaz1", a))
{
    //bağlantı başarılı ve veri geliyor ise
     Serial.println("String tipinde veri gönderimi başarılı");
 
  }else{
  //hata varsa hata mesajı ve nedeni yazdırılıyor

   Serial.print("String tipindeki veri gönderilemedi, ");
   Serial.println(veritabanim.errorReason());
  }

  

}
