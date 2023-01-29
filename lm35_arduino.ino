int sicaklik; // Sıcaklık biriminin tutulması için kullanılacak değişkeni tanımla
void setup() {
  pinMode(A0,INPUT); // LM35 bağlı olan A0 pinini giriş pini olarak ata
  Serial.begin(9600); // Bilgisayara veri aktarımını başlat  
}
void loop() {
  int deger = analogRead(A0); // A0 pinindeki sensör değerini oku
  sicaklik = deger * 0.48828125; // A0 pininden okunan değeri Celcius cinsinden sıcaklık birimine dönüştür
  Serial.println(sicaklik); // Seri port ekranına sıcaklığı yazdır
  delay(1000); // Bir sonraki ölçüm için 1 saniye bekle
}
