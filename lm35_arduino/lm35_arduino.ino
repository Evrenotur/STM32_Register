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
/*
 * Kod, her saniyede bir defa A3 pininden sensörün 
 * çıkışındaki Vout değerini okur ve 0,4882815 
 * çarpanı ile çarparak Celcius derece olarak hesaplar. 
 * Bu çarpanın nasıl hesaplandığı ise oldukça basittir. 
 * Bilindiği üzere Arduino analog pini okuduğu 0 ile 5V arasındaki gerilimi 0 ile 1023 arasında bir değer aralığında haritalar. LM35 sensörün Vout çıkışından okunan her 10mV değişim, fiziksel dünyadaki 1°C değişimi göstermektedir. 
 * A3 pinindeki 0-5000mV arasındaki değişimi 0-1023 aralığına bölündüğünde; 
 * 5000/1024=4,882815 sonucu elde edilir. 
 * Ancak bu değişim her 10mV ta 1°C olduğu için bu çarpan 10′ a bölünerek 4,882815/10=0,4882815 çarpanı elde edilir. Böylece, analog pinden okunan 0-1023 aralığındaki değer 0,4882815 çarpanı ile çarpılırsa fiziksel 
 * ortamın Celcius cinsinden kaç derece olduğu hesaplanmış olur. 
 * Örneğin A3 pininden ölçülen değer 21 ise, ortamın sıcaklığı; 
 * 21×0,4882815=10,25°C olarak hesaplanır.Kod, her saniyede bir 
 * defa A3 pininden sensörün çıkışındaki Vout değerini okur ve 
 * 0,4882815 çarpanı ile çarparak Celcius derece olarak hesaplar. 
 * Bu çarpanın nasıl hesaplandığı ise oldukça basittir. 
 * Bilindiği üzere Arduino analog pini okuduğu 0 ile 5V arasındaki gerilimi 0 ile 1023 arasında 
 * bir değer aralığında haritalar. LM35 sensörün Vout çıkışından okunan her 10mV değişim, 
 * fiziksel dünyadaki 1°C değişimi göstermektedir. A3 pinindeki 0-5000mV arasındaki değişimi 
 * 0-1023 aralığına bölündüğünde; 5000/1024=4,882815 sonucu elde edilir. Ancak bu değişim her 
 * 10mV ta 1°C olduğu için bu çarpan 10′ a bölünerek 4,882815/10=0,4882815 çarpanı elde edilir. 
 * Böylece, analog pinden okunan 0-1023 aralığındaki değer 0,4882815 çarpanı ile çarpılırsa 
 * fiziksel ortamın Celcius cinsinden kaç derece olduğu hesaplanmış olur. 
 * Örneğin A3 pininden ölçülen değer 21 ise, ortamın sıcaklığı; 
 * 21×0,4882815=10,25°C olarak hesaplanır.
 * 
 * 
 * 
 */
