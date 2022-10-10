
#include <SimpleKalmanFilter.h>
#include "LoRa_E32.h"
#include<Wire.h>
#include <SFE_BMP180.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;

int t1 = 3;
int t2 = 4;
float kalman_new = 0, cov_new = 0, kalman_gain = 0, kalman_calculated = 0;
int sayac = 0;
float Xivme_kalman_old = 0 , Xivme_cov_old = 0, Yivme_kalman_old = 0 , Yivme_cov_old = 0, Zivme_kalman_old = 0 , Zivme_cov_old = 0;
float ivmeX = 0, ivmeY = 0, ivmeZ = 0;
float Xdonme_kalman_old = 0 , Xdonme_cov_old = 0, Ydonme_kalman_old = 0 , Ydonme_cov_old = 0, Zdonme_kalman_old = 0 , Zdonme_cov_old = 0;
float donmeX = 0, donmeY = 0, donmeZ = 0;
double iX, iY, iZ, gX, gY, gZ ;
TinyGPS gps;
SoftwareSerial lora(10,11);//rx tx
LoRa_E32 e32ttl(&lora);
SoftwareSerial ss(4,3);//rx tx
SFE_BMP180 pressure;
bool tetik1, tetik2 , tetik;
static void smartdelay(unsigned long ms);
struct Signal {
    byte Xgyro[4];
    byte Ygyro[4];
    byte Zgyro[4];
    byte Xivme[4];
    byte Yivme[4];
    byte Zivme[4];
    byte h[4];
    byte p[4];
    byte gpsE[4];
    byte gpsB[4];
    byte gpsH[4];
} data;
 

SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
float baseline,h,P,t ,e ,b,be; 

void setup() {
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.begin(9600);
  ss.begin(9600);
  Wire.begin();
  e32ttl.begin();
  pressure.begin();
  baseline = getPressure();
    pinMode(t1,OUTPUT);
  pinMode(t2, OUTPUT);

}

double getPressure() {
  char status;
  double T,P;
  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      status = pressure.startPressure(3);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P,T);
        if (status != 0) {
          return(P);
        }
      } 
    }  
  } 
}




void loop() {

  if (tetik == 0) {
    if (h >= 2700) {
   
      digitalWrite(t1, HIGH);
      tetik1=1;
      Serial.println(h);
      tetik=1;

      }
  }
  if (tetik == 1){
     if (h <= 600) {
 
      digitalWrite(t2, HIGH);
      digitalWrite(t1, LOW);
      Serial.println(h);
      tetik=0;
      tetik2=1;
      }
  }
  
  
bmp180();
gyro();
ivme();
gpsAltitude();
gpsKonum();
smartdelay(500);

  struct Signal  {
    byte Xgyro[4];
    byte Ygyro[4];
    byte Zgyro[4];
    byte Xivme[4];
    byte Yivme[4];
    byte Zivme[4];
    byte h[4];
    byte p[4];
    byte gpsE[4];
    byte gpsB[4];
    byte gpsH[4];
    
  } data2;
 
  *(float*)(data2.Xgyro) = gX ;
  *(float*)(data2.Ygyro) = gY ;
  *(float*)(data2.Zgyro) = gZ ;

  *(float*)(data2.Xivme) = iX ;
  *(float*)(data2.Yivme) = iY ;
  *(float*)(data2.Zivme) = iZ ;
  
  *(float*)(data2.h) = h ;
  *(float*)(data2.p) = P ;
  
  *(float*)(data2.gpsE) = e ;
  *(float*)(data2.gpsB) = b ;
  *(float*)(data2.gpsH) = be ;

  

  ResponseStatus rs = e32ttl.sendFixedMessage(0, 63, 20, &data2, sizeof(Signal));
  Serial.println(rs.getResponseDescription());


}
void gyro()
{
  float gyro[3]= {0.0,0.0,0.0};
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sayac = 3;
  gX = kalman_filter(map(a.acceleration.x*4096/9.81,-4096,4096,0,180));

  sayac = 4;
  gY = kalman_filter(map(a.acceleration.y*4096/9.81,-4096,4096,0,180));
  
  sayac = 5;
  gZ = kalman_filter(map(a.acceleration.z*4096/9.81,-4096,4096,0,180));
  
  gyro[0]=gX;
  gyro[1]=gY;
  gyro[2]=gZ;
}
  
  void ivme()
{
  float ivme[3]= {0.0,0.0,0.0};
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sayac = 0;
  iX = kalman_filter(a.acceleration.x);

  sayac = 1;
  iY = kalman_filter(a.acceleration.y);
  
  sayac = 2;
  iZ = kalman_filter(a.acceleration.z);
  ivme[0]=iX;
  ivme[1]=iY;
  ivme[2]=iZ;
  
}
void bmp180()
{
  float hp[2]= {0.0,0.0};
  float p = getPressure();
  float altitude = pressure.altitude(p,baseline);
  float estimated_altitude = pressureKalmanFilter.updateEstimate(altitude);
  hp[0]= estimated_altitude;
  hp[1]=p;
  h=hp[0];
  P=hp[1];
}
void gpsKonum() {
  float gpsk[2]= {0.000000,0.000000};
  float flat, flon, invalid;
  gps.f_get_position(&flat, &flon);
  invalid = TinyGPS::GPS_INVALID_F_ANGLE;
  
  if (flat == invalid || flon == invalid) {
    gpsk[0]= 0.000000;
    gpsk[1]= 0.000000;
    e = gpsk[0];
    b = gpsk[1];
  } else {
    gpsk[0]= float((flat),6);
    gpsk[1]= float((flon),6);
    float e = gpsk[0];
    float b = gpsk[1];
    
  }
}
/*
void gpsSatellite() {
  float satellite, invalid;
  satellite = gps.satellites();
  invalid   = TinyGPS::GPS_INVALID_SATELLITES;
  if (satellite == invalid) {
    lora.print("*");
    lora.print(float(0.00));

  } else {
    lora.print("*");
    lora.print(satellite); 
  }
}*/
void gpsAltitude(){
  float gpsa[1]= {0.00};
  float gpsAltitude , invalid;
  gpsAltitude = gps.altitude() ;
  invalid   = TinyGPS::GPS_INVALID_ALTITUDE;
  if (gpsAltitude == invalid) {
    gpsa[2]= 0.00;
    be = gpsa[0];
  } else {
    gpsa[2]= float(gpsAltitude/1229.0000);
    be = gpsa[0];
    }
}
static void smartdelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}


float kalman_filter (float input)
{
  if(sayac == 0)
  {
    kalman_new = Xivme_kalman_old; // eski değer alınır
    cov_new = Xivme_cov_old + 0.50; //yeni kovaryans değeri belirlenir. Q=0.50 alınmıştır
    kalman_gain = cov_new / (cov_new + 0.9); //kalman kazancı hesaplanır. R=0.9 alınmıştır
    
    kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new)); //kalman değeri hesaplanır
  
    cov_new = (1 - kalman_gain) * Xivme_cov_old; //yeni kovaryans değeri hesaplanır
    Xivme_cov_old = cov_new; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir
  
    Xivme_kalman_old = kalman_calculated;

    return kalman_calculated; //hesaplanan kalman değeri çıktı olarak verilir
  }
  
  else if(sayac == 1)
  {
    kalman_new = Yivme_kalman_old; // eski değer alınır
    cov_new = Yivme_cov_old + 0.50; //yeni kovaryans değeri belirlenir. Q=0.50 alınmıştır
    kalman_gain = cov_new / (cov_new + 0.9); //kalman kazancı hesaplanır. R=0.9 alınmıştır
    
    kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new)); //kalman değeri hesaplanır
  
    cov_new = (1 - kalman_gain) * Yivme_cov_old; //yeni kovaryans değeri hesaplanır
    Yivme_cov_old = cov_new; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir
  
    Yivme_kalman_old = kalman_calculated;

    return kalman_calculated; //hesaplanan kalman değeri çıktı olarak verilir
  }
  
  else if(sayac == 2)
  {
    kalman_new = Zivme_kalman_old; // eski değer alınır
    cov_new = Zivme_cov_old + 0.50; //yeni kovaryans değeri belirlenir. Q=0.50 alınmıştır
    kalman_gain = cov_new / (cov_new + 0.9); //kalman kazancı hesaplanır. R=0.9 alınmıştır
    
    kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new)); //kalman değeri hesaplanır
  
    cov_new = (1 - kalman_gain) * Zivme_cov_old; //yeni kovaryans değeri hesaplanır
    Zivme_cov_old = cov_new; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir
  
    Zivme_kalman_old = kalman_calculated;

    return kalman_calculated; //hesaplanan kalman değeri çıktı olarak verilir
  }
  
  else if(sayac == 3)
  {
    kalman_new = Xdonme_kalman_old; // eski değer alınır
    cov_new = Xdonme_cov_old + 0.50; //yeni kovaryans değeri belirlenir. Q=0.50 alınmıştır
    kalman_gain = cov_new / (cov_new + 0.9); //kalman kazancı hesaplanır. R=0.9 alınmıştır
    
    kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new)); //kalman değeri hesaplanır
  
    cov_new = (1 - kalman_gain) * Xdonme_cov_old; //yeni kovaryans değeri hesaplanır
    Xdonme_cov_old = cov_new; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir
  
    Xdonme_kalman_old = kalman_calculated;

    return kalman_calculated; //hesaplanan kalman değeri çıktı olarak verilir
  }
  
  else if(sayac == 4)
  {
    kalman_new = Ydonme_kalman_old; // eski değer alınır
    cov_new = Ydonme_cov_old + 0.50; //yeni kovaryans değeri belirlenir. Q=0.50 alınmıştır
    kalman_gain = cov_new / (cov_new + 0.9); //kalman kazancı hesaplanır. R=0.9 alınmıştır
    
    kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new)); //kalman değeri hesaplanır
  
    cov_new = (1 - kalman_gain) * Ydonme_cov_old; //yeni kovaryans değeri hesaplanır
    Ydonme_cov_old = cov_new; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir
  
    Ydonme_kalman_old = kalman_calculated;

    return kalman_calculated; //hesaplanan kalman değeri çıktı olarak verilir
  }
  
  else if(sayac == 5)
  {
    kalman_new = Zdonme_kalman_old; // eski değer alınır
    cov_new = Zdonme_cov_old + 0.50; //yeni kovaryans değeri belirlenir. Q=0.50 alınmıştır
    kalman_gain = cov_new / (cov_new + 0.9); //kalman kazancı hesaplanır. R=0.9 alınmıştır
    
    kalman_calculated = kalman_new + (kalman_gain * (input - kalman_new)); //kalman değeri hesaplanır
  
    cov_new = (1 - kalman_gain) * Zdonme_cov_old; //yeni kovaryans değeri hesaplanır
    Zdonme_cov_old = cov_new; //yeni değerler bir sonraki döngüde kullanılmak üzere kaydedilir
  
    Zdonme_kalman_old = kalman_calculated;

    return kalman_calculated; //hesaplanan kalman değeri çıktı olarak verilir
  }
  

}
