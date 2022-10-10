
#include <SimpleKalmanFilter.h>
#include "LoRa_E22.h"
#include<Wire.h>
#include <SFE_BMP180.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;
int t1 = 3;
int t2 = 4;
float flame_sensor =A0;
float kalman_new = 0, cov_new = 0, kalman_gain = 0, kalman_calculated = 0;
int sayac = 0;
float Xivme_kalman_old = 0 , Xivme_cov_old = 0, Yivme_kalman_old = 0 , Yivme_cov_old = 0, Zivme_kalman_old = 0 , Zivme_cov_old = 0;
float ivmeX = 0, ivmeY = 0, ivmeZ = 0;
float Xdonme_kalman_old = 0 , Xdonme_cov_old = 0, Ydonme_kalman_old = 0 , Ydonme_cov_old = 0, Zdonme_kalman_old = 0 , Zdonme_cov_old = 0;
float donmeX = 0, donmeY = 0, donmeZ = 0;
float iX, iY, iZ, gX, gY, gZ,ta,ts ;
TinyGPS gps;
LoRa_E22 E22(&Serial);
SoftwareSerial ss(6,5);//rx tx
SFE_BMP180 pressure;
bool tetik1, tetik2,donme;
static void smartdelay(unsigned long ms);
int tix , th, thf  = 0;
float fark,fark2 = 0.0;

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
    byte gpsS[4];
    byte T1[4];
    byte T2[4];
} data;
 

SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);
float baseline,h,P,t ,e ,b,be,bes; 

void setup() {

  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.begin(9600);
  ss.begin(9600);
  Wire.begin();
  E22.begin();
  pressure.begin();
  baseline = getPressure();
  pinMode(t1,OUTPUT);
  pinMode(t2, OUTPUT);
  digitalWrite(t1, LOW); 
  digitalWrite(t2, LOW); 
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
  
    bmp180();
    gyro();
    ivme();
    gpsAltitude();
    gpsKonum();
    gpsSpeed();
    smartdelay(250);
    
    Yfark();
    Xfark();
    if ((h >= 1500 && fark >1.0) && (iX<=-8 && fark2 > 1.0)){
      digitalWrite(t1, HIGH);
      thf=1 ;
      bmp180();
      ts = h;
    }
    if((h <= 620 && thf == 1 )&&( fark > 1.0 && fark2 > 1.0)){
      digitalWrite(t2, HIGH); 
      thf =0;
      bmp180();
      ta = h;
    }

       
  *(float*)(data.Xgyro) = gX ;
  *(float*)(data.Ygyro) = gY ;
  *(float*)(data.Zgyro) = gZ ;

  *(float*)(data.Xivme) = iX ;
  *(float*)(data.Yivme) = iY ;
  *(float*)(data.Zivme) = iZ ;
  
  *(float*)(data.h) = h ;
  *(float*)(data.p) = P ;
  
  *(float*)(data.gpsE) = e ;
  *(float*)(data.gpsB) = b ;
  *(float*)(data.gpsH) = be ;
  *(float*)(data.gpsS) = bes ;

  *(float*)(data.T1) = ts ;
  *(float*)(data.T2) = ta ;
  
  ResponseStatus rs = E22.sendFixedMessage(0, 63, 20, &data, sizeof(Signal));
}
void Yfark(){
  
  bmp180();
  fark = h;
  delay(250);
  bmp180();
  fark = fark - h;
  
}
void Xfark(){
  
  bmp180();
  fark2 = h;
  delay(250);
  bmp180();
  fark2= fark2 - h;
  
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
    gpsk[0]= float(flat),6;
    gpsk[1]= float(flon),6;
    e = gpsk[0];
    b = gpsk[1]; 
  }
}
void gpsAltitude(){
  float gpsa[1]= {0.00};
  float gpsAltitude , invalid;
  gpsAltitude = gps.f_altitude() ;
  invalid   = TinyGPS::GPS_INVALID_F_ALTITUDE;
  if (gpsAltitude == invalid) {
    gpsa[0]= 0.00;
    be = gpsa[0];
  } else {
    gpsa[0]= float(gpsAltitude-909.20);
    be = gpsa[0];
    }
}
void gpsSpeed(){
  float gpss[1]= {0.00};
  float gpsSpeed , invalid;
  gpsSpeed = gps.f_speed_kmph() ;
  invalid   = TinyGPS::GPS_INVALID_F_SPEED;
  if (gpsSpeed == invalid) {
    gpss[0]= 0.00;
    bes = gpss[0];
  } else {
    gpss[0]= float(gpsSpeed);
    bes = gpss[0];
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
