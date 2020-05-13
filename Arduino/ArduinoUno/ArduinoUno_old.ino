#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define intPin     2
#define trigPin          A0
#define echoPin          A1

MPU6050 mpu;
MPU6050 accelgyro;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = err)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprp;
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

int16_t ax, ay, az;
float axx, ayy, azz;
int16_t gx, gy, gz;
float gxx, gyy, gzz;
int16_t gxCal, gyCal, gzCal;
float temperature;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

long echoTime;
int  dystans;

unsigned long timer;
uint8_t error;

boolean calibrate = true;

void setup() {
  Wire.begin();                                   //rozpoczęscie komunikacji I2C
  Wire.setClock(400000);                            //ustawienie zegara I2C
  mpu.initialize();                                   //initializacje czujnika, w zaleznosci od wykorzystywanych funkcji
  accelgyro.initialize();
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  pinMode(trigPin, OUTPUT);                            //zdefiniowanie wejscia i wyjscia dla czujnika odleglosci
  pinMode(echoPin, INPUT);
  pinMode(intPin, INPUT_PULLUP);                   //wejscie cyfrowe zarezerwowane dla przerwania od MPU-6050
  attachInterrupt(digitalPinToInterrupt(intPin), dmpDataReady, RISING);        //wlaczenie przerwania dla MPU-6050
  accelgyro.setXAccelOffset(-3090);                       //ustawienia offsetu dla akcelerometru, kalibracja
  accelgyro.setYAccelOffset(-2330);
  accelgyro.setZAccelOffset(1280);
  //dmpReady = true;
  timer = millis();                                       //inicjalizacja wartosci timera przed wejsciem do petli loop()
}

void dmpDataReady() {
  mpuInterrupt = true;
}

void loop()
{
  error = 0;

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  echoTime = echoTimeCalc();
  dystans = echoTime / 58;
  dystans = filtruj(dystans);

  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  temperature = mpu.getTemperature() / 340 + 36.53;

  if (calibrate)
  {
    gxCal = gx;
    gyCal = gy;
    gzCal = gz;
    calibrate = false;
  }

  axx = ax * .000061f * 9.8;
  ayy = ay * .000061f * 9.8;
  azz = az * .000061f * 9.8;

  gxx = (gx - gxCal) * .060975f;
  gyy = (gy - gyCal) * .060975f;
  gzz = (gz - gzCal) * .060975f;

  while (!mpuInterrupt && fifoCount < packetSize) {
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

    mpu.resetFIFO();
    error = 1;

  } else if (mpuIntStatus & 0x02) {

    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }

  if ((ypr[0] * 180 / M_PI - yprp) > 20 && millis() - timer > 3000)
  {
    mpu.resetFIFO();
    error = 2;
    ypr[0] = yprp * M_PI / 180;
    timer = millis();
  }

  wyslij_dane();
  yprp = ypr[0] * 180 / M_PI;
}

//******************************************************************************************************************

void wyslij_dane()
{
  char buff[64];
  Wire.beginTransmission(1);
  dtostrf(dystans, 2, 0, buff);
  Wire.write(buff);
  Wire.write(", ");
  dtostrf(ypr[0] * 180 / M_PI, 4, 2, buff);
  Wire.write(buff);
  Wire.write(", ");
  dtostrf(axx, 2, 2, buff);
  Wire.write(buff);
  Wire.write(", ");
  dtostrf(ayy, 2, 2, buff);
  Wire.write(buff);
  Wire.write(", ");
  dtostrf(azz, 2, 2, buff);
  Wire.write(buff);
  Wire.write(", ");
  dtostrf(gxx, 2, 2, buff);
  Wire.write(buff);
  Wire.write(", ");
  dtostrf(gyy, 2, 2, buff);
  Wire.write(buff);
  Wire.write(", ");
  dtostrf(gzz, 2, 2, buff);
  Wire.write(buff);
  Wire.write(", ");
  dtostrf(temperature, 2, 2, buff);
  Wire.write(buff);
  Wire.write(", ");
  if (error == 0)
    Wire.write(" - ");
  else if (error == 1)
    Wire.write("FIFO overflow!");
  else if (error == 2)
    Wire.write("FIFO reset!");
  Wire.write('\n');
  Wire.endTransmission(true);
}

long echoTimeCalc() {
  return pulseIn(echoPin, HIGH, 4000);
}

int filtruj(int dystans) {
  return (dystans >= 2 && dystans < 70) ? dystans : -1;
}
