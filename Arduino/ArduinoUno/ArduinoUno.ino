/**
  Arduino robot - renew version

  ATTENTION!!!
  This version isn't complete!

  @author  Mateusz £yszkiewicz
  @version 0.1
  @since   2020 - 03 - 20

  Arduino UNO code implemented to manage robot's sensors.

 Code has handle:
 - MPU-6050 (accelerometer, gyroscope) IMU
 - sonic sensor HC-SR04
 - transmission with other microprocessor by I2C communication

 I2Cdev device library code is placed under the MIT license.
 Thanks for Jeff Rowberg for this library.
*/

#include "I2Cdev.h"			
#include "MPU6050_6Axis_MotionApps20.h"


#define intPin     2
#define trigPin    A0
#define echoPin    A1


MPU6050 mpu;
MPU6050 accelgyro;

uint8_t mpuIntStatus;   									// Holds actual interrupt status byte from MPU
uint8_t devStatus;     										// Return status after each device operation (0 = success, !0 = err)
uint16_t packetSize;    									// Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     									// Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; 									// FIFO storage buffer

Quaternion quaternion;          					// [w, x, y, z]         quaternion container
VectorFloat gravity;    									// [x, y, z]            gravity vector
float euler[3];         									// [psi, theta, phi]    Euler angle container
float ypr[3];           									// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yPrev;												      // Yaw previous data

volatile bool mpuInterrupt = false;       // Indicates whether MPU interrupt pin has gone high
  						

uint8_t error = 0;

// Gyroscope data after calibration
int16_t gxCal, gyCal, gzCal;								

int16_t ax, ay, az;
int16_t gx, gy, gz;

float axx, ayy, azz;
float gxx, gyy, gzz;

float temperature;
int distance;

unsigned long timer;


void setup() {
  
  // I2C communication begining
	Wire.begin();											
	Wire.setClock(400000);			
  
  // MPU-6050 initialize						
	mpu.initialize();										
	accelgyro.initialize();
	devStatus = mpu.dmpInitialize();

	if (devStatus == 0) {
		mpu.setDMPEnabled(true);
		mpuIntStatus = mpu.getIntStatus();
		packetSize = mpu.dmpGetFIFOPacketSize();
	}

  // HC-SR05 sensor input and output
	pinMode(echoPin, INPUT);								
	pinMode(trigPin, OUTPUT);

  // MPU-6050 interrupt input
	pinMode(intPin, INPUT_PULLUP);							

  // MPU-6050 interrupt input enable
	attachInterrupt(digitalPinToInterrupt(intPin),dmpDataReady, RISING);								

  // MPU-6050 accelerometer calibration
	accelgyro.setXAccelOffset(-3090);                       
	accelgyro.setYAccelOffset(-2330);
	accelgyro.setZAccelOffset(1280);

	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Gyroscope autocalibrate after microprocessor run
	gxCal = gx;												
	gyCal = gy;
	gzCal = gz;

  // Time variable initialize
	timer = millis();                                       
}

void loop() {

	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	temperature = mpu.getTemperature() / 340 + 36.53;
	distance = distanceMeasurement();

  // Convertion to metric value
	axx = ax * .000061f * 9.8;								
	ayy = ay * .000061f * 9.8;
	azz = az * .000061f * 9.8;

	gxx = (gx - gxCal) * .060975f;
	gyy = (gy - gyCal) * .060975f;
	gzz = (gz - gzCal) * .060975f;

  // Data collecting
	while (!mpuInterrupt && fifoCount < packetSize);

  // Reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;									
	mpuIntStatus = mpu.getIntStatus();

  // Get current FIFO count
	fifoCount = mpu.getFIFOCount();							

  // Reset MPU-6050 after data overflow
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {		
		mpu.resetFIFO();
		error = 1;
	} else if (mpuIntStatus & 0x02) {

		while (fifoCount < packetSize)
			fifoCount = mpu.getFIFOCount();

		mpu.getFIFOBytes(fifoBuffer, packetSize);
		fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	}

  // Reset MPU-6050 after calculate error
	if ((ypr[0] * 180 / M_PI - yawPrev) > 20 &&				
		millis() - timer > 3000) {

		mpu.resetFIFO();
		error = 2;
		ypr[0] = yawPrev * M_PI / 180;
		timer = millis();
	}

	dataPrint();
	yawPrev = ypr[0] * 180 / M_PI;
}

void dmpDataReady() {
	mpuInterrupt = true;
}

int distanceMeasurement() {

	int echoTime = 0;
	int distance = 0;

	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);

  // Time from transmit to receiving sonic signal 
	echoTime = pulseIn(echoPin, HIGH, 4000);			

  // Time to distance convertion
	distance = echoTime / 58;							

	return (distance >= 2 && distance < 70) ? distance : -1;
}

// Data frame send by I2C to other device
void dataPrint() {										

	char buff[64];

	Wire.beginTransmission(1);

	dtostrf(distance, 2, 0, buff);
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
		Wire.write("MPU reset!");
	Wire.write('\n');

	Wire.endTransmission(true);
}
