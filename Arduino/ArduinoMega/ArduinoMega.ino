/**
	Arduino robot - renew version

  ATTENTION!!!
  This version isn't complete!

	@author  Mateusz £yszkiewicz
	@version 0.1
	@since   2020 - 03 - 20

	Example usages PS2X, Chrono and PinChangeInt library. 

	Thanks to Bill Porter for his PS2X library.
	Thanks to Sofian Audry and Thomas Ouellet Fredericks for Chrono library.
  Thanks to Mike Schwager for PinChangeInt library.
*/

#include "Arduino.h"
#include <Chrono.h>
#include <PS2X_lib.h>
#include <PinChangeInt.h>
#include "RobotDrives2WD.hpp"

// SPI pins
constexpr uint8_t data = A0;
constexpr uint8_t command = A1;
constexpr uint8_t select = A2;
constexpr uint8_t clock = A3;

// PS2 gamepad functionality
byte error = 0;
constexpr bool pressures = false;
constexpr bool rumble = false;

// Arduino logic pins for H bridge
constexpr uint8_t IN1 = 4;
constexpr uint8_t IN2 = 5;
constexpr uint8_t IN3 = 6;
constexpr uint8_t IN4 = 7;

// Arduino enable pins for H bridge
constexpr uint8_t PWM1 = 3;
constexpr uint8_t PWM2 = 9;

// Arduino auto reset pin when connecting with ps2 gamepad is lost
constexpr int resetPin = 2;

// Robot initial supply and drive voltages
constexpr float supplyVoltage = 8.2;
constexpr float nominalVoltage = 6.;
constexpr float offsetVoltage = 4.;

// Ramp times
constexpr int rampUpTime = 750;
constexpr int rampDownTime = 250;

// Interrupt pins for encoders
constexpr int interruptPinRR = A8;
constexpr int interruptPinRL = A9;
constexpr int interruptPinFR = A10;
constexpr int interruptPinFL = A11;

// Encoders variables
Chrono chronoRR;
Chrono chronoRL;
Chrono chronoFR;
Chrono chronoFL;
volatile uint8_t filter = 0;
volatile uint8_t encoderPulseRL = 0;
volatile uint8_t encoderPulse_RR = 0;
volatile uint8_t encoderPulseFL = 0;
volatile uint8_t encoderPulseFR = 0;
float offsetRR = 0;
float offsetRL = 0;
float offsetFR = 0;
float offsetFL = 0;

char dataFromUno[60];

boolean settings = false;
boolean communicationUpdate = false;

RobotDrives2WD robotDrives2WD;
PS2X gamepad;



constexpr bool DEBUG = false;



void setup() {

	// Timers frequency set as high as possible for better current control by PWM
	TCCR1B = TCCR1B & B11111000 | B00000001;
	TCCR2B = TCCR2B & B11111000 | B00000001;

  if (DEBUG)
    Serial.begin(115200);

  // Serial to Android device communication
  Serial1.begin(57600);
  
	// Gamepad and I2C configure
  Wire.begin(1);
  Wire.onReceive(receiveEvent);
	error = gamepad.config_gamepad(clock, command, select, data, pressures, rumble);

	// Robot with 2 axis configure
	robotDrives2WD.pinConfigure(IN1, IN2, PWM1, IN3, IN4, PWM2);
	robotDrives2WD.setVoltage(supplyVoltage, nominalVoltage, offsetVoltage);

	// Arduino auto reset configure
	digitalWrite(resetPin, HIGH);
	pinMode(resetPin, OUTPUT);

  // Arduino interrupts pin configure
  pinMode(interruptPinRL, INPUT_PULLUP);
  attachPinChangeInterrupt(interruptPinRL, RL, CHANGE);
  pinMode(INTERRUPT_PIN_RR, INPUT_PULLUP);
  attachPinChangeInterrupt(interruptPinRR, RR, CHANGE);
  pinMode(INTERRUPT_PIN_FL, INPUT_PULLUP);
  attachPinChangeInterrupt(interruptPinFL, FL, CHANGE);
  pinMode(INTERRUPT_PIN_FR, INPUT_PULLUP);
  attachPinChangeInterrupt(interruptPinFR, FR, CHANGE);

  supplyVoltage = analogRead(A5);
}

Chrono chronoCommunicationUpdate;

void loop() {

	// Data update every 10ms
	if (millis() % 10 == 0) {

// Stop when communication is lost
      if (communicationUpdate) {
    chronoCommunicationUpdate.reset();
    communicationUpdate = false;
  }
  else if (chronoCommunicationUpdate.elapsed() > 500) {
    dataSend();
    robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::stop);
  }
  
		// Gamepad data update
    if (error != 1) {
      gamepad.read_gamepad();

		// Arduino reset when gamepad has lost signal
		if (gamepad.Analog(PSS_LY) == 128 &&
			gamepad.Analog(PSS_LX) == 128 &&
			gamepad.Analog(PSS_RY) == 128 &&
			gamepad.Analog(PSS_RX) == 128 &&
			millis() > 5000)
			digitalWrite(resetPin, LOW);

		// Turbo mode (higher supply voltage and ramp time equals zero) when L1 and R1 is pressed
		if (gamepad.Button(PSB_L1) && gamepad.Button(PSB_R1)) {
			robotDrives2WD.setRampUpTime(0);
			robotDrives2WD.setRampDownTime(0);
			robotDrives2WD.setNominalVoltage(supplyVoltage);
		}
		else {
			robotDrives2WD.setRampUpTime(rampUpTime);
			robotDrives2WD.setRampDownTime(rampDownTime);
			robotDrives2WD.setNominalVoltage(nominalVoltage);
		}

		// Speed, angle and condition settings for robot
		if (gamepad.Analog(PSS_LY) == 127) {
			robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::stop);
		}
		else {
			int speed = gamepad.Analog(PSS_LY) - 127;
			if (speed < 0) {
				speed = -speed;
				speed++;
			}
			robotDrives2WD.setSpeed(speed << 1);

			int angle = gamepad.Analog(PSS_RX) - 128;
			if (angle < 0) {
				angle = -angle;
				angle++;
			}
			robotDrives2WD.setAngle(angle << 1);

			// If L2 and R2 is pressed, robot can rotate around its axis
			if (gamepad.Analog(PSS_LY) < 127) {
				if (!angle)
					robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::forward);
				else if (gamepad.Analog(PSS_RX) < 128) {
					if (gamepad.Button(PSB_L2) && gamepad.Button(PSB_R2))
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::left);
					else
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::forwardLeft);
				}
				else {
					if (gamepad.Button(PSB_L2) && gamepad.Button(PSB_R2))
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::right);
					else
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::forwardRight);
				}
			}
			else if (gamepad.Analog(PSS_LY) > 127) {
				if (!angle)
					robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::rear);
				else if (gamepad.Analog(PSS_RX) < 128) {
					if (gamepad.Button(PSB_L2) && gamepad.Button(PSB_R2))
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::left);
					else
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::rearLeft);
				}
				else {
					if (gamepad.Button(PSB_L2) && gamepad.Button(PSB_R2))
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::right);
					else
						robotDrives2WD.setNextCondition(RobotDrives2WD::RobotCondition::rearRight);
				}
			}
		}
    }

// Encoder filter scaled with actual speed
  encoderFilter = map(robotDrives2WD.getSpeed(), 0, 255, 12, 5);
  
		// Robot move update
		robotDrives2WD.conditionUpdate();
   dataSend();
	}
}

void serialEvent1()
{

}

void serialEvent2()
{
 
}

void receiveEvent(int bytes) {
  if (Wire.available() > 0)
    Wire.readBytesUntil('\n', dane2, bytes);
}

void dataSend() {
  if (DEBUG) {
    Serial.print(supplyVoltage);
    Serial.print(", ");
    Serial.print((float)robotDrives2WD.getSpeed() / 255, 2);
    Serial.print(", ");
    Serial.print(0);
    Serial.print(", ");
    Serial.print(encoderPulseRL);
    Serial.print(", ");
    Serial.print(encoderPulseRR);
    Serial.print(", ");
    Serial.print(speedRL);
    Serial.print(", ");
    Serial.print(speedRR);
    Serial.print(", ");
    Serial.print(speedFL);
    Serial.print(", ");
    Serial.print(speedFR);
    Serial.print(", ");
    Serial.println(dataFromUno);
  }
  if (settings) {
    Serial1.print("ust, ");
    Serial1.print((float)robotDrives2WD.getSpeed() / 255, 2);
    Serial1.print(", ");
    Serial1.print(offsetRR / 2.55, 1);
    Serial1.print(", ");
    Serial1.print(offsetRL / 2.55, 1);
    Serial1.print(", ");
    Serial1.print(offsetFR / 2.55, 1);
    Serial1.print(", ");
    Serial1.print(offsetFL / 2.55, 1);
    Serial1.print(", ");
    //    Serial1.print(0);
    //    Serial1.print(", ");
    //    Serial1.print(encoderPulseRL);
    //    Serial1.print(", ");
    //    Serial1.print(encoderPulseRR);
    //    Serial1.print(", ");
    Serial1.print(", ");
    Serial1.print(speedRL);
    Serial1.print(", ");
    Serial1.print(speedRR);
    Serial1.print(", ");
    Serial1.print(speedFL);
    Serial1.print(", ");
    Serial1.print(speedFR);
    Serial1.print(", ");
    Serial1.println(dataFromUno);
  }
  else
  {
    Serial.print((float)robotDrives2WD.getSpeed() / 255, 2);
    Serial1.print(", ");
    Serial1.print(napiecie);
    Serial1.print(", ");
    Serial1.print(encoderPulseRL);
    Serial1.print(", ");
    Serial1.print(encoderPulseRR);
    Serial1.print(", ");
    Serial1.println(dataFromUno);
  }

  Serial2.print((float)robotDrives2WD.getSpeed() / 255, 2);
  Serial2.print(", ");
  Serial2.println(0);
}

void RR()
{
  if (chronoRR.elapsed() >= encoderFilter)  {
    static volatile uint8_t sumPulseRR;
    static Chrono tempChronoRR;
    if (robotDrives2WD.getRightDrive().getCondition() == Drive::Condition::right) {
      sumPulseRR++;
      encoderPulseRR++;
    }
    else if (robotDrives2WD.getRightDrive().getCondition() == Drive::Condition::left) {
      sumPulseRR++;
      encoderPulseRR--;
    }
  }
  chronoRR.reset();
  if (sumPulseRR == 5) {
    speedRR = 2.04 / 5 / tempChronoRR.elapsed() * 1000;
    sumPulseRR = 0;
    tempChronoRR.reset();
  }
}

void RL()
{
  if (chronoRL.elapsed() >= encoderFilter)  {
    static volatile uint8_t sumPulseRL;
    static Chrono tempChronoRL;
    if (robotDrives2WD.getLeftDrive().getCondition() == Drive::Condition::left) {
      sumPulseRL++;
      encoderPulseRL++;
    }
    else if (robotDrives2WD.getLeftDrive().getCondition() == Drive::Condition::right) {
      sumPulseRL++;
      encoderPulseRL--;
    }
  }
  chronoRL.reset();
  if (sumPulseRL == 5) {
    speedRL = 2.04 / 5 / tempChronoRL.elapsed() * 1000;
    sumPulseRL = 0;
    tempChronoRL.reset();
  }
}

void FR()
{
  if (chronoFR.elapsed() >= encoderFilter)  {
    static volatile uint8_t sumPulseFR;
    static Chrono tempChronoFR;
    if (robotDrives2WD.getRightDrive().getCondition() == Drive::Condition::right) {
      sumPulseFR++;
      encoderPulseFR++;
    }
    else if (robotDrives2WD.getRightDrive().getCondition() == Drive::Condition::left) {
      sumPulseFR++;
      encoderPulseFR--;
    }
  }
  chronoFR.reset();
  if (sumPulseFR == 5) {
    speedFR = 2.04 / 5 / tempChronoFR.elapsed() * 1000;
    sumPulseFR = 0;
    tempChronoFR.reset();
  }
}

void FL()
{
  if (chronoFL.elapsed() >= encoderFilter)  {
    static volatile uint8_t sumPulseFL;
    static Chrono tempChronoFL;
    if (robotDrives2WD.getLeftDrive().getCondition() == Drive::Condition::left) {
      sumPulseFL++;
      encoderPulseFL++;
    }
    else if (robotDrives2WD.getLeftDrive().getCondition() == Drive::Condition::right) {
      sumPulseFL++;
      encoderPulseFL--;
    }
  }
  chronoFL.reset();
  if (sumPulseFL == 5) {
    speedFL = 2.04 / 5 / tempChronoFL.elapsed() * 1000;
    sumPulseFL = 0;
    tempChronoFL.reset();
  }
}
