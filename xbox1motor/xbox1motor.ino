/*
 Example sketch for the Xbox ONE USB library - by guruthree, based on work by
 Kristian Lauszus.

 Combined with DCMotorTest example from Adafruit Motor Shield v2 library.
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4. In this case, M2
Adafruit_DCMotor *myMotor = AFMS.getMotor(2);

// Analog joysticks do not always return 0 when at dead center
// Create +/- DEAD_ZONE around 0.
#define DEAD_ZONE (2000)

#include <XBOXONE.h>

// On SAMD boards where the native USB port is also the serial console, use
// Serial1 for the serial console. This applies to all SAMD boards except for
// Arduino Zero and M0 boards.
#if (USB_VID==0x2341 && defined(ARDUINO_SAMD_ZERO)) || (USB_VID==0x2a03 && defined(ARDUINO_SAM_ZERO))
#define SerialDebug SERIAL_PORT_MONITOR
#else
#define SerialDebug Serial1
#endif

USBHost UsbH;
XBOXONE Xbox(&UsbH);

bool lastDirForward = true;
uint8_t lastSpeed = 0;

void setup_motor() {
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz

  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(0);
  myMotor->run(FORWARD);
  // turn on motor
  myMotor->run(RELEASE);
  SerialDebug.println(F("Motor controller started"));
}

void setup() {
  SerialDebug.begin(115200);
  SerialDebug.println(F("Xbox One DC motor demo"));

  if (UsbH.Init()) {
    SerialDebug.print(F("\r\nUSB host did not start"));
    while (1); //halt
  }
  SerialDebug.print(F("\r\nXBOX USB Library Started"));

  setup_motor();
}

void loop() {
  UsbH.Task();

  if (Xbox.XboxOneConnected) {
    int hat_print = 0;
    if (Xbox.getAnalogHat(LeftHatX) < -DEAD_ZONE || Xbox.getAnalogHat(LeftHatX) > DEAD_ZONE) {
      SerialDebug.print(F("LeftHatX: "));
      SerialDebug.print(Xbox.getAnalogHat(LeftHatX));
      SerialDebug.print("\t");
      hat_print++;
    }

    int16_t y = Xbox.getAnalogHat(LeftHatY);
    if (y < 0) {
      if (lastDirForward) {
        lastDirForward = false;
        myMotor->run(BACKWARD);
      }
      if (y == -32768) {
        y = 32767;
      }
      else {
        y = -y;
      }
    }
    else {
      if (!lastDirForward) {
        lastDirForward = true;
        myMotor->run(FORWARD);
      }
    }
    if (y < DEAD_ZONE) y = 0;
    uint8_t speed = (uint8_t)map(y, 0, 32767, 0, 255);
    if (speed != lastSpeed) {
      lastSpeed = speed;
      myMotor->setSpeed(speed);
    }
    if (y != 0) {
      SerialDebug.print(F("LeftHatY: "));
      SerialDebug.print(y);
      SerialDebug.print("\t");
      hat_print++;
    }

    if (Xbox.getAnalogHat(RightHatX) < -DEAD_ZONE || Xbox.getAnalogHat(RightHatX) > DEAD_ZONE) {
      SerialDebug.print(F("RightHatX: "));
      SerialDebug.print(Xbox.getAnalogHat(RightHatX));
      SerialDebug.print("\t");
      hat_print++;
    }

    if (Xbox.getAnalogHat(RightHatY) < -DEAD_ZONE || Xbox.getAnalogHat(RightHatY) > DEAD_ZONE) {
      SerialDebug.print(F("RightHatY: "));
      SerialDebug.print(Xbox.getAnalogHat(RightHatY));
      hat_print++;
    }
    if (hat_print > 0) SerialDebug.println();

    if (Xbox.getButtonPress(L2) > 0 || Xbox.getButtonPress(R2) > 0) {
      if (Xbox.getButtonPress(L2) > 0) {
        SerialDebug.print(F("L2: "));
        SerialDebug.print(Xbox.getButtonPress(L2));
        SerialDebug.print("\t");
      }
      if (Xbox.getButtonPress(R2) > 0) {
        SerialDebug.print(F("R2: "));
        SerialDebug.print(Xbox.getButtonPress(R2));
        SerialDebug.print("\t");
      }
      SerialDebug.println();
    }

    if (Xbox.getButtonClick(UP))
      SerialDebug.println(F("Up"));
    if (Xbox.getButtonClick(DOWN))
      SerialDebug.println(F("Down"));
    if (Xbox.getButtonClick(LEFT))
      SerialDebug.println(F("Left"));
    if (Xbox.getButtonClick(RIGHT))
      SerialDebug.println(F("Right"));

    if (Xbox.getButtonClick(START))
      SerialDebug.println(F("Start"));
    if (Xbox.getButtonClick(BACK))
      SerialDebug.println(F("Back"));
    if (Xbox.getButtonClick(XBOX))
      SerialDebug.println(F("Xbox"));
    if (Xbox.getButtonClick(SYNC))
      SerialDebug.println(F("Sync"));

    if (Xbox.getButtonClick(L1))
      SerialDebug.println(F("L1"));
    if (Xbox.getButtonClick(R1))
      SerialDebug.println(F("R1"));
    if (Xbox.getButtonClick(L2))
      SerialDebug.println(F("L2"));
    if (Xbox.getButtonClick(R2))
      SerialDebug.println(F("R2"));
    if (Xbox.getButtonClick(L3))
      SerialDebug.println(F("L3"));
    if (Xbox.getButtonClick(R3))
      SerialDebug.println(F("R3"));


    if (Xbox.getButtonClick(A))
      SerialDebug.println(F("A"));
    if (Xbox.getButtonClick(B))
      SerialDebug.println(F("B"));
    if (Xbox.getButtonClick(X))
      SerialDebug.println(F("X"));
    if (Xbox.getButtonClick(Y))
      SerialDebug.println(F("Y"));
  }
  delay(1);
}
