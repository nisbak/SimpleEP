#include <RBDdimmer.h>//

//#define USE_SERIAL  SerialUSB //Serial for boards whith USB serial port
#define USE_SERIAL  Serial
#define outputPin  12
#define minPWM  10 
#define GroupSolenoid  5 

//dimmerLamp dimmer(outputPin, zerocross); //initialase port for dimmer for ESP8266, ESP32, Arduino due boards
dimmerLamp dimmer(outputPin); //initialase port for dimmer for MEGA, Leonardo, UNO, Arduino M0, Arduino Zero
//Zero-cross on D2 for MEGA
int outVal = 0;

int analogPin = A3; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V

void setup() {
  USE_SERIAL.begin(9600);
  dimmer.begin(NORMAL_MODE, OFF); //dimmer initialisation: name.begin(MODE, STATE) 
  pinMode(GroupSolenoid, OUTPUT);
  digitalWrite(GroupSolenoid, LOW);  //
}

void loop() 
{
  outVal = map(analogRead(analogPin), 1, 1024, 100, 0); // analogRead(analog_pin), min_analog, max_analog, 100%, 0%);
  USE_SERIAL.println(outVal); 
  if (outVal > minPWM) {
    dimmer.setPower(outVal);  
    dimmer.setState(ON);
    digitalWrite(GroupSolenoid, HIGH);
  }
  else {
    dimmer.setState(OFF);
    digitalWrite(GroupSolenoid, LOW);
  }
}
