#include <Servo.h>

Servo servo;
bool oldState = true;
const int servoPin = 2;
const int gndSwitchPin = A2;
const int switchPin = A1;

const uint16_t power = 75;

void setup()
{
  Serial.begin(115200);
  Serial.println("Initializing Maccaroni-Gun");
  
  // prepare esc controll
  servo.attach(servoPin);
  servo.write(0); // set to 0
  delay(5000);    // let the esc initialize

  // prepare on/off switch
  pinMode(gndSwitchPin, OUTPUT);
  digitalWrite(gndSwitchPin, LOW);
  pinMode(switchPin, INPUT_PULLUP);

  // init done
  Serial.println("Maccaroni-Gun Ready!");
}

void loop()
{
  // transition from off to on
  if ((oldState == true) && (digitalRead(switchPin) == false))
  {
    Serial.println("enable motor");
    servo.write(power);
  }
  
  // transition from on to off (only for serial message)
  if ((oldState == false) && (digitalRead(switchPin) == true))
  {
    Serial.println("off");
  }
  
  // if switch off motor is also off (slightly safety critical)
  if (digitalRead(switchPin) == true)
  {
    servo.write(0);
  }
  
  oldState = digitalRead(switchPin);
  delay(10);
}
