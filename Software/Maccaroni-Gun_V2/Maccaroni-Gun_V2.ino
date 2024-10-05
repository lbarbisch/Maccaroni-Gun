#include <Servo.h>

Servo servo;
bool oldState = true;
const int servoPin = 2;
const int gndSwitchPin = A2;
const int switchPin = A1;
const int triggerPin = A4;
const int gndTriggerPin = A5;

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
  pinMode(gndTriggerPin, OUTPUT);
  digitalWrite(gndTriggerPin, LOW);
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(triggerPin, INPUT_PULLUP);

  // init done
  Serial.println("Maccaroni-Gun Ready!");
}

void loop()
{
  if (digitalRead(switchPin) == false)    // check if safety is set to "DANGER"
  {
    // transition from off to on
    if ((oldState == false) && (digitalRead(triggerPin) == true))
    {
      Serial.println("enable motor");
      servo.write(power);
    }
    
    // transition from on to off (only for serial message)
    if ((oldState == true) && (digitalRead(triggerPin) == false))
    {
      Serial.println("off");
      servo.write(0);
    }
  }
  
  else      // safety switch is set to "SAFE"
  {
    servo.write(0);
  }
  
  oldState = digitalRead(triggerPin);
  delay(10);
}
