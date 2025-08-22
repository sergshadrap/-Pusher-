#include <Arduino.h>

#include <MppServer.h>
// #include <MppDevice.h>
#include <Stepper.h>

const int IN1 = 4;
const int IN2 = 13;
const int IN3 = 12;
const int IN4 = 14;
// Adjust to your motor; this only affects RPM scale in Stepper lib.

// Your required travel:
const int kTravelSteps = 1800;

const char* DeviceVersion = "MppPusher 1.0.0"; 
static const char *properties[] = {
  P_BUTTON_PIN,      // optional local button
  P_LED_PIN,         // optional status LED
  P_INITIAL,         // initial state
  P_USE_LAST,        // persist & restore last state
  P_CYCLE_RECOVERY,  // optional wifi recovery restarts
  P_WIFI_RESTART,    // optional wifi watchdog
  NULL
};
        
const int stepsPerRevolution = 1800;  // number of steps per revolution for your motor

// initialize the Stepper library on pins 8 through 11:
// Stepper myStepper(stepsPerRevolution, IN1, IN2, IN3, IN4);
class MppServer mppServer(DeviceVersion, properties);

class MotorSwitch : public MppDevice {
public:
  MotorSwitch(unsigned p1, unsigned p2, unsigned p3, unsigned p4)
  : stepper(stepsPerRevolution, p1, p2, p3, p4),
    pin1(p1), pin2(p2), pin3(p3), pin4(p4) {}

  void begin() override {
    // Initialize GPIOs and stepper speed
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(pin3, OUTPUT);
    pinMode(pin4, OUTPUT);
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, LOW);
    digitalWrite(pin4, LOW);
   stepper.setSpeed(100); // RPM (tune as needed)

    // Start in OFF state
    put(STATE, "off");
  }


  bool handleAction(String action, MppParameters parms) override {
    bool handled = false;

    // Preferred API: /state?state=on|off
    if (action == "state" && parms.hasParameter("state")) {
      String s = parms.getParameter("state");
      s.toLowerCase();
      if (s == "on" || s == "1" || s == "true") {
        handled = true;
        runOn();
      } else if (s == "off" || s == "0" || s == "false") {
        handled = true;
        runOff();
      }
    }
 /*   // Also accept shorthand actions
    else if (action == "on") {
      handled = true;
      runSequenceAndReset();
    } else if (action == "off") {
      handled = true;
      put(STATE, "off");
    }*/

    // Fallback to base class if not handled
    return handled ? true : MppDevice::handleAction(action, parms);
  }


private:
  void runOn() {
    // Mark ON while running
    put(STATE, "on");

    // Counterclockwise 1800, wait, clockwise 1800
    stepper.step(-kTravelSteps);
    delay(100);
    stepper.step(kTravelSteps);

    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, LOW);
    digitalWrite(pin4, LOW);
  }

  void runOff() {
    // Mark ON while running
    put(STATE, "off");

    // Counterclockwise 1800, wait, clockwise 1800
    stepper.step(-kTravelSteps);
    delay(100);
    stepper.step(kTravelSteps);

    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
    digitalWrite(pin3, LOW);
    digitalWrite(pin4, LOW);
  }

  Stepper stepper;
  unsigned pin1, pin2, pin3, pin4;
};

MotorSwitch motor(IN1, IN2, IN3, IN4);


void setup() {

  
  
//  myStepper.setSpeed(100);
  // initialize the serial port:
  Serial.begin(115200);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
   digitalWrite(IN1, 0);
    digitalWrite(IN2, 0);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 0);


    mppServer.manageDevice(&motor, getDefaultUDN(MppSwitch));
    mppServer.begin();
}

void loop() {
  mppServer.handleClients();
}


/*
 * 
 * 
 void loop() {
  // step one revolution in one direction:
  Serial.println("clockwise ");
  myStepper.step(-stepsPerRevolution);
    delay(100);
    Serial.println("counterclockwise");
   myStepper.step(stepsPerRevolution);  
  // step one revolution in the other direction:

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  delay(30000);
}

void setup()
{
  Serial.begin(115200);
 Serial.printf("Reverse/forward driving begin\n");
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  //delay is used to control the speed, the lower the faster.
  //reverse(step,delay);
//  reverse(1000,3);
  //forward(step,delay);
  forward(1000,3);
}

void loop()
{
  reverse(1800,4);
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 0);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 0);
  delay(1000);
  forward(1800,2);
   digitalWrite(IN1, 0);
    digitalWrite(IN2, 0);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 0);
  delay(30000);
}

void reverse(int i, int j) {
  Serial.printf("Reverse driving\n");

// Serial.printf("Reverse driving GPIO%d in state %d / GPIO%d in state %d /GPIO%d in state %d / GPIO%d in state %d \n",IN1,digitalRead(IN1),IN2,digitalRead(IN2),IN1,digitalRead(IN3),IN1,digitalRead(IN4));
  while (1)   {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
//Serial.printf("Reverse driving1 GPIO%d in state %d / GPIO%d in state %d /GPIO%d in state %d / GPIO%d in state %d  DELAY:%d\n",IN1,digitalRead(IN1),IN2,digitalRead(IN2),IN1,digitalRead(IN3),IN1,digitalRead(IN4),j);    
    delay(j);
    i--;
    if (i < 1) break; 

    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
//Serial.printf("Reverse driving2 GPIO%d in state %d / GPIO%d in state %d /GPIO%d in state %d / GPIO%d in state %d DELAY:%d \n",IN1,digitalRead(IN1),IN2,digitalRead(IN2),IN1,digitalRead(IN3),IN1,digitalRead(IN4),j);    
    delay(j);  
    i--;
    if (i < 1) break;

    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
//Serial.printf("Reverse driving3 GPIO%d in state %d / GPIO%d in state %d /GPIO%d in state %d / GPIO%d in state %d DELAY:%d \n",IN1,digitalRead(IN1),IN2,digitalRead(IN2),IN1,digitalRead(IN3),IN1,digitalRead(IN4),j);    
    delay(j);
    i--;
    if (i < 1) break;

    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
//Serial.printf("Reverse driving4 GPIO%d in state %d / GPIO%d in state %d /GPIO%d in state %d / GPIO%d in state %d DELAY:%d \n",IN1,digitalRead(IN1),IN2,digitalRead(IN2),IN1,digitalRead(IN3),IN1,digitalRead(IN4),j);    
    delay(j);  
    i--;
    if (i < 1) break;
  }
  
}  

void forward(int i, int j) {
Serial.printf("Forward driving\n");
  while (1)   {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
//Serial.printf("Forward driving1 GPIO%d in state %d / GPIO%d in state %d /GPIO%d in state %d / GPIO%d in state %d DELAY:%d \n",IN1,digitalRead(IN1),IN2,digitalRead(IN2),IN1,digitalRead(IN3),IN1,digitalRead(IN4),j);    
    delay(j);  
    i--;
    if (i < 1) break;

    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
//Serial.printf("Forward driving2 GPIO%d in state %d / GPIO%d in state %d /GPIO%d in state %d / GPIO%d in state %d DELAY:%d \n",IN1,digitalRead(IN1),IN2,digitalRead(IN2),IN1,digitalRead(IN3),IN1,digitalRead(IN4),j);    
    delay(j);
    i--;
    if (i < 1) break;

    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
//Serial.printf("Forward driving3 GPIO%d in state %d / GPIO%d in state %d /GPIO%d in state %d / GPIO%d in state %d DELAY:%d \n",IN1,digitalRead(IN1),IN2,digitalRead(IN2),IN1,digitalRead(IN3),IN1,digitalRead(IN4),j);
    delay(j);  
    i--;
    if (i < 1) break;

    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
//Serial.printf("Forward driving4 GPIO%d in state %d / GPIO%d in state %d /GPIO%d in state %d / GPIO%d in state %d DELAY:%d \n",IN1,digitalRead(IN1),IN2,digitalRead(IN2),IN1,digitalRead(IN3),IN1,digitalRead(IN4),j);
    delay(j);
    i--;
    if (i < 1) break;
  }



}  */
