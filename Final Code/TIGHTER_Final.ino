// Required Libraries
#include <TMCStepper.h>     // Used to operate TMC2209 Board
#include <AccelStepper.h>   // Tightener Stepper Control
#include <Servo.h>          // Jaw Servo Control
#include <SoftwareSerial.h> // Serial for 2209 UART

// SoftSerial Pins
#define TX_PIN      10
#define RX_PIN      9

// TMC Pins
#define P_DIAG      11   //INT0, unused
#define EN_PIN      12
#define STEP_PIN    3
#define DIR_PIN     2
// CLK tied to GND to use TMC Internal clock

// Button Pins
#define IDLER_PIN   4
#define SLIDE_PIN   6
#define TRIGGER_PIN 8

// Servo PWM Pin
#define SERVO_PIN   5

// TMC Setup Consts
#define R_SENS 0.11f // R Sense value, ohms
#define ADDR   0b00  // TMC Address
#define SG     200    // StallGuard Value, higher is more sensitive
#define TC     0xFFFFF

// Initialize SWSerial Connection
SoftwareSerial mySerial = SoftwareSerial(RX_PIN, TX_PIN);

// Initialize TMC2209
TMC2209Stepper driver(&mySerial, R_SENS, ADDR);

// Initialize Stepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Initialize Jaw Servo
Servo jawServo;

// Servo Variables
int closedPos = 0; // Closed
int openPos = 65; // Open
int halfPos = 24; // Partially Closed

// TMC Status Variables
bool isOn = false;       // Is the TMC driver enabled?
bool stalled = false;    // Did SG detect a stall
bool dir = false;        // Tightener motor direction, false = tighten

// Stepper Variables
int spd = 5000;
int openDist = 7000;
int retractDist = 2500;

// System status variables
int state = 0;

// Stall interrupt, unused
void stall() {
  stalled = true;
}

void setup() {
  // TMC Pinmodes
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(P_DIAG, INPUT);

  // Serial Pinmodes
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

  // Button Pinmodes, uses internal resistor so only wires needed are pin and gnd
  pinMode(IDLER_PIN, INPUT_PULLUP);
  pinMode(SLIDE_PIN, INPUT_PULLUP);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);

  // Servo Setup
  jawServo.attach(SERVO_PIN);
  jawServo.write(openPos);

  // Start SW Serial 
  mySerial.begin(115200);

  // TMC Setup
  digitalWrite(EN_PIN, HIGH); // Disables output
  attachInterrupt(digitalPinToInterrupt(P_DIAG), stall, RISING); // Unused
  driver.begin();
  driver.pdn_disable(true);
  driver.toff(4);
  driver.blank_time(24);
  driver.I_scale_analog(false);
  driver.internal_Rsense(false);
  driver.mstep_reg_select(true);
  driver.rms_current(350);
  driver.SGTHRS(SG);
  driver.microsteps(4);
  driver.TCOOLTHRS(TC);
  driver.TPWMTHRS(0);
  driver.semin(0);
  driver.shaft(dir);
  driver.en_spreadCycle(false);

  // Stepper Setup
  stepper.setMaxSpeed(spd);
  stepper.setSpeed(spd);

  // Wait after powerup, used if jaw stuck to open jaw for 1s
  delay(1000);
}


void loop() {
  bool pulled = digitalRead(TRIGGER_PIN); // Is trigger pulled: 0 when not pulled, 1 when pulled
  bool isHome = digitalRead(SLIDE_PIN);   // Is slider in home pos:  0 when not home, 1 when home
  bool hasZip = !digitalRead(IDLER_PIN);   // Is there a zip tie: 0 when no tie, 1 when tie (inverted from value read)

  switch(state) {
      case 0: // Initial state: slider home, jaw open, TMC disabled
        if(isOn) disable();

        if(pulled) state = 6;
        
        if(!isHome) state = 1;
        break;
      
      case 1: // First state: slider grabbing tie, jaw half closed
        jawServo.write(halfPos);
        
        if(isHome) state = 2;
        break;
        
      case 2: // Second state: slider home, tie in jaws, trigger not pulled
        jawServo.write(halfPos);
        
        if(isHome && pulled) state = 3;
        break;
      
      case 3: // Third state: slider home, trigger pulled, start tightening
        if(!isOn) enable();
        if(dir) setDirection(false);

        jawServo.write(closedPos);
        
        stepper.setSpeed(spd);
        stepper.runSpeed();
        
        if(!pulled && isHome) {
          state = 4;
          stepper.setCurrentPosition(0);
        }
        break;
      
      case 4: // Fourth state: release tie, retighten if trigger pulled again
        if(!dir) setDirection(true);

        if(stepper.currentPosition() < openDist)
          jawServo.write(halfPos);
        else
          jawServo.write(openPos);
        
        stepper.setSpeed(spd);
        stepper.runSpeed();

        
        if(isHome && pulled) state = 3;

        if(!hasZip) {
          state = 5;
          stepper.move(retractDist);
        }
        break;

      case 5: // Fifth state: loosen fully after zip isn't in idler anymore
        if(!dir) setDirection(true);
        
        jawServo.write(openPos);
        
        stepper.setSpeed(spd);
        stepper.runSpeedToPosition();
        
        if(stepper.distanceToGo() == 0) state = 0;
        break;

      
      case 6: // Seventh state: if trigger pulled during state 0, loosen zip tie
        if(!isOn) enable();
        if(!dir) setDirection(true);
        
        stepper.setSpeed(spd);
        stepper.runSpeed();

        if(!pulled) state = 0;
        break;
  }
  
}

void enable() {
  isOn = true;
  digitalWrite(EN_PIN, LOW);
}

void disable() {
  isOn = false;
  digitalWrite(EN_PIN, HIGH);
}

void setDirection(bool val){
  dir = val;
  driver.shaft(dir);
}
