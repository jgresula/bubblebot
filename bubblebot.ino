//! -*- mode:c++ -*-
#include "bubblebot.h"

#define FAN_PIN 0
#define SERVO_PIN 1
#define FAN_SPEED_POT_PIN A1
#define BOT_SPEED_PIN A3

#define TCNTn   TCNT1
#define OCRnx   OCR1A
#define OCFnx   OCF1A
#define OCIEnx  OCIE1A


//  4 ticks (32us) - combined time spent inside the interrupt
#define TRIM_DURATION 0

// HITEC HS-81
#define SERVO_PULSE_MIN 700
#define SERVO_PULSE_MAX 2400
// prescaler PCK/64 is not 8us but rigol shows 1/510.2Hz/255
#define REAL_TIMER_TICK 7.6862


#define ARM_DOWN_ANGLE 149
#define ARM_UP_ANGLE 32
#define ARM_STEP_ANGLE 15
#define ARM_MOVE_DELAY 80
#define POT_MAX_READ_VAL 1024
#define POT_READ_INTERVAL 50

// time to wait before moving the arm down after the fan stops
#define BEFORE_ARM_DOWN_DELAY 100
// min time to wait before moving the arm up
#define MIN_ARM_DOWN_DELAY (ARM_MOVE_DELAY+20)
// max time to wait before moving the arm down
#define MAX_ARM_DOWN_DELAY 15000
// time to wait before starting the fan after the arm
#define BEFORE_FAN_BLOW_DELAY 100


#define MIN_FAN_BLOW_DURATION 2500
#define MAX_FAN_BLOW_DURATION 1000
#define MIN_FAN_PWM 70
#define MAX_FAN_PWM 200

#define BEFORE_DETACH_DELAY (2*ARM_MOVE_DELAY)

// pot 1
int fanSpeedPWM = 0;

// pot 2
int16_t armDownDelay = MIN_ARM_DOWN_DELAY; //5000;
unsigned long armDownDelayStartTime = 0;


unsigned long botScheduler = 0;
unsigned long nextPotReadTime = 0;
bool readPwmPot = true;
unsigned long timeNow = 0;
int armCurrentAngle = 0;
bot_state_t botState = STATE_INITIAL;
bot_state_t stateAfterServoDetach = STATE_INITIAL;
unsigned long delayAfterServoDetach = 0;


//
//
//
void setup() {
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  // wait for decoupling caps
  delay(1000);
  servoTimerSetup();
  initialArmSetup();
}

//
//
//
void loop() {
  timeNow = millis();

  // read the fan speed pot
  if (nextPotReadTime < timeNow) {
    // -
    if (readPwmPot) {
      int potVal = analogRead(FAN_SPEED_POT_PIN);
      fanSpeedPWM = map(potVal, 0, POT_MAX_READ_VAL, MIN_FAN_PWM, MAX_FAN_PWM);
      fanSpeedPWM = min(MAX_FAN_PWM, fanSpeedPWM);
    } else {
      int potVal = analogRead(BOT_SPEED_PIN);
      armDownDelay = map(potVal, 0, POT_MAX_READ_VAL, MIN_ARM_DOWN_DELAY, MAX_ARM_DOWN_DELAY);
      armDownDelay = min(MAX_ARM_DOWN_DELAY, armDownDelay);
      armDownDelay = max(MIN_ARM_DOWN_DELAY, armDownDelay);

      // try to update immediately
      if (armDownDelayStartTime > 0 && botState == STATE_ARM_UP) {
        botScheduler = armDownDelayStartTime + armDownDelay;
      }
    }

    readPwmPot = !readPwmPot;
    nextPotReadTime += POT_READ_INTERVAL;
  }

  if (botScheduler < timeNow)
    bubbleBotLoop();
}


void initialArmSetup()
{
  armCurrentAngle = ARM_DOWN_ANGLE;
  writeServo(armCurrentAngle);
  delay(1000);
  botState = STATE_ARM_UP;
}

bool updateArmAngle(int targetAngle) {
  if (targetAngle > armCurrentAngle) {
    armCurrentAngle += ARM_STEP_ANGLE;
    if (armCurrentAngle >= targetAngle) {
      armCurrentAngle = targetAngle;
    }
  } else if (targetAngle < armCurrentAngle) {
    armCurrentAngle -= ARM_STEP_ANGLE;
    if (armCurrentAngle < targetAngle) {
      armCurrentAngle = targetAngle;
    }
  }

  writeServo(armCurrentAngle);
  return armCurrentAngle == targetAngle;
}

void scheduleNext(int offset, enum bot_state_t state) {
  botScheduler = timeNow + offset;
  botState = state;
}

void scheduleDetachServoAndThen(int offset, enum bot_state_t state) {
  botScheduler = timeNow + BEFORE_DETACH_DELAY;
  botState = STATE_DETACH_SERVO;
  delayAfterServoDetach = max(0, (offset-BEFORE_DETACH_DELAY));
  stateAfterServoDetach = state;
}

int fanOnCountdown = 0;

//
//
//
void bubbleBotLoop() {
  switch(botState) {
  case STATE_ARM_DOWN:
    attachServo();
    if(updateArmAngle(ARM_DOWN_ANGLE)) {
      armDownDelayStartTime = timeNow;
      scheduleDetachServoAndThen(armDownDelay, STATE_ARM_UP);
    } else {
      scheduleNext(ARM_MOVE_DELAY, STATE_ARM_DOWN);
    }
    break;

  case STATE_ARM_UP:
    armDownDelayStartTime = 0; 
    if(updateArmAngle(ARM_UP_ANGLE)) {
      scheduleDetachServoAndThen(BEFORE_FAN_BLOW_DELAY, STATE_FAN_BLOW);
    } else {
      scheduleNext(ARM_MOVE_DELAY, STATE_ARM_UP);
    }
    break;

  case STATE_FAN_BLOW: {
    analogWrite(FAN_PIN, fanSpeedPWM);
    // linear inerpolation of PWM between MIN/MAX_FAN_BLOW_DURATION
    int16_t fanBlowDuration =
      map(fanSpeedPWM, MIN_FAN_PWM, MAX_FAN_PWM, MIN_FAN_BLOW_DURATION, MAX_FAN_BLOW_DURATION);
    fanOnCountdown = fanBlowDuration / POT_READ_INTERVAL + 1;
    scheduleNext(POT_READ_INTERVAL, STATE_FAN_BLOW_IN_PROGRESS);
  }
    break;

  case STATE_FAN_BLOW_IN_PROGRESS:
    // live fan speed update according to the pot position
    analogWrite(FAN_PIN, fanSpeedPWM);
    if (0 == --fanOnCountdown) {
      analogWrite(FAN_PIN, 0);  // stop fan
      scheduleNext(BEFORE_ARM_DOWN_DELAY, STATE_ARM_DOWN);
    } else {
      scheduleNext(POT_READ_INTERVAL, STATE_FAN_BLOW_IN_PROGRESS);
    }
    break;

  // out of order
  case STATE_DETACH_SERVO:
    detachServo();
    scheduleNext(delayAfterServoDetach, stateAfterServoDetach);
    break;
  }

   
}



volatile uint8_t servoPulseTicks = 0;
int servoNumInterruptsWaitingForNext = 0;

enum servo_state_t {
  SERVO_DISABLED,
  WAITING_FOR_HIGH,
  WAITING_FOR_512,
  WAITING_FOR_LOW,
  WAITING_FOR_NEXT_PULSE
};
volatile servo_state_t servoState = SERVO_DISABLED;

void attachServo() {
  DDRB |= (1<<SERVO_PIN);
}

void writeServo(int angle) {
  int16_t pulseLength = map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);

  int16_t ticks = pulseLength/REAL_TIMER_TICK - 64;
  ticks = max(0, ticks);
  ticks = min(255, ticks);
  servoPulseTicks = static_cast<uint8_t>(ticks);

  TCNTn = 0;
  OCRnx = 64;
  servoState = WAITING_FOR_HIGH;
}

void detachServo() {
  servoState = SERVO_DISABLED;
  TCNTn = 0;
  OCRnx = 64;
}


ISR(TIM1_COMPA_vect)
{

switch(servoState) {
  case SERVO_DISABLED:
    PORTB &= ~(1<<SERVO_PIN);
    break;

  case WAITING_FOR_HIGH:
    PORTB |= (1<<SERVO_PIN);
    TCNTn = 0;
    OCRnx = 64 - TRIM_DURATION;
    servoState = WAITING_FOR_512;
    break;

  case WAITING_FOR_512:
    OCRnx = servoPulseTicks;
    servoState = WAITING_FOR_LOW;
    TCNTn = 0;
    if (OCRnx != 0) {
      // we need to clear the OCF1A flag because it is possible that the counter
      // value incremented and matched the output compare value while this
      // function was being executed
      TIFR = (1 << OCF1A);  // write logical 1 to the OCF0A flag to clear it
                            // also have to write 0 to all other bits for this to work.
    } else {
      TCNTn = 0xff;
    }
    break;

  case WAITING_FOR_LOW:
    PORTB &= ~(1<<SERVO_PIN);
    servoNumInterruptsWaitingForNext = 10;

    TCNTn = 0;
    OCRnx = 255 - servoPulseTicks;
    servoState = WAITING_FOR_NEXT_PULSE;
    break;

  case WAITING_FOR_NEXT_PULSE:
    if (0 == --servoNumInterruptsWaitingForNext) {
      servoState = WAITING_FOR_HIGH;
      TCNTn = 0;
      OCRnx = 1;
    } else {
      TCNTn = 0;
      OCRnx = 255;
    }
    break;
  }

}

//
//
//
void servoTimerSetup()
{
    //set up the timer prescaler based on which timer was selected and our F_CPU clock
    setupTimerPrescaler();

    // Enable Output Compare Match Interrupt
    TIMSK |= (1 << OCIEnx);

    //reset the counter to 0
    TCNTn  = 0;
    //set the compare value to any number larger than 0
    OCRnx = 255;
    // Enable global interrupts
    sei();
}


//
//
//
void setupTimerPrescaler()
{
        //reset the Timer Counter Control Register to its reset value
        TCCR1 = 0;

        #if F_CPU == 8000000L
            //set counter1 prescaler to 64
            //our F_CPU is 8mhz so this makes each timer tick 8 microseconds long
            TCCR1 &= ~(1<< CS13); //clear
            TCCR1 |=  (1<< CS12); //set
            TCCR1 |=  (1<< CS11); //set
            TCCR1 |=  (1<< CS10); //set

        #elif F_CPU == 1000000L
            //set counter1 prescaler to 8
            //our F_CPU is 1mhz so this makes each timer tick 8 microseconds long
            TCCR1 &= ~(1<< CS13); //clear
            TCCR1 |=  (1<< CS12); //set
            TCCR1 &= ~(1<< CS11); //clear
            TCCR1 &= ~(1<< CS10); //clear
        #else
            #error "unsupported clock speed"
        #endif
}

