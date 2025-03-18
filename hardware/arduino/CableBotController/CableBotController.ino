#include <Arduino.h>
#include <SPI.h>

#include <TimerOne.h>

#define BAUDRATE 250000
#define MAX_ACCELERATION 5000
#define MAX_SPEED 1000
#define N_MOTORS 4  // number of activated motors
#define RMS_CURRENT 500

#include "DriverConfig.h" // defines the TMC5160 driver controller

#include "StepperData.h" // defines objects that describe the state of the stepper motors

#include "StepEngine.h" // defines the engine that generates the signals for the steppers

// =================================================================== SETUP
void setup() {
  digitalWrite(LED_BUILTIN,LOW) ;

  SPI.begin();
  Serial.begin(BAUDRATE);
  while (!Serial);  // Wait for serial port to connect
  Serial.println();

  // INITIALIZE TMC5160 DRIVERS
  drivers.setup();
  drivers.testConnection(); // optional
  drivers.applyConfig();
  drivers.enable();

  // Wait for Serial to open
  digitalWrite(LED_BUILTIN,HIGH);

  // Initialize stepper engine
  steppers.setup();

  Serial.println("Setup Finished.");
}

// =================================================================== LOOP
unsigned long lastMillis = 0;
unsigned long printPeriodMillis = 100;
void loop() {

  if (Serial.available()) { // a new buffer has been sent
    Timer1.stop(); delay(100);
    steppers.buffer.clear();
    // CONTINUOUS ROTATION
      // for (unsigned long i=0;i<BUFFER_SIZE/2;i++) {
      //   step_t newStep ;
      //   newStep.step = 0b00001111;//random(255);//0b00000000;
      //   newStep.dir = 0b00001111;//random(255);//0b00000000;
      //   float speed = float(MAX_SPEED)*2.0*float(i)/float(BUFFER_SIZE);
      //   newStep.period = long(1000000.0/speed); //random(minTimerDelay,maxTimerDelay);
      //   stepBuffer.unshift(newStep);
      // };
      // for (unsigned long i=BUFFER_SIZE/2;i>0;i--) {
      //   step_t newStep ;
      //   newStep.step = 0b00001111;//random(255);//0b00000000;
      //   newStep.dir = 0b00001111;//random(255);//0b00000000;
      //   float speed = float(MAX_SPEED)*2.0*float(i)/float(BUFFER_SIZE);
      //   newStep.period = long(1000000.0/speed); //random(minTimerDelay,maxTimerDelay);
      //   stepBuffer.unshift(newStep);
      // };
    // RANDOM BUFFER
      // for (unsigned long i=0;i<BUFFER_SIZE;i++) {
      //   step_t newStep ;
      //   newStep.step = random(255);//0b00000000;
      //   newStep.dir = random(255);//0b00000000;
      //   newStep.period = 10000; //random(minTimerDelay,maxTimerDelay);
      //   stepBuffer.unshift(newStep);
      // }
    // BUFFER FROM SERIAL
    while (Serial.available()>=2) {
      byte buffer[2];
      step_t newStep ;
      newStep.step = Serial.read();
      newStep.dir = Serial.read();
      // newStep.period = Serial.readByte();
      while (!steppers.buffer.available()); // wait for the buffer to have free space
      steppers.buffer.unshift(newStep);
    }
    // PRINT BUFFER
    while(Serial.available()) Serial.read(); // empty the serial input
    printBuffer();
    Timer1.start();
  }

  // Print some info
  unsigned long dmillis = millis()-lastMillis;
  if (dmillis>=printPeriodMillis) {
    Serial.print("ticks:"+String(steppers.timerTicks));
    Serial.print("\t pos: "+steppers._currentPosition.to_String());
    Serial.println();
    lastMillis = millis();
  }
}


// =================================================================== STEP INTERRUPT
// void step() {
//   // keep track of the number of interrupts
//   timerTicks++;
//   // RETRIEVE THE NEXT STEP
//   if (stepBuffer.isEmpty()) return;
//   step_t nextStep = stepBuffer.pop(); // this might take too much time..
//   // SET PORT STATES; DIR before STEP
//   PORT_DIR = nextStep.dir;
//   PORT_STEP = nextStep.step;
//   delayMicroseconds(2);
//   PORT_STEP =  0b00000000;
//   // MODIFY THE TIMER PERIOD
//   Timer1.setPeriod(nextStep.period);
//   // IF LOOPING, RE-PUSH the current step at the end of the buffer
//   if (loopBuffer) stepBuffer.unshift(nextStep);
//   // Update the current positions
//   for (int8_t b=0;b<8;b++) currentPosition[b] += (bitRead(nextStep.dir,b) ? bitRead(nextStep.step,b) : -bitRead(nextStep.step,b));
//   // Is the buffer empty ?
//   if (stepBuffer.isEmpty()) Timer1.setPeriod(defaultStep.period);
// }


// =================================================================== PRINT FUNCTIONS
void printBuffer() {
  Serial.println("=== CURRENT BUFFER ===");
  for (unsigned long i=0;i<steppers.buffer.size();i++) {
    step_t step = steppers.buffer[i];
    Serial.print(i);
    Serial.print("\t STEP:");
    printBIN8(step.step);
    Serial.print("\t DIR:");
    printBIN8(step.dir);
    Serial.print("\t PERIOD:");
    Serial.print(step.period,DEC);
    Serial.println("us");
  }
  Serial.println("======================");
}

void printBIN8(uint8_t B8) {
    for (int b=0;b<8;b++) Serial.print(bitRead(B8,b),BIN);
}

void printPosition(long pos[8]){
  for (int8_t b=0;b<8;b++) {
    Serial.print("\t P");
    Serial.print(b);
    Serial.print(":");
    Serial.print(pos[b]);
  }
}

// =================================================================== OTHER FUNCTIONS
void bresenham(long x0[8],long x1[8]) {
  // which coordinate is the master ?
}
