// DEFINES A STEPPING ENGINE BASE ON TIMER INTERRUPTS AND PORT MANIPULATION


#define CIRCULAR_BUFFER_INT_SAFE // make circular buffers interrupt-compatible
#define BUFFER_SIZE 512
#define DEFAULT_TIMER_PERIOD 5000
#include "CircularBuffer.hpp"

// Stepping structure
struct step_t {
  uint8_t step = 0b00000000; // the STEP PORT state
  uint8_t dir = 0b00000000; // the DIR PORT state
  unsigned long period = DEFAULT_TIMER_PERIOD; // the corresponding period in microseconds
};

// BUFFER CONTAINING A COLLECTION OF STEPS
class StepBuffer : public CircularBuffer<step_t, BUFFER_SIZE> {
  void print();
};

// void StepBuffer::print() {
//   Serial.println("=== CURRENT BUFFER ===");
//   for (unsigned long i=0;i<size();i++) {
//     step_t step = this[i];
//     Serial.print(i);
//     Serial.print("\t STEP:");
//     // printBIN8(step.step);
//     Serial.print("\t DIR:");
//     // printBIN8(step.dir);
//     Serial.print("\t PERIOD:");
//     Serial.print(step.period,DEC);
//     Serial.println("us");
//   }
//   Serial.println("======================");
// };

// STEPPER ENGINE CLASS
class StepEngine {
  public:
    StepBuffer buffer;
    bool loopBuffer = true; // keep data on buffers for looping over ?
    unsigned long timerTicks = 0;
    StepperPositions _currentPosition;
    void run();
    void setup();
};

// Define the unique stepper engine
const StepEngine steppers;
// Function for the interrupt
void runSteppers() {
  steppers.run();
};

void StepEngine::run() {
  // keep track of the number of interrupts
  timerTicks++;
  // RETRIEVE THE NEXT STEP
  if (buffer.isEmpty()) return;
  step_t nextStep = buffer.pop(); // this might take too much time..
  // SET PORT STATES; DIR before STEP
  PORT_DIR = nextStep.dir;
  // Step HIGH then WAIT then LOW
  PORT_STEP = nextStep.step;
  delayMicroseconds(1);
  PORT_STEP = 0b00000000;
  // MODIFY THE TIMER PERIOD
  Timer1.setPeriod(nextStep.period);
  // IF LOOPING, RE-PUSH the current step at the end of the buffer
  if (loopBuffer) buffer.unshift(nextStep);
  // Update the current positions
  for (int8_t b=0;b<8;b++) _currentPosition[b] += (bitRead(nextStep.dir,b) ? bitRead(nextStep.step,b) : -bitRead(nextStep.step,b));
  // Is the buffer empty ?
  if (buffer.isEmpty()) Timer1.setPeriod(DEFAULT_TIMER_PERIOD);
}

void StepEngine::setup() {
  Timer1.initialize(DEFAULT_TIMER_PERIOD);
  Timer1.attachInterrupt(runSteppers);
}
