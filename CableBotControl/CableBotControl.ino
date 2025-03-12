#include <Arduino.h>
#include <TMCStepper.h>
#include <SPI.h>
#include <AccelStepper.h>

const long unsigned int baudrate = 250000;

// INITIAL STEPPER MOTOR CONFIGURATION
struct {
    // TMC5160 driver configuration
    uint8_t blank_time = 24;        // [16, 24, 36, 54]
    uint8_t off_time = 3;           // [1..15]
    int8_t hysteresis_start = 1;   // [1..8]
    int8_t hysteresis_end = 12;     // [-3..12]
    uint16_t rms_current = 400;     // [0..5000]
    uint8_t microstep = 0;     // [0,1,2,4,8,16,32,64,128,256]
    bool en_pwm_mode = true; // Enable extremely quiet stepping
    bool pwm_autoscale = true; // run automatic setup of PWM mode
    uint16_t TCOOLTHRS = 0xFFFFF;
    uint16_t TPWMTHRS = 0xFFFFF;
    uint16_t THIGH = 0xFFFFF;
    // AccelStepper configuration
    uint16_t maxspeed = 300;
    uint16_t acceleration = 1000;
    float moveto[8] = {100,-100,100,-100,0,0,0,0}; // initial motion
} config ;

// see MEGA_PORTS_PINS.txt
//Driver DIR: port F (pins A0:54 to A7:61) -> DRV_REFR pins of (drivers 0 to 7)
#define DIRregister PORTF           //pin
#define DIRsetupRegister DDRF       //mode
const uint8_t DIR_PIN_array[] = {54,55,56,57,58,59,60,61};
//Driver STEP: port K (pins A8:62 to A15:69) -> DRV_REFL pins of (drivers 0 to 7)
#define STEPregister PORTK          //pin
#define STEPsetupRegister DDRK      //mode
const uint8_t STEP_PIN_array[] = {62,63,64,65,66,67,68,69};
//Driver ENABLE: port C (pins 37 to 30) -> DRV_ENN pins of (drivers 0 to 7)
#define ENABLEregister PORTC        //pin
#define ENABLEsetupRegister DDRC    //mode
const uint8_t EN_PIN_array[] = {37,36,35,34,33,32,31,30};
//SPI Chip select: port A !inverted! (pins 29 to 22) -> CSN pins of (drivers 0 to 7)
#define CSregister PORTA            //pin
#define CSsetupRegister DDRA        //mode
const uint8_t CS_PIN_array[] = {29,28,27,26,25,24,23,22};

// current sense resistance of 75 milli Ohms of the Breakout board
const float R_SENSE = 0.075;

// Declare the drivers
#define nActiveDrivers 4 // number of drivers that are completly wired to the Mega
// using namespace TMC2130_n;
// #define STALL_VALUE      15 // [-64..63]
TMC5160Stepper driver0 = TMC5160Stepper(CS_PIN_array[0], R_SENSE);
TMC5160Stepper driver1 = TMC5160Stepper(CS_PIN_array[1], R_SENSE);
TMC5160Stepper driver2 = TMC5160Stepper(CS_PIN_array[2], R_SENSE);
TMC5160Stepper driver3 = TMC5160Stepper(CS_PIN_array[3], R_SENSE);
TMC5160Stepper driver4 = TMC5160Stepper(CS_PIN_array[4], R_SENSE);
TMC5160Stepper driver5 = TMC5160Stepper(CS_PIN_array[5], R_SENSE);
TMC5160Stepper driver6 = TMC5160Stepper(CS_PIN_array[6], R_SENSE);
TMC5160Stepper driver7 = TMC5160Stepper(CS_PIN_array[7], R_SENSE);
TMC5160Stepper *driver_array[8] = {
                                  &driver0,
                                  &driver1,
                                  &driver2,
                                  &driver3,
                                  &driver4,
                                  &driver5,
                                  &driver6,
                                  &driver7
                                  };
AccelStepper stepper0 = AccelStepper(AccelStepper::DRIVER, STEP_PIN_array[0],DIR_PIN_array[0]);
AccelStepper stepper1 = AccelStepper(AccelStepper::DRIVER, STEP_PIN_array[1],DIR_PIN_array[1]);
AccelStepper stepper2 = AccelStepper(AccelStepper::DRIVER, STEP_PIN_array[2],DIR_PIN_array[2]);
AccelStepper stepper3 = AccelStepper(AccelStepper::DRIVER, STEP_PIN_array[3],DIR_PIN_array[3]);
AccelStepper stepper4 = AccelStepper(AccelStepper::DRIVER, STEP_PIN_array[4],DIR_PIN_array[4]);
AccelStepper stepper5 = AccelStepper(AccelStepper::DRIVER, STEP_PIN_array[5],DIR_PIN_array[5]);
AccelStepper stepper6 = AccelStepper(AccelStepper::DRIVER, STEP_PIN_array[6],DIR_PIN_array[6]);
AccelStepper stepper7 = AccelStepper(AccelStepper::DRIVER, STEP_PIN_array[7],DIR_PIN_array[7]);
AccelStepper *stepper_array[8] = { //first index to null pointer in order to have index matching the motor numerotation
                                  &stepper0,
                                  &stepper1,
                                  &stepper2,
                                  &stepper3,
                                  &stepper4,
                                  &stepper5,
                                  &stepper6,
                                  &stepper7
                                  };


void setup() {

  setupGPIO(); // Init pin states

  Serial.begin(baudrate);
  while (!Serial);  // Wait for serial port to connect

  setupMotorDrivers();
  ENABLEregister = 0b00000000; //enable all drivers in hardware

  // Initialize the accelstepper library
  for( uint8_t i = 0; i<nActiveDrivers; i++){
    stepper_array[i]->setMaxSpeed(config.maxspeed);
    stepper_array[i]->setAcceleration(config.acceleration);
    stepper_array[i]->setEnablePin(EN_PIN_array[i]);
    stepper_array[i]->setPinsInverted(false, false, true);
    stepper_array[i]->enableOutputs(); 
  }

  // Initial motion
  for( uint8_t i = 0; i<nActiveDrivers; i++){
    stepper_array[i]->moveTo(config.moveto[i]);
  }

}

void loop() {

  // Run the motors
  for( uint8_t i = 0; i<nActiveDrivers; i++){
    if (stepper_array[i]->distanceToGo() == 0) {
      stepper_array[i]->moveTo(-stepper_array[i]->currentPosition());  // Move 100mm
    }
  }

  // Run the motors
  for( uint8_t i = 0; i<nActiveDrivers; i++) stepper_array[i]->run();
}

void setupMotorDrivers(){
  Serial.println("Setup Motor Drivers...");
  ENABLEregister = 0b11111111;                //disable all drivers
  for( uint8_t i = 0; i<nActiveDrivers; i++){
    testDriverConnection(i);
    driver_array[i]->push();                  // reset the driver to defaults
    driver_array[i]->toff(config.off_time);
    driver_array[i]->rms_current(config.rms_current);       // Set stepper current.
    driver_array[i]->microsteps(config.microstep);           // set microstep to FULLSTEP
    driver_array[i]->en_pwm_mode(config.en_pwm_mode);          // Enable extremely quiet stepping
    driver_array[i]->pwm_autoscale(config.pwm_autoscale);        // run automatic setup of PWM mode
    driver_array[i]->TCOOLTHRS(config.TCOOLTHRS);  // FFFFF
    driver_array[i]->TPWMTHRS(config.TPWMTHRS);  // FFFFF
    driver_array[i]->THIGH(config.THIGH);  // FFFFF
  }
  Serial.println("Done.");
}

void testDriverConnection(int drv){
    Serial.print("Testing connection with driver "+String(drv)+"...");
    uint8_t result = driver_array[drv]->test_connection();
    if (result) {
        Serial.print("failed! ");
        Serial.print("Likely cause: ");
        switch(result) {
            case 1: Serial.println("loose connection"); break;
            case 2: Serial.println("no power"); break;
        }
        abort();
    }
    Serial.println("OK");
}

void setupGPIO(){
  Serial.print("Setup GPIO...");
  // sets up PINs port in OUTPUT mode
  DIRsetupRegister = 0b11111111;
  STEPsetupRegister = 0b11111111;
  CSsetupRegister = 0b11111111;
  ENABLEsetupRegister = 0b1111111;
  // initialize the state of PINs
  DIRregister = 0b0; // init all STEP as LOW
  STEPregister = 0b0; // init all STEP as HIGH
  CSregister = 0b11111111; //disable all SPI CS PINs
  ENABLEregister = 0b11111111; //disable all drivers
  SPI.begin(); delay(100);
  Serial.println("Done.");
}

// void printSGValues() {
//   Serial.print("SG Values...");
//   for( uint8_t i = 0; i<nActiveDrivers; i++){
//     DRV_STATUS_t drv_status{0};
//     drv_status.sr = driver_array[i]->DRV_STATUS();
//     Serial.print(" ");
//     Serial.print(i);
//     Serial.print(":");
//     Serial.print(drv_status.sg_result, DEC);
//     Serial.print("/");
//     Serial.print(driver_array[i]->cs2rms(drv_status.cs_actual), DEC);
//   }
//   Serial.println();

//   // Serial.print("0 ");
//   // Serial.print(drv_status.sg_result, DEC);
//   // Serial.print(" ");
//   // Serial.println(driver.cs2rms(drv_status.cs_actual), DEC);
// }
