// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


///////////////////////////////////////////////////////
//////////////////////// SERIAL OUTPUT DEFINES


//#define DATAOUT
#define DEBUGOUT


///////////////////////////////////////////////////////
//////////////////////// PUNCH DEFINES

#define A_THRESHPOS 10.0
#define A_MAXNEG    -10.0
#define A_THRESHNEG -1.0

///////////////////////////////////////////////////////
/////////////////////// FIREBALL DEFINES
#define FIRETIME 350 //[ms]

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


#define LED_PIN     15
#define ARC_PIN     16
#define VALVE_PIN   17

bool blinkState = false;

float aaXBefore;
float aaRealXNorm;

unsigned long millisNow;
unsigned long millisTMin;
unsigned long millisTMax;
float aaMax = 0;
float aaMin = 0;
bool awaitingPunch = true;
bool aaMaxSet = false;
bool aaMinSet = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

//integration and measurement vars
float dt = 0;
float vx = 0;
float vxPos;
float vxNeg;


// ================================================================
// ===                       FIRE ROUTINE                       ===
// ================================================================

void fire() {
  digitalWrite(ARC_PIN, HIGH);
  digitalWrite(VALVE_PIN, HIGH);

  delay(FIRETIME);

  digitalWrite(ARC_PIN, LOW);
  digitalWrite(VALVE_PIN, LOW);
}

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    
    #ifdef DEBUGOUT
    Serial.println("#MPU6050 Fire Fist");
    
    // initialize device
    Serial.println(F("#Initializing I2C devices..."));
    #endif
    mpu.initialize();


    #ifdef DEBUGOUT
    // verify connection
    Serial.println(F("#Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("#MPU6050 connection successful") : F("#MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("#Initializing DMP..."));
    #endif
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(151);
    mpu.setYGyroOffset(7);
    mpu.setZGyroOffset(72);
    mpu.setXAccelOffset(-1324);
    mpu.setYAccelOffset(948);
    mpu.setZAccelOffset(890);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        #ifdef DEBUGOUT
        Serial.println(F("#Enabling DMP..."));
        #endif
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        #ifdef DEBUGOUT
        Serial.println(F("#Enabling interrupt detection (Arduino external interrupt 0)..."));
        #endif
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        #ifdef DEBUGOUT
        Serial.println(F("#DMP ready! Waiting for first interrupt..."));
        #endif
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        #ifdef DEBUGOUT
        Serial.print(F("#DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        #endif
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    // also do this for arc lighter and for valve
    pinMode(ARC_PIN, OUTPUT);
    pinMode(VALVE_PIN, OUTPUT);
    // turn them off initially
    digitalWrite(ARC_PIN, LOW);
    digitalWrite(VALVE_PIN, LOW);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here 
        
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        #ifdef DEBUGOUT
        Serial.println(F("#FIFO overflow!"));
        #endif

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            
        
        //math for measurements
        millisNow = millis();
        aaXBefore = aaRealXNorm;
        aaRealXNorm = (aaReal.x  * 9.81 / 8192);

        if (awaitingPunch) {
          if(aaRealXNorm >= A_THRESHPOS) {
            awaitingPunch = 0;
            aaMax = 0;
            aaMin = 0;
            #ifdef DEBUGOUT
            Serial.println("posthresh passed!");
            #endif
          }
        } else {
          if (aaRealXNorm >= aaMax) {
            aaMax = aaRealXNorm;
            millisTMax = millisNow;
          }
          if ((aaXBefore == aaMax) && (aaRealXNorm < aaXBefore)) {
            aaMaxSet = true;
            #ifdef DEBUGOUT
            Serial.println("aMax set!");
            #endif
          }
        }

        if (aaMaxSet) {
          if (aaRealXNorm <= aaMin) {
            aaMin = aaRealXNorm;
            millisTMin = millisNow;
          }
          if ((aaXBefore == aaMin) && (aaRealXNorm > aaXBefore)) {
            aaMinSet = true;
            #ifdef DEBUGOUT
            Serial.println("aMin set!");
            #endif
            if (aaMin <= A_MAXNEG) {
              #ifdef DEBUGOUT
              Serial.println("fire!");
              #endif
              fire();
            }
          }
        }
        
        if (aaMinSet) {
          if(aaRealXNorm >= A_THRESHNEG) {
            aaMinSet = false;
            aaMaxSet = false;
            awaitingPunch = true;
            #ifdef DEBUGOUT
            Serial.println("negthresh passed!");
            Serial.print("amax: ");
            Serial.print(aaMax);
            Serial.print(" | amin: ");
            Serial.print(aaMin);
            Serial.print(" | tdecel: ");
            Serial.print(millisTMin - millisTMax);
            Serial.print(" | tmin: ");
            Serial.print(millisTMin);
            Serial.print(" | tmax: ");
            Serial.println(millisTMax);
            
            #endif
          }
        }

        #ifdef DATAOUT
        //Serial.print(millisNow);
        Serial.print(" ");
        Serial.print(aaReal.x * 9.81 / 8192);
        Serial.print(" ");
        Serial.print(aaReal.y * 9.81 / 8192);
        Serial.print(" ");
        Serial.print(aaReal.z * 9.81 / 8192);
        Serial.print("\n");
        #endif //DATAOUT
 
        
        
        
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
