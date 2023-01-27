
// irs - vl53l1 based sensors on irs17a boards (pwm signal) 
// vlx - chinesium vl53l0x boards
// driver - drv8833 

#include <MPU9250.h>
#include <PID_v2.h>

TaskHandle_t Task2;
MPU9250 mpu;

// pinout
#define AIN_1 26
#define AIN_2 25
#define BIN_1 32
#define BIN_2 27
#define MOT_EN 33

#define IRS_LEFT 23
#define IRS_RIGHT 19
#define IRS_FRONT 18


#define BAUD_RATE 115200
#define VLX_REFRESH_PERIOD 20000 //us
#define START_DELAY 2000 // minimal start delay


#define AVOID_DIST 150 // mm
#define BREAKING_DIST_FRONT 500 // mm
#define BREAKING_DIST_SIDES 250
#define BOOST_DIST_FRONT 1000 // mm
#define BOOST_DIST_SIDES 600 // mm 


#define MAX_GYRO_ERROR 5 // degrees 
#define MAX_VIENKE_ROT 6
#define MAX_DELTA_SPD 5
#define ACCEL_GAP 1

#define MIN_MOTOR_SPD 210 
#define MAX_MOTOR_SPD 50
#define AVOID_SPD 100
#define GYRO_CORECT_SPD 100
#define PID_SPD 140
#define BOOST_MAX_SPD 150
#define VIENKE_DELTA_SPD 60

#define KP_DEFAULT 1
#define KI_DEFAULT 0
#define KD_DEFAULT 0
#define KP_BOOST 0.0075
#define KI_BOOST -0.0005
#define KD_BOOST 0.001


unsigned long milk_0; // timer for start delay
unsigned long milk_1; // timer for acceleration
unsigned long milk_2;

double distF;
double dist1;
double dist2;
double prevdistF=1;
double prevdist1=1;
double prevdist2=1;

double Yaw;
double Pitch;
double Roll;
double aPitch;

double kP;
double kI;
double kD;
double target = 0;

double lastSPD=0;
int acelCounter = 0;
int senstime;

bool vienke = false;


void sensors(void * pvParameters);
int speed();
void PID();
void drive();
void avoid();
void vienkeCorrect(int strenght, int speedA);
//bool gyroCorr(bool correct = false);
PID_v2 PIDobj(kP, kI, kD, PID::Direct);


void setup() {
  milk_0 = millis();

  xTaskCreatePinnedToCore(
             sensors,  /* Task function. */
             "Task2",    /* name of task. */
             10000,      /* Stack size of task */
             NULL,       /* parameter of the task */
             1,          /* priority of the task */
             &Task2,     /* Task handle to keep track of created task */
             1);  
  
  Wire.begin();
  Serial.begin(BAUD_RATE);
  Serial.println("Hi");
  // pinmodes
  pinMode(AIN_1, OUTPUT);
  pinMode(AIN_2, OUTPUT);
  pinMode(BIN_1, OUTPUT);
  pinMode(BIN_2, OUTPUT);
  pinMode(MOT_EN, OUTPUT);

  pinMode(IRS_LEFT, INPUT);
  pinMode(IRS_RIGHT, INPUT);

  

  //mpu init
  if (!mpu.setup(0x68)) {  // change to your own address
      while (1) {
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(5000);
      }
  }
  
  // start delay
  while(millis()-milk_0 <= START_DELAY)
  {
    digitalWrite(MOT_EN, LOW);
  }
  digitalWrite(MOT_EN, HIGH);
  Serial.println("Hi");
}

void loop() {

  /*
  Serial.print(dist1);
  Serial.print("  ");
  Serial.print(distF);
  Serial.print("  ");
  Serial.print(dist2);
  Serial.print("  ");//*/
  milk_1 =millis();
  if(distF <= AVOID_DIST)
  {
    avoid();
  }
  else
  {
    int spdNow = speed();
    int error = (dist1-dist2);
    double PIDval = PIDobj.Run(error);
    drive(spdNow+PIDval, spdNow-PIDval);
  }
  //Serial.print(millis() - milk_1);
  //Serial.print("  ");
  //Serial.println(senstime);
}

void sensors(void * pvParameters)
{
while(1)
{
  milk_2 = millis();
  dist1 = pulseIn(IRS_LEFT, HIGH);
  dist1 = (dist1 - 1000) * 2;
  if(dist1 > 1300 ) 
    dist1 = 1300;
  if(dist1 < 0)
    dist1 =prevdist1;
  prevdist1 = dist1;

  distF = pulseIn(IRS_FRONT, HIGH);
  distF = (distF - 1000) * 2;
  if(distF > 480 ) 
    distF = 550;
  if(distF < 0)
    distF = prevdistF;
  prevdistF = distF;

  dist2 = pulseIn(IRS_RIGHT, HIGH);
  dist2 = (dist2 - 1000) * 2;
  if(dist2 > 1300 ) 
    dist2 = 1300;
  if(dist2 < 0)
    dist2 =prevdist2;
  prevdist2 = dist2;

  if (mpu.update()) {
    Yaw = mpu.getYaw();
    Pitch = mpu.getPitch();
    Roll = mpu.getRoll();
    vienke = false;
    double TMP =mpu.getGyroY();
    if(TMP > MAX_VIENKE_ROT)
      vienke = true;
  }
  
  senstime = millis()-milk_2;
}
}

int speed()
{
  // wanted speed 
  int returnSPD = PID_SPD;

  // boost check
  kP = KP_DEFAULT;
  kI = KI_DEFAULT;
  kD = KD_DEFAULT;
  if(distF >= BOOST_DIST_FRONT && dist1 >= BOOST_DIST_SIDES && dist2 >= BOOST_DIST_SIDES)
  {
    returnSPD = BOOST_MAX_SPD;
    kP = KP_BOOST;
    kI = KI_BOOST;
    kD = KD_BOOST;
  }
  //breaking
  /*if(distF <= BREAKING_DIST_FRONT)
  {
    returnSPD = map(distF, AVOID_DIST, BREAKING_DIST_FRONT, MIN_MOTOR_SPD, PID_SPD);
    returnSPD = constrain(returnSPD, MIN_MOTOR_SPD, MAX_MOTOR_SPD);
  }
  else
  {
    if(dist1 <=BREAKING_DIST_SIDES || dist2 <=BREAKING_DIST_SIDES)
    {
      if(dist1 <= dist2)
      {
        returnSPD = map(dist1, AVOID_DIST, BREAKING_DIST_SIDES, MIN_MOTOR_SPD, PID_SPD);
        returnSPD = constrain(returnSPD, MIN_MOTOR_SPD, MAX_MOTOR_SPD);
      }
      else
      {
        returnSPD = map(dist2, AVOID_DIST, BREAKING_DIST_SIDES, MIN_MOTOR_SPD, PID_SPD);
        returnSPD = constrain(returnSPD, MIN_MOTOR_SPD, MAX_MOTOR_SPD);
      }
    }
  }*/
   
  //acceleration
  if(lastSPD-returnSPD >= MAX_DELTA_SPD)
  {
    acelCounter++;
    if(acelCounter >= ACCEL_GAP)
    {
        returnSPD = lastSPD+1; 
    }
  }
  else
  {
    acelCounter = 0;  
  }
  lastSPD = returnSPD;

  if(vienke)
  {
    vienkeCorrect(200, returnSPD);
  }
  
  return returnSPD;
}

void drive(int spdA, int spdB)
{
  ///*
  if(spdA == spdB)
  {
    Serial.print("<---( ");
    Serial.print(spdA);
    Serial.println(" )--->");
  }
  else
  {
    if(spdA > spdB)
    {
      Serial.print("--( ");
      Serial.print(spdA);
      Serial.print(" )--( ");
      Serial.print(spdB);
      Serial.println(" )->");
    }
    else
    {
      Serial.print("<-( ");
      Serial.print(spdA);
      Serial.print(" )--( ");
      Serial.print(spdB);
      Serial.println(" )--");
    }
  }
  //*/
  
  //actual driving crap
  if(spdA==0 && spdB==0)  
  {
    digitalWrite(AIN_1, HIGH);
    digitalWrite(AIN_2, HIGH);
    digitalWrite(BIN_1, HIGH);
    digitalWrite(BIN_2, HIGH);
  }  
  else
  {
    //motor A  
    if(spdA > 0)
    {
      analogWrite(AIN_1, spdA);
      digitalWrite(AIN_2, LOW);
    }
    else 
    {
      digitalWrite(AIN_1, LOW);
      analogWrite(AIN_2, -spdA);
    }
    // motor B
    if(spdB > 0)
    {
      analogWrite(BIN_1, spdB);
      digitalWrite(BIN_2, LOW);
    }
    else 
    {
      digitalWrite(BIN_1, LOW);
      analogWrite(BIN_2, -spdB);
    }
  }
}

void avoid()
{
  if(dist1 >= dist2)
  {
    while(distF<= AVOID_DIST+20)
    {
      Serial.println("Avoiding");

      drive(-AVOID_SPD, -0.5*AVOID_SPD);
    }  
  }
  else
  {
    while(distF<= AVOID_DIST+20)
    {
      Serial.println("Avoiding");

      drive(-0.5*AVOID_SPD, -AVOID_SPD);
    } 
  }
  drive(0, 0);
}

void vienkeCorrect(int strenght, int speedA)
{
  Serial.println("vienke");
  int bsA = speedA - VIENKE_DELTA_SPD;
  bsA = constrain(bsA, 0, MAX_MOTOR_SPD);
  drive(bsA, bsA);
  delayMicroseconds(strenght);
}















