#include <Wire.h>
#include <SoftwareSerial.h>
#include <math.h>

// MPU6050
const int MPU_ADDR = 0x68;//I2C address
SoftwareSerial BTSerial(2, 3); // RXI, TXO

int16_t accelerometer_x, accelerometer_y, accelerometer_z; //Accelerometer Raw Values
int16_t gyro_x, gyro_y, gyro_z;//Gyroscope Raw Values

float ax, ay, az;//Accleration
float gx, gy, gz;//Angular Accleration

float roll = 0, pitch = 0, yaw = 0;//Rotational Components

float ax_offset = 0, ay_offset = 0, az_offset = 0;// Offset to deal with drift and gravity affects
float gx_offset = 0, gy_offset = 0, gz_offset = 0;// Offset to deal with drift and gravity affects

unsigned long last_time, current_time; //For calualte dt
float dt;//dt to calualte roll, yaw, pitch
const float alpha = 0.98;//gravity

// Finger sensors
const int hallPins[4] = {A0, A1, A2, A3};  // Index, Middle, Ring, Pinky (analogue)

// Palm sensors (digital)
const int palmIndexPin  = 6;
const int palmMiddlePin = 7;
const int palmRingPin   = 8;
const int palmPinkyPin  = 9;

// Thumb sensor (digital)
const int thumbPin = 5;
const int buttonPin = 4;

float maxVoltages[4] = {0, 0, 0, 0};
float minVoltages[4] = {0, 0, 0, 0};

//const String fingerName[5] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};

bool minCalibrated = false;//check if min voltage calibrated
bool maxCalibrated = false;//check if max voltage calibrated
bool mpuCalibrated = false;//check if mpu calibrated

float readVoltage(int pin) {//voltage checking
  int sensorValue = analogRead(pin);
  return sensorValue * (5.0 / 1023.0);
}

float calibrateVoltage(int pin) {//averaging voltages
  float sumVoltage = 0;
  for (int i = 0; i < 10; i++) {
    sumVoltage += readVoltage(pin);
    delay(10);
  }
  return sumVoltage / 10.0;
}

void calibrateMinVoltages() {//min voltage checking
  for (int i = 0; i < 4; i++) {
    minVoltages[i] = calibrateVoltage(hallPins[i]);
  }
}

void calibrateMaxVoltages() {//max voltage checking
  for (int i = 0; i < 4; i++) {
    maxVoltages[i] = calibrateVoltage(hallPins[i]);
  }
}

float mapVoltageToPercentage(float voltage, float minVoltage, float maxVoltage) {//Bend percentage
  voltage = constrain(voltage, minVoltage, maxVoltage);
  return ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
}

void printVoltages(float voltages[4]) {//orinting calibrated voltages
  for (int i = 0; i < 4; i++) {
    BTSerial.print("Sensor ");
    BTSerial.print(i + 1);
    BTSerial.print(": ");
    BTSerial.print(voltages[i]);
    BTSerial.println(" V");
  }
}

void readMPU6050() {
  Wire.beginTransmission(MPU_ADDR);//begin communication with MPU6050 using i2c protocol
  Wire.write(0x3B);
  Wire.endTransmission(false);//end communication
  Wire.requestFrom(MPU_ADDR, 14, true);//continue communication

  accelerometer_x = Wire.read() << 8 | Wire.read();//read raw linear accleration data
  accelerometer_y = Wire.read() << 8 | Wire.read();
  accelerometer_z = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();// temperture data not used
  gyro_x = Wire.read() << 8 | Wire.read();// read raw rotation accleration data
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();
//convert raw data to ms^-2
  ax = (accelerometer_x / 16384.0) * 9.81;
  ay = (accelerometer_y / 16384.0) * 9.81;
  az = (accelerometer_z / 16384.0) * 9.81;
//convert raw data to degress*s^-2
  gx = gyro_x / 131.0;
  gy = gyro_y / 131.0;
  gz = gyro_z / 131.0;
}

void calibrateMPU() {
  const int num_samples = 200;// 200 samples in total
  float sum_ax = 0, sum_ay = 0, sum_az = 0;//sum of x, y, z axis linear acceleration
  float sum_gx = 0, sum_gy = 0, sum_gz = 0;//sum of x, y, z axis rotational acceleration

  for (int i = 0; i < num_samples; i++) {//loop 200 times for collecting 200 samples
    readMPU6050();//get data from this function
    sum_ax += ax;//summing all samples
    sum_ay += ay;
    sum_az += az - 9.81;//Remove Graivty only once there since az = 9.81 m/s^2 while stationary
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    delay(1);
  }
//average error when glove is still -> offset to deal with error
  ax_offset = sum_ax / num_samples;
  ay_offset = sum_ay / num_samples;
  az_offset = sum_az / num_samples;
  gx_offset = sum_gx / num_samples;
  gy_offset = sum_gy / num_samples;
  gz_offset = sum_gz / num_samples;
}

void setup() {
  Wire.begin();
  BTSerial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);//second set of sensor set up -> digital
  pinMode(thumbPin, INPUT);
  pinMode(palmIndexPin, INPUT);
  pinMode(palmMiddlePin, INPUT);
  pinMode(palmRingPin, INPUT);
  pinMode(palmPinkyPin, INPUT);

  Wire.beginTransmission(MPU_ADDR);//I2C 
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  last_time = millis();
  BTSerial.println("Press the button to calibrate minVoltages (magnets close).");
}

void loop() {
  static int buttonPressCount = 0;//button press counter

  if (digitalRead(buttonPin) == LOW) {//Botton to GND for calibration
    delay(250);
    buttonPressCount++;

    if (buttonPressCount == 1) {//If button press -> Calibrate minvoltage
      calibrateMinVoltages();
      BTSerial.println("Min Voltages Calibrated");
      printVoltages(minVoltages);
      minCalibrated = true;
      BTSerial.println("Press the button to calibrate maxVoltages (magnets far away).");
    } else if (buttonPressCount == 2) {//If button press -> Calibrate manvoltage
      calibrateMaxVoltages();
      BTSerial.println("Max Voltages Calibrated");
      printVoltages(maxVoltages);
      maxCalibrated = true;
      BTSerial.println("Press the button to start MPU calibration (keep glove still).");
    } else if (buttonPressCount == 3) {//If button press -> calibrate MPU6050
      calibrateMPU();
      mpuCalibrated = true;
      BTSerial.println("MPU Calibration complete. Glove active.");
    }
  }

  if (buttonPressCount >= 3) {
    float bendPercentages[5];

    // Thumb (digital)
    if (digitalRead(thumbPin) == LOW) {
      bendPercentages[0] = 100;
    } else {
      bendPercentages[0] = 0;
    }

    for (int i = 0; i < 4; i++) {//100% bending confirmation
      float voltage = readVoltage(hallPins[i]);
      float bend = mapVoltageToPercentage(voltage, minVoltages[i], maxVoltages[i]);

      if (i == 0 && bend >= 70 && digitalRead(palmIndexPin) == LOW) bend = 100;
      if (i == 1 && bend >= 70 && digitalRead(palmMiddlePin) == LOW) bend = 100;
      if (i == 2 && bend >= 70 && digitalRead(palmRingPin) == LOW) bend = 100;
      if (i == 3 && bend >= 70 && digitalRead(palmPinkyPin) == LOW) bend = 100;

      bendPercentages[i + 1] = bend;
    }

    current_time = millis();//Get curret time
    dt = (current_time - last_time) / 1000.0;//get dt
    last_time = current_time;//new last time is old current time

    readMPU6050();//read raw data from MPU6050
//remove errors
    ax -= ax_offset;
    ay -= ay_offset;
    az -= az_offset;
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    float accel_roll = atan2(ay, az) * 180 / M_PI;//roll angle estimation
    float accel_pitch = atan2(ax, az) * 180 / M_PI;//pitch angle estimation
//filter combines gyro and accleratometer data for accuracy
    roll = alpha * (roll + gx * dt) + (1 - alpha) * accel_roll;
    pitch = alpha * (pitch + gy * dt) + (1 - alpha) * accel_pitch;
    yaw += gz * dt;
//Estimate axis affected by gravity
    float gravityX = sin(pitch * M_PI / 180.0) * 9.81;
    float gravityY = -sin(roll * M_PI / 180.0) * 9.81;
    float gravityZ = cos(pitch * M_PI / 180.0) * cos(roll * M_PI / 180.0) * 9.81;
//Remove gravity
    float ax_linear = ax - gravityX;
    float ay_linear = ay - gravityY;
    float az_linear = az - gravityZ;

    //For Simple Test

    /*BTSerial.print("Finger- T:"); BTSerial.print(bendPercentages[0], 1);
    BTSerial.print(" I:"); BTSerial.print(bendPercentages[1], 1);
    BTSerial.print(" M:"); BTSerial.print(bendPercentages[2], 1);
    BTSerial.print(" R:"); BTSerial.print(bendPercentages[3], 1);
    BTSerial.print(" P:"); BTSerial.print(bendPercentages[4], 1);

    BTSerial.print(" | Roll:"); BTSerial.print(roll, 1);
    BTSerial.print(" Pitch:"); BTSerial.print(pitch, 1);
    BTSerial.print(" Yaw:"); BTSerial.print(yaw, 1);

    BTSerial.print(" | ax:"); BTSerial.print(ax_linear, 1);
    BTSerial.print(" ay:"); BTSerial.print(ay_linear, 1);
    BTSerial.print(" az:"); BTSerial.println(az_linear, 1);*/

    //Unity All
    BTSerial.print("Finger-T:");
    BTSerial.print(bendPercentages[0], 1);
    BTSerial.print(",I:");
    BTSerial.print(bendPercentages[1], 1);
    BTSerial.print(",M:");
    BTSerial.print(bendPercentages[2], 1);
    BTSerial.print(",R:");
    BTSerial.print(bendPercentages[3], 1);
    BTSerial.print(",P:");
    BTSerial.print(bendPercentages[4], 1);

    // Add rotation values (roll, pitch, yaw)
    BTSerial.print(",Roll:");
    BTSerial.print(roll, 1);
    BTSerial.print(",Pitch:");
    BTSerial.print(pitch, 1);
    BTSerial.print(",Yaw:");
    BTSerial.print(yaw, 1);

// Add linear acceleration (you can later interpret this as velocity/position)
    BTSerial.print(",ax:");
    BTSerial.print(ax_linear, 1);
    BTSerial.print(",ay:");
    BTSerial.print(ay_linear, 1);
    BTSerial.print(",az:");
    BTSerial.print(az_linear, 1);

    BTSerial.println("|");

  }
}




