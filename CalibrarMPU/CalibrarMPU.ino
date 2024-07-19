#include <stdio.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// #define USE_OFFSET

// MPU6050 mpu;
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
float gx, gy, gz;

float gx_offset=-0.04712604,gy_offset=-0.00997102,gz_offset=-0.00537880;

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  Wire.begin();
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  delay(2000);

  digitalWrite(2, HIGH);
  int count = 5000/10;  
  
  for (int i=0; i<count; i++) {
    mpu.getEvent(&a, &g, &temp);
    gx += g.gyro.x;
    gy += g.gyro.y;
    gz += g.gyro.z;
    delay(10);
  }
  gx /= count;
  gy /= count;
  gz /= count;

  #ifdef USE_OFFSET
  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;
  #endif

  digitalWrite(2, LOW);
  printf("\nResultado da Calibração:\ngx_offset = %.08f;\ngy_offset = %.08f;\ngz_offset = %.08f;\n", gx, gy, gz);
}

void loop() {
  delay(1000);
}
