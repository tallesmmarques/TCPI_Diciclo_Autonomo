#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "FS.h"
#include "SPIFFS.h"

#include "tools.h"

void PinSetup();
void taskBlinkLed(void* pvParameter);
int fileCountInit();
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);

// CONST VARIABLES
const float ts = 3e-3;
const float alpha_w = 0.5;
const float r_w = 0.03;
const float w = 11e-2;
const float K[3] = {-44.9181, -30.9875, -6.1045};
const float Ki = 35.3553;

// GLOBALS VARIABLES
ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

Data *data_array;
Modes startMode = INIT; 

// MPU6050 mpu;
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
float gy_offset = -0.01000992;

int count = 0;
int flash_count = 0;
int file_count = 0;
char buffer[100];

Kalman kalman_theta(ts);

float position_r, position_l, x, dx, theta, psi;
float last_position_r, last_position_l, last_theta, Vr, Vl;
float g_x, g_y, g_z;
float a_pitch, a_yaw, a_roll;
float dwheel_f_r, dwheel_f_l;
float V, DeltaV;
int pwm_value;

float error_psi, error_psi_int;
float ref_psi_f, ref_psi = 0;

float q[3] = {0};
float error_dx_int = 0;
float ref_dx = 0.0;

void taskFlash(void* pvParameter) {
  String filename = "/arquivo" + String(file_count) + ".csv";
  Serial.println(filename);
  File file = SPIFFS.open(filename, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }

  Data data_read;
  while(1) {
    if (flash_count < count) {
      data_read = data_array[flash_count % BUFFER_SIZE];

      if (count - flash_count > BUFFER_SIZE) startMode = BUFFER_OVERFLOW;

      file.write((const uint8_t*)&data_read, sizeof(Data));

      flash_count++;
    } else if (startMode == CONFIG || (startMode == BUFFER_OVERFLOW && flash_count == count)) {
      file.close();

      file_count++;
      file = SPIFFS.open(FILE_COUNT_BKP, FILE_WRITE);
      if(!file || file.isDirectory()) {
        Serial.println("- failed to open file bkp for something");
      }
      file.write((uint8_t*)&file_count, sizeof(file_count));
      file.close();

      printf("End Flash\n");
      break;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  vTaskDelete(NULL);
}

void taskControl(void* pvParameter) {
  xTaskCreate(&taskFlash, "task_flash", 4096, NULL, 2, NULL);

  Data data;
  TickType_t init_loop_time;
  float t0 = micros()/1e6;
  float t = 0;

  while(1) {
    init_loop_time = xTaskGetTickCount();
    t = micros()/1e6 - t0;

    if (t > 30) {
      motor(0, L_CHANNEL);
      motor(0, R_CHANNEL);

      startMode = CONFIG;
      digitalWrite(2, HIGH);

      printf("End Control\n");
      printf("Configuration Mode (l:list, f:format, R:read only, r:read and format)\n");
      break;
    } else if (t > 25) {
      ref_dx = 0;
    } else if (t > 20) {
      ref_psi = PI/2;
    } else if (t > 15) {
      ref_dx = 0.3;
    } else {
      ref_psi = 0;
      ref_dx = 0;
    }

    position_r = 2*PI*encoder_r.getCount()/(float)(4*7*30);
    position_l = 2*PI*encoder_l.getCount()/(float)(4*7*30);
    x = r_w * (position_l - position_r)/2;

    mpu.getEvent(&a, &g, &temp);
    g_x = -g.gyro.x;
    g_y = g.gyro.y - gy_offset;
    a_pitch = atan2(a.acceleration.z, a.acceleration.y);

    // direita
    dwheel_f_r = alpha_w*dwheel_f_r + (1-alpha_w)*(position_r - last_position_r)/ts;
    // esquerda
    dwheel_f_l = alpha_w*dwheel_f_l + (1-alpha_w)*(position_l - last_position_l)/ts;
    dx = r_w * (dwheel_f_l - dwheel_f_r)/2;

    // corpo
    theta = kalman_theta.getAngle(a_pitch, g_x);
    psi += g_y*ts;
    // printf("%.04f,%.04f\n", psi, -r_w * (position_l + position_r)/w);

    q[0] = theta;
    q[1] = dx;
    q[2] = (theta - last_theta)/ts;
    error_dx_int += (ref_dx - dx)*ts;

    // psi
    ref_psi_f = 0.99*ref_psi_f + 0.01*ref_psi;
    error_psi = ref_psi_f - psi;
    error_psi_int += error_psi*ts;

    V = -(K[0]*q[0] + K[1]*q[1] + K[2]*q[2]) - Ki*error_dx_int;
    DeltaV = -3*(error_psi + 2*error_psi_int);
    Vl = (V + DeltaV); 
    Vr = -(V - DeltaV);

    if (abs(theta) >= PI/4) {
      error_dx_int = 0;
      error_psi_int = 0;
      Vr = 0; Vl = 0;
      psi = 0;
    }
    pwm_value = mapfloat(Vr, -7.4, 7.4, -100, 100);  
    motor(pwm_value, R_CHANNEL);
    pwm_value = mapfloat(Vl, -7.4, 7.4, -100, 100);
    motor(pwm_value, L_CHANNEL);

    data.t = t;
    data.dx = dx;
    data.theta = theta;
    data.psi = psi;
    data.u = V;
    data.ref_psi = ref_psi;
    data.ref_dx = ref_dx;
    data_array[count % BUFFER_SIZE] = data;

    last_theta = theta;
    last_position_r = position_r;
    last_position_l = position_l;

    count++;
    vTaskDelayUntil(&init_loop_time, pdMS_TO_TICKS(ts*1000));
  }

  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(921600);
  PinSetup();
  Wire.begin();
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
  delay(100);

  data_array = (Data*)heap_caps_calloc(BUFFER_SIZE, sizeof(Data), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
  if (data_array == NULL) {
    Serial.println("Malloc for 'data_array' has failed!");
    return;
  }

  flash_count = 0;
  file_count = 0;
  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)) {
    Serial.println("Falha ao montar SPIFFS");
    return;
  }
  file_count = fileCountInit();

  xTaskCreate(&taskBlinkLed, "blink_led", 2048, NULL, 1, NULL);
  while (startMode == INIT) delay(100);
  if (startMode == RUN) {
    digitalWrite(2, LOW);
    delay(2000);
    printf("Running Mode...\n");
    xTaskCreate(&taskControl, "task_control", 4096, NULL, 10, NULL);
  } else if (startMode == CONFIG) {
    printf("Configuration Mode (l:list, f:format, R:read only, r:read and format)\n");
  }
}

void loop() {
  if (Serial.available() >= 1) {
    char command = Serial.read();

    if (command == 'l') {
      listDir(SPIFFS, "/", 0);
    }

    if (command == 'f') {
      SPIFFS.format();
      Serial.println("Memória formatada");
    }
    
    if (command == 'r' || command == 'R') {
      File root = SPIFFS.open("/");
      if(!root){
        Serial.println("- failed to open directory");
        return;
      }
      if(!root.isDirectory()){
        Serial.println(" - not a directory");
        return;
      }

      Data data_read;
      File file = root.openNextFile();
      while(file){
        if (!file.isDirectory()) {
          String filename = "/" + String(file.name());

          File file = SPIFFS.open(filename.c_str());
          
          if(!file || file.isDirectory()){
            Serial.println("- failed to open file for reading");
            break;
          }
          if (!filename.equals(String(FILE_COUNT_BKP))) {
            Serial.println("Reading: " + filename);

            Serial.println("- read from file:");
            Serial.println("t;u;ref_dx;ref_psi;dx;theta;psi");
            while(file.available()){
              // Serial.write(file.read());
              if (file.read((uint8_t*)&data_read, sizeof(Data)) != sizeof(Data)) {
                break;
              }
              sprintf(buffer, "%.4f;%d;%.4f;%.4f;%.4f;%.4f;%.4f\n", 
                data_read.t, data_read.u, data_read.ref_dx, data_read.ref_psi, data_read.dx, data_read.theta, data_read.psi);
              Serial.write(buffer);
            }
          }
          file.close();
        }
        file = root.openNextFile();
      }

      if (command == 'r') {
        SPIFFS.format();
        Serial.println("Memória formatada");
      }
    }
  }
}

void PinSetup() {
  // Configuração dos pinos
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Configuração do PWM
  ledcSetup(L_CHANNEL, 50000, 8);
  ledcSetup(R_CHANNEL, 50000, 8);
  ledcAttachPin(ENA, L_CHANNEL);
  ledcAttachPin(ENB, R_CHANNEL);
  ledcWrite(L_CHANNEL, 0);
  ledcWrite(R_CHANNEL, 0);

  // Configuração dos encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
	encoder_r.attachFullQuad(ENCODER_C1_R, ENCODER_C2_R);
	encoder_l.attachFullQuad(ENCODER_C1_L, ENCODER_C2_L);
  encoder_r.clearCount();
  encoder_l.clearCount();

  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
}

void taskBlinkLed(void* pvParameter) 
{
  pinMode(2, OUTPUT);
  pinMode(0, INPUT);

  unsigned long init = millis();
  bool state = true;
  startMode = INIT;

  while (millis() - init <= 5000) {
    digitalWrite(2, state);
    state = !state;

    if (digitalRead(0) == LOW) {
      startMode = RUN;
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  if (startMode == INIT) startMode = CONFIG;

  vTaskDelete(NULL);
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels)
{
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("- failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.path(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

int fileCountInit() 
{
  int file_count = 0;

  File file = SPIFFS.open(FILE_COUNT_BKP, FILE_READ);
  if(!file || file.isDirectory()){
    file_count = 0;
  }
  while(file.available()){
    file.read((uint8_t*)&file_count, sizeof(file_count));
  }
  file.close();

  Serial.println("File count init = " + String(file_count));

  return file_count;
}