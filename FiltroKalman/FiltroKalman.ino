#include <Wire.h>
#include <MPU6050.h>
#include <stdio.h>

// Cria um objeto MPU6050
MPU6050 mpu;

// Configura parâmetros do filtro de Kalman
const float dt = 0.01; // Intervalo de tempo (ajuste conforme necessário)
float angleX = 0;
float angleY = 0;

// Classe para o filtro de Kalman
class Kalman {
  public:
    Kalman() {
      angle = 0;
      bias = 0;
      P[0][0] = 1;
      P[0][1] = 0;
      P[1][0] = 0;
      P[1][1] = 1;
    }

    float getAngle(float newAngle, float newRate) {
      float rate = newRate - bias;
      angle += dt * rate;

      P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      float S = P[0][0] + R_measure;
      float K[2];
      K[0] = P[0][0] / S;
      K[1] = P[1][0] / S;

      float y = newAngle - angle;
      angle += K[0] * y;
      bias += K[1] * y;

      float P00_temp = P[0][0];
      float P01_temp = P[0][1];

      P[0][0] -= K[0] * P00_temp;
      P[0][1] -= K[0] * P01_temp;
      P[1][0] -= K[1] * P00_temp;
      P[1][1] -= K[1] * P01_temp;

      return angle;
    }

  private:
    float angle;
    float bias;
    float P[2][2];
    float Q_angle = 0.001;
    float Q_bias = 0.003;
    float R_measure = 0.03;
};

Kalman kalmanX;
Kalman kalmanY;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  mpu.initialize();
  
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }
}

void loop() {
  // Leitura dos dados do MPU6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Converte os dados para valores mais apropriados
  float accelAngleX = atan2(ay, az);
  float accelAngleY = atan2(ax, az);

  // Ajusta os valores de giroscópio para graus por segundo
  float gyroX = gx / 65.5; // Ajuste o divisor conforme necessário
  float gyroY = gy / 65.5; // Ajuste o divisor conforme necessário

  // Aplica o filtro de Kalman
  angleX = kalmanX.getAngle(accelAngleX, gyroX);
  angleY = kalmanY.getAngle(accelAngleY, gyroY);
  
  // Serial.print("l:-180,L:180,Angle_X:");
  // Serial.print(angleX);
  // Serial.print(",Angle_Y:");
  // Serial.println(angleY);
  printf("l:-3.14,L:3.14,Angle_X:%.04f,Angle_Y:%.04f\n", angleX, angleY);

  delay(10); // Ajuste o delay conforme necessário
}
