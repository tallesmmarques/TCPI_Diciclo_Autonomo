#include <stdio.h>
#include <ESP32Encoder.h>

#define ENCODER_C1_R 32
#define ENCODER_C2_R 33 
#define ENCODER_C1_L 14 
#define ENCODER_C2_L 27
#define IN1 16
#define IN2 17
#define ENA 4
#define IN3 18
#define IN4 19
#define ENB 23
#define L_CHANNEL 0
#define R_CHANNEL 1

#define SPEED_TIME 0.500
#define SPEED_SAMPLE 10

void pinSetup();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void motor(int PWM, int chanel);

ESP32Encoder encoder_r;
ESP32Encoder encoder_l;

void setup() {
  Serial.begin(115200);
  printf("Serial Init\n");
  pinSetup();

  delay(2000);
  digitalWrite(2, LOW);

  const float ts = 10e-3; 
  double speed_l, speed_r;
  int64_t last_count_l, last_count_r; 
  int64_t current_count_l, current_count_r;

  for (int pwm=-100; pwm <= 100; pwm += 10) {
    motor(pwm, L_CHANNEL);
    motor(pwm, R_CHANNEL);
    speed_l = 0;
    speed_r = 0;
    for (int i=0; i<SPEED_SAMPLE; i++) {
      current_count_l = encoder_l.getCount();
      current_count_r = encoder_r.getCount();
      speed_l = (current_count_l - last_count_l)/ts;
      speed_r = (current_count_r - last_count_r)/ts;
      delay(10);
    }
    printf("PWM: %d, Speed Left (RPM): %03.04f, Speed Rigth (RPM): %03.04f\n", pwm, speed_l, speed_r);

    motor(0, L_CHANNEL);
    motor(0, R_CHANNEL);
    delay(500);
  }
  
  motor(0, L_CHANNEL);
  motor(0, R_CHANNEL);
}

void loop() {
  digitalWrite(2, HIGH);
  delay(1000);
  digitalWrite(2, LOW);
  delay(1000);
}

void pinSetup() {
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
  digitalWrite(2, HIGH);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void motor(int PWM, int chanel) {
  int p1, p2;
  if (chanel == L_CHANNEL) {
    p1 = IN1;
    p2 = IN2;
  } else {
    p1 = IN3;
    p2 = IN4;
  }
  bool PWM_Zero = PWM == 0 ? true : false;
  PWM = constrain(PWM, -100, 100);
  // PWM = (PWM >= 0 ? 1 : -1) * mapfloat(abs(PWM), 0, 100, MIN_PWM, 100);
  PWM = map(PWM, -100, 100, -255, 255);
  if (PWM > 0) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, HIGH);
  } else if (PWM < 0) {
    PWM = -PWM;
    digitalWrite(p1, HIGH);
    digitalWrite(p2, LOW);
  } else if (PWM_Zero) {
    digitalWrite(p1, LOW);
    digitalWrite(p2, LOW);
    PWM = 0;
  }
  ledcWrite(chanel, PWM);
}