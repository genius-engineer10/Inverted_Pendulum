// === Motor Driver Pins ===
#define ENA 10
#define IN1 8
#define IN2 9

// === Pendulum Rod Encoder (600 PPR) ===
#define ENCODER_THETA_A 2  // INT0
#define ENCODER_THETA_B 5

// === Motor Encoder Pins (N20) ===
#define ENCODER_POS_A 3  // INT1
#define ENCODER_POS_B 4

// === Global Variables ===
volatile long encoder_count_theta = 0;
volatile long encoder_count_pos = 0;

// === Constants ===
const int PPR = 600;                      // Pulses per revolution
const float gear_ratio = 30.0;           // Gear ratio of motor
const float wheel_diameter_cm = 6.5;
const float wheel_circumference = PI * wheel_diameter_cm;

// === PID Constants ===
float Kp_theta = -220;
float Ki_theta = -3.5;
float Kd_theta = -55;

float Kp_pos = 4.0;
float Ki_pos = 0.0;
float Kd_pos = 0.2;

float theta_target = 0.0;
float cart_position_target = 0.0;

float integral_theta = 0, prev_theta_error = 0;
float integral_pos = 0, prev_pos_error = 0;

unsigned long last_time = 0;

void setup() {
  Serial.begin(9600);

  // Motor Driver Pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Pendulum Encoder Setup
  pinMode(ENCODER_THETA_A, INPUT_PULLUP);
  pinMode(ENCODER_THETA_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_THETA_A), readEncoderTheta, RISING);

  // Motor Encoder Setup
  pinMode(ENCODER_POS_A, INPUT_PULLUP);
  pinMode(ENCODER_POS_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_POS_A), readEncoderPos, RISING);

  last_time = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0;
  if (dt < 0.005) return;  // Limit loop rate
  last_time = now;

  float theta = getTheta();             // Pendulum angle (degrees)
  float cart_pos = getCartPosition();   // Cart position (cm)

  // --- Outer Loop: Cart Position PID ---
  float pos_error = cart_position_target - cart_pos;
  integral_pos += pos_error * dt;
  float d_pos = (pos_error - prev_pos_error) / dt;
  prev_pos_error = pos_error;

  theta_target = Kp_pos * pos_error + Ki_pos * integral_pos + Kd_pos * d_pos;

  // --- Inner Loop: Theta PID ---
  float theta_error = theta_target - theta;
  integral_theta += theta_error * dt;
  float d_theta = (theta_error - prev_theta_error) / dt;
  prev_theta_error = theta_error;

  float output = Kp_theta * theta_error + Ki_theta * integral_theta + Kd_theta * d_theta;

  // --- Safety Mechanism: Return to center if X exceeds limits ---
  if (abs(cart_pos) > 40.0) {
    cart_position_target = 0.0;
  }

  setMotor(output);

  // --- Debug Output ---
  Serial.print("X: ");
  Serial.print(cart_pos, 2);
  Serial.print(" cm | θ: ");
  Serial.print(theta, 2);
  Serial.print("° | Out: ");
  Serial.println(output, 2);

  delay(10);
}

// === Encoder ISR Functions ===
void readEncoderTheta() {
  if (digitalRead(ENCODER_THETA_B)) encoder_count_theta--;
  else encoder_count_theta++;
}

void readEncoderPos() {
  if (digitalRead(ENCODER_POS_B)) encoder_count_pos--;
  else encoder_count_pos++;
}

// === Calculate Pendulum Angle (degrees) ===
float getTheta() {
  float rev = encoder_count_theta / (float)(PPR * gear_ratio);
  float theta_rad = rev * 2 * PI;
  return theta_rad * 180.0 / PI;
}

// === Calculate Cart Position (cm) ===
float getCartPosition() {
  float motor_rev = encoder_count_pos / (float)PPR;
  float wheel_rev = motor_rev / gear_ratio;
  return wheel_rev * wheel_circumference;
}

// === Drive Motor Based on Output Signal ===
void setMotor(float command) {
  int pwm = constrain(abs(command), 0, 255);s

  if (command > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else if (command < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, pwm);
}