// === Pin Definitions ===
const int ENA = 10; // PWM pin for speed control
const int IN1 = 8;  // Motor direction input 1
const int IN2 = 9;  // Motor direction input 2

// Pendulum Angle Encoder (Theta)
const int ENCODER_THETA_A = 2; // Interrupt Pin 0
const int ENCODER_THETA_B = 5;

// Wheel Position Encoder (Position)
const int ENCODER_POS_A = 3; // Interrupt Pin 1
const int ENCODER_POS_B = 4;

// === Physical Constants ===
const float WHEEL_DIAMETER = 7.0; // cm
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI; // cm
const int MOTOR_ENCODER_PPR = 600;
const int GEAR_RATIO = 50;
const int PENDULUM_ENCODER_PPR = 600;
const float DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / (MOTOR_ENCODER_PPR * GEAR_RATIO);

// === PID Controller Tuning ===
double Kp = 35.0;
double Ki = 0.8;
double Kd = 3.0;

double Kp_pos = 0.1;
double Ki_pos = 0.0;
double Kd_pos = 0.05;

// === Global Variables ===
volatile long theta_pulse_count = 0;
volatile long pos_pulse_count = 0;

double setpoint_angle = 0.0;
double current_angle = 0.0;
double error = 0.0;
double last_error = 0.0;
double integral = 0.0;
double derivative = 0.0;
double pid_output = 0.0;

double setpoint_pos = 0.0;
double current_pos_cm = 0.0;
double pos_error = 0.0;
double last_pos_error = 0.0;
double pos_integral = 0.0;
double pos_derivative = 0.0;
double pos_pid_output = 0.0;

unsigned long last_time = 0;
bool swingUpDone = false;

void setup() {
  Serial.begin(115200);
  Serial.println("Inverted Pendulum Initialization...");
  Serial.println("Hold pendulum downward and reset Arduino to calibrate.");

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENCODER_THETA_A, INPUT_PULLUP);
  pinMode(ENCODER_THETA_B, INPUT_PULLUP);
  pinMode(ENCODER_POS_A, INPUT_PULLUP);
  pinMode(ENCODER_POS_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_THETA_A), readAngleEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_POS_A), readPositionEncoder, RISING);

  last_time = millis();

  delay(1000);
  Serial.println("Initialization Complete. Starting control loop.");
}

void loop() {
  unsigned long current_time = millis();
  double dt = (double)(current_time - last_time) / 1000.0;
  last_time = current_time;

  // Update angle and position
  current_angle = (theta_pulse_count * 360.0) / (PENDULUM_ENCODER_PPR * 4.0);
  current_pos_cm = pos_pulse_count * DISTANCE_PER_PULSE; // displacement from mean

  // === SWING-UP LOGIC ===
  if (!swingUpDone) {
    // Swing up until angle is close to upright (within ±10 degrees of 180)
    if (abs(current_angle - 180.0) > 10.0) {
      int swing_direction = (current_angle < 180) ? 1 : -1;
      setMotorSpeed(120 * swing_direction);
      Serial.print("Swinging up | Angle: ");
      Serial.println(current_angle);
    } else {
      swingUpDone = true;
      theta_pulse_count = 0; // reset encoder for upright reference
      pos_pulse_count = 0;   // reset position
      Serial.println("Swing-up complete. Entering balance mode.");
    }
    delay(10);
    return;
  }

  // === CONTROL LOOP ===
  error = setpoint_angle - current_angle;

  if (abs(error) < 20.0) {
    integral += error * dt;
    derivative = (error - last_error) / dt;
    pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    last_error = error;

    // === Position Controller with BOUNDARY CHECK ===
    if (abs(current_pos_cm) > 30.0) {
      // Beyond 30cm limit, reverse slightly
      int correction = (current_pos_cm > 0) ? -100 : 100;
      setMotorSpeed(correction);
      Serial.println("OUT OF BOUNDS - Correcting position.");
      return;
    }

    pos_error = setpoint_pos - current_pos_cm;
    pos_integral += pos_error * dt;
    pos_derivative = (pos_error - last_pos_error) / dt;
    pos_pid_output = (Kp_pos * pos_error) + (Ki_pos * pos_integral) + (Kd_pos * pos_derivative);
    last_pos_error = pos_error;

    double final_output = pid_output + pos_pid_output;
    final_output = constrain(final_output, -255, 255);
    setMotorSpeed(final_output);
  } else {
    // Out of control range
    setMotorSpeed(0);
    integral = 0;
    last_error = 0;
    pos_integral = 0;
    last_pos_error = 0;
  }

  // === DEBUG OUTPUT ===
  Serial.print("Angle: ");
  Serial.print(current_angle);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | Displacement (cm): ");
  Serial.print(current_pos_cm);
  Serial.print(" | Pulse Count: ");
  Serial.print(pos_pulse_count);
  Serial.print(" | Final Out: ");
  Serial.println(pid_output + pos_pid_output);

  delay(10);
}

void setMotorSpeed(int speed) {
  int motor_speed = constrain(abs(speed), 0, 255);

  if (speed > 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, motor_speed);
  } else if (speed < 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, motor_speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }
}

void readAngleEncoder() {
  if (digitalRead(ENCODER_THETA_B) == HIGH) {
    theta_pulse_count++;
  } else {
    theta_pulse_count--;
  }
}

void readPositionEncoder() {
  // Phase-based quadrature decoding
  bool A = digitalRead(ENCODER_POS_A);
  bool B = digitalRead(ENCODER_POS_B);

  if (A == B) {
    pos_pulse_count++;
  } else {
    pos_pulse_count--;
  }
}
