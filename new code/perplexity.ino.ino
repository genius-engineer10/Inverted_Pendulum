// === Pin Definitions ===
const int ENA = 10; // PWM pin for speed control
const int IN1 = 8; // Motor direction input 1
const int IN2 = 9; // Motor direction input 2

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

// === PID Controller Tuning (Balance) ===
double Kp = 35.0;
double Ki = 0.8;
double Kd = 3.0;

// === Position outer-loop PID (for drift correction) ===
double Kp_pos = 0.1;
double Ki_pos = 0.0;
double Kd_pos = 0.05;

// === Swing-up parameters ===
// Simple energy-based/bang-bang approach
bool swingup_enabled = true;
bool balancing_enabled = false;
double capture_angle_deg = 12.0; // when |angle| < this, switch to balancing
double swing_max_pwm = 200; // motor command limit during swing-up
double swing_direction = 1; // toggled by energy phase
double angle_for_phase_deg = 15.0; // angle magnitude defining "near bottom" region
unsigned long swing_last_toggle_ms = 0;
unsigned long swing_toggle_min_period_ms = 250; // min time between direction flips

// === Soft position limits and track ===
const float TRACK_HALF_CM = 50.0; // half of 100 cm track (for context)
const float DISP_LIMIT_CM = 30.0; // ±30 cm soft limit from mean
const double limit_kp = 4.0; // push-back proportional gain when beyond limit
const int limit_max_pwm = 220; // clamp for limit correction

// === Global Variables ===
volatile long theta_pulse_count = 0;
volatile long pos_pulse_count = 0;

long pos_zero_count = 0; // reference count to make displacement from mean

double setpoint_angle = 0.0; // upright
double current_angle = 0.0;
double error = 0.0;
double last_error = 0.0;
double integral = 0.0;
double derivative = 0.0;
double pid_output = 0.0;

double setpoint_pos = 0.0; // desired displacement (cm) from mean
double displacement_cm = 0.0; // displacement from mean (cm)
double pos_error = 0.0;
double last_pos_error = 0.0;
double pos_integral = 0.0;
double pos_derivative = 0.0;
double pos_pid_output = 0.0;

unsigned long last_time = 0;

void setup() {
Serial.begin(115200);
Serial.println("Inverted Pendulum Initialization...");
Serial.println("Hold pendulum upright and reset Arduino to calibrate.");

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
delay(700);

// Set current wheel position as the mean (zero displacement)
noInterrupts();
pos_zero_count = pos_pulse_count;
interrupts();

Serial.println("Initialization Complete. Starting control loop.");
}

void loop() {
unsigned long current_time = millis();
double dt = (double)(current_time - last_time) / 1000.0;
if (dt <= 0) dt = 0.001;
last_time = current_time;

// Read sensors
long theta_count_local;
long pos_count_local;
noInterrupts();
theta_count_local = theta_pulse_count;
pos_count_local = pos_pulse_count;
interrupts();

// Angle in degrees (quadrature x4)
current_angle = (theta_count_local * 360.0) / (PENDULUM_ENCODER_PPR * 4.0);

// Displacement from mean (use zeroed reference)
long disp_counts = pos_count_local - pos_zero_count;
displacement_cm = disp_counts * DISTANCE_PER_PULSE;

// Operating mode: swing-up first, then switch to balance when captured
int motor_cmd = 0;

if (swingup_enabled && !balancing_enabled) {
// Swing-up controller


// Simple bang-bang based on pendulum phase:
// When pendulum passes near the bottom region, toggle drive direction
// to inject energy. Limit toggling rate to avoid chatter.
double angle_abs = fabs(current_angle);

if (angle_abs < angle_for_phase_deg) {
  if (current_time - swing_last_toggle_ms > swing_toggle_min_period_ms) {
    swing_direction = -swing_direction;
    swing_last_toggle_ms = current_time;
  }
}

// Push the cart in the chosen direction
double swing_output = swing_direction * swing_max_pwm;

// Apply soft limits to avoid running off track ends
swing_output += limitCorrection(displacement_cm);

// If we are close enough to upright, transition to balance
if (angle_abs < capture_angle_deg) {
  balancing_enabled = true;
  swingup_enabled = false;
  // reset PID terms
  integral = 0;
  last_error = 0;
  pos_integral = 0;
  last_pos_error = 0;
  Serial.println("Captured. Switching to balance mode.");
}

motor_cmd = (int)constrain(swing_output, -255, 255);
setMotorSpeed(motor_cmd);

} else {
// Balance controller (PID on angle) with optional position outer loop

error = setpoint_angle - current_angle;

// Only integrate near upright to avoid windup
if (fabs(error) < 30.0) {
  integral += error * dt;
  derivative = (error - last_error) / dt;
  pid_output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  last_error = error;
} else {
  // If we fall too far, stop and re-enter swing-up
  pid_output = 0;
  integral = 0;
  last_error = 0;
  pos_integral = 0;
  last_pos_error = 0;
  balancing_enabled = false;
  swingup_enabled = true;
  setMotorSpeed(0);
  Serial.println("Lost balance. Returning to swing-up.");
  delay(10);
  printTelemetry(pid_output, 0);
  return;
}

// Outer-loop position correction to keep near 0 displacement
// Only act when displacement is outside a small deadband to avoid chattering
double deadband_cm = 2.0;
if (fabs(displacement_cm - setpoint_pos) > deadband_cm) {
  pos_error = setpoint_pos - displacement_cm;
  pos_integral += pos_error * dt;
  pos_derivative = (pos_error - last_pos_error) / dt;
  pos_pid_output = (Kp_pos * pos_error) + (Ki_pos * pos_integral) + (Kd_pos * pos_derivative);
  last_pos_error = pos_error;
} else {
  pos_integral = 0;
  pos_pid_output = 0;
  last_pos_error = 0;
}

// Soft limit correction when beyond ±DISP_LIMIT_CM
double limit_out = limitCorrection(displacement_cm);

// Combine
double final_output = pid_output + pos_pid_output + limit_out;

// Clamp and command
final_output = constrain(final_output, -255, 255);
motor_cmd = (int)final_output;
setMotorSpeed(motor_cmd);

}

// Telemetry
printTelemetry(pid_output, pos_pid_output);

delay(10);
}

double limitCorrection(double disp_cm) {
// If displacement beyond ±DISP_LIMIT_CM, push back proportionally
double corr = 0.0;
if (disp_cm > DISP_LIMIT_CM) {
corr = -limit_kp * (disp_cm - DISP_LIMIT_CM); // drive back toward center
} else if (disp_cm < -DISP_LIMIT_CM) {
corr = limit_kp * (-DISP_LIMIT_CM - disp_cm); // drive forward toward center
}
// clamp correction
if (corr > limit_max_pwm) corr = limit_max_pwm;
if (corr < -limit_max_pwm) corr = -limit_max_pwm;
return corr;
}

void printTelemetry(double angle_pid, double pos_pid) {
Serial.print("Angle(deg): ");
Serial.print(current_angle, 2);
Serial.print(" | Disp(cm): ");
Serial.print(displacement_cm, 2);
Serial.print(" | AngleErr: ");
Serial.print(setpoint_angle - current_angle, 2);
Serial.print(" | PosErr: ");
Serial.print(setpoint_pos - displacement_cm, 2);
Serial.print(" | AnglePID: ");
Serial.print(angle_pid, 2);
Serial.print(" | PosPID: ");
Serial.print(pos_pid, 2);
Serial.print(" | Mode: ");
Serial.println(balancing_enabled ? "BAL" : "SWING");
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