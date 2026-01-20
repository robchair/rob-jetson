// control_chair_bts7960.ino
// Compatible with BTS7960 drivers and your Jetson serial protocol.

// -------- PIN DEFINITIONS --------
// Left motor BTS7960
const int L_RPWM = 5;   // Left board RPWM  (PWM)
const int L_LPWM = 6;   // Left board LPWM  (PWM)

// Right motor BTS7960
const int R_RPWM = 9;   // Right board RPWM (PWM)
const int R_LPWM = 10;  // Right board LPWM (PWM)

// Optional: if you wired R_EN / L_EN to pins instead of 5V,
// define them here and set them HIGH in setup().
// Otherwise, just tie them to 5V and ignore in code.
const int L_REN = -1;   // set to real pin if used, otherwise -1
const int L_LEN = -1;
const int R_REN = -1;
const int R_LEN = -1;

// ---- SPEED LIMITS ----
const int MAX_SPEED = 255;    // max PWM
const int FWD_SPEED = 150;    // nominal forward speed
const int TURN_SPEED = 130;   // turning speed
const int BACK_SPEED = 130;   // backward speed

// -------- MOTOR CONTROL HELPERS --------

// Clamp value into [-MAX_SPEED, MAX_SPEED]
int clampSpeed(int s) {
  if (s > MAX_SPEED) return MAX_SPEED;
  if (s < -MAX_SPEED) return -MAX_SPEED;
  return s;
}

// Drive one BTS7960-motor pair with a signed speed:
//   speed > 0  -> forward (RPWM = speed, LPWM = 0)
//   speed < 0  -> backward (RPWM = 0, LPWM = -speed)
//   speed = 0  -> stop (both 0)
void driveBTS7960(int rpwmPin, int lpwmPin, int speed) {
  speed = clampSpeed(speed);

  if (speed > 0) {
    analogWrite(rpwmPin, speed);
    analogWrite(lpwmPin, 0);
  } else if (speed < 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -speed);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

// Set both motors at once
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  driveBTS7960(L_RPWM, L_LPWM, leftSpeed);
  driveBTS7960(R_RPWM, R_LPWM, rightSpeed);
}

// High-level motion commands
void stopMotors() {
  setMotorSpeeds(0, 0);
}

void forward() {
  setMotorSpeeds(FWD_SPEED, -FWD_SPEED);
}

void backward() {
  setMotorSpeeds(-BACK_SPEED, BACK_SPEED);
}

void turnLeft() {
  // left motor stopped, right motor forward
  setMotorSpeeds(0, -TURN_SPEED);
}

void turnRight() {
  // right motor stopped, left motor forward
  setMotorSpeeds(TURN_SPEED, 0);
}

// Optional “kill”/disable semantics – here we just stop.
// If you wired EN pins to Arduino, you could pull them LOW here.
void quitDrive() {
  stopMotors();
  // Example if EN pins are used:
  // if (L_REN >= 0) digitalWrite(L_REN, LOW);
  // if (L_LEN >= 0) digitalWrite(L_LEN, LOW);
  // if (R_REN >= 0) digitalWrite(R_REN, LOW);
  // if (R_LEN >= 0) digitalWrite(R_LEN, LOW);
}

// -------- ARDUINO SETUP/LOOP --------

void setup() {
  Serial.begin(9600);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);

  if (L_REN >= 0) {
    pinMode(L_REN, OUTPUT);
    digitalWrite(L_REN, HIGH);
  }
  if (L_LEN >= 0) {
    pinMode(L_LEN, OUTPUT);
    digitalWrite(L_LEN, HIGH);
  }
  if (R_REN >= 0) {
    pinMode(R_REN, OUTPUT);
    digitalWrite(R_REN, HIGH);
  }
  if (R_LEN >= 0) {
    pinMode(R_LEN, OUTPUT);
    digitalWrite(R_LEN, HIGH);
  }

  stopMotors();
}


void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "forward") {
      forward();
    } else if (command == "backward") {
      backward();
    } else if (command == "turnLeft") {
      turnLeft();
    } else if (command == "turnRight") {
      turnRight();
    } else if (command == "stop") {
      stopMotors();
    } else if (command == "quit") {
      quitDrive();
    }

    // Optional: echo for debugging
    // Serial.print("Received: ");
    // Serial.println(command);
  }
}

