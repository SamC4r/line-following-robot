/*
  ELEGOO Smart Robot Car V4.0
  UNO R3 + SmartCar Shield V1.1
  USB serial motor control for TB6612-based shield

  Commands:
    M,left,right

  Examples:
    M,180,180
    M,220,120
    M,0,0
*/

const int AIN1 = 7;
const int AIN2 = 8;
const int PWMA = 5;

const int BIN1 = 9;
const int BIN2 = 10;
const int PWMB = 6;

// Very important for TB6612
const int STBY = 3;

// Flip these if one side runs backward
const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = false;

String inputLine = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 500;

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Wake up TB6612
  digitalWrite(STBY, HIGH);

  stopMotors();
  lastCommandTime = millis();

  Serial.println("READY");
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      inputLine.trim();

      if (inputLine.length() > 0) {
        handleCommand(inputLine);
        lastCommandTime = millis();

        digitalWrite(LED_BUILTIN, HIGH);
        delay(10);
        digitalWrite(LED_BUILTIN, LOW);
      }

      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }

  if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
    stopMotors();
  }
}

void handleCommand(String cmd) {
  if (!cmd.startsWith("M,")) {
    Serial.print("IGNORED: ");
    Serial.println(cmd);
    return;
  }

  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);

  if (firstComma < 0 || secondComma < 0) {
    Serial.print("BAD CMD: ");
    Serial.println(cmd);
    return;
  }

  int leftSpeed = cmd.substring(firstComma + 1, secondComma).toInt();
  int rightSpeed = cmd.substring(secondComma + 1).toInt();

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  Serial.print("LEFT=");
  Serial.print(leftSpeed);
  Serial.print(" RIGHT=");
  Serial.println(rightSpeed);

  setMotorSpeeds(leftSpeed, rightSpeed);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

void setLeftMotor(int speedValue) {
  int pwm = constrain(abs(speedValue), 0, 255);
  bool forward = (speedValue >= 0);

  if (INVERT_LEFT_MOTOR) {
    forward = !forward;
  }

  if (pwm == 0) {
    analogWrite(PWMA, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    return;
  }

  if (forward) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  analogWrite(PWMA, pwm);
}

void setRightMotor(int speedValue) {
  int pwm = constrain(abs(speedValue), 0, 255);
  bool forward = (speedValue >= 0);

  if (INVERT_RIGHT_MOTOR) {
    forward = !forward;
  }

  if (pwm == 0) {
    analogWrite(PWMB, 0);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    return;
  }

  if (forward) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  analogWrite(PWMB, pwm);
}

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}