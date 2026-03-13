


//  IZQUIERDO visto desde adelante
const int DIR_LEFT = 7;
const int PWMA = 5;

//  DERECHO visto desde adelante
const int DIR_RIGHT = 8; 
const int PWMB = 6;


// Very important for TB6612
const int STBY = 3;

const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = false;

const int MAX_THRUST = 512;



String inputLine = "";
unsigned long lastCommandTime = 0;
const unsigned long COMMAND_TIMEOUT_MS = 100;

void setup() {
  Serial.begin(115200);

  pinMode(DIR_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);


  pinMode(PWMA, OUTPUT);
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

  // if (millis() - lastCommandTime > COMMAND_TIMEOUT_MS) {
  //   stopMotors();
  // }
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

  leftSpeed = constrain(leftSpeed, -MAX_THRUST, MAX_THRUST);
  rightSpeed = constrain(rightSpeed, -MAX_THRUST, MAX_THRUST);

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
  int pwm = constrain(abs(speedValue), 0, MAX_THRUST);
  bool forward = (speedValue >= 0);
  if (INVERT_LEFT_MOTOR) forward = !forward;

  if (pwm == 0) { 
    analogWrite(PWMB, 0); 
    return; 
  }

  digitalWrite(DIR_RIGHT, forward ? HIGH : LOW);
  analogWrite(PWMB, pwm);
}


void setRightMotor(int speedValue) {
  int pwm = constrain(abs(speedValue), 0, MAX_THRUST);
  bool forward = (speedValue >= 0);
  if (INVERT_RIGHT_MOTOR) forward = !forward;

  if (pwm == 0) { analogWrite(PWMA, 0); return; }

  digitalWrite(DIR_LEFT, forward ? HIGH : LOW);
  analogWrite(PWMA, pwm);
}


void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}