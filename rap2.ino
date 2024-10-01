const int ain1 = 7;
const int ain2 = 16;
const int pwma = 10;
const int bin1 = 14;
const int bin2 = 15;
const int pwmb = 5;
const int led = 17;
const int button = 2;
const float Kp = 2.0;
const float Ki = 0.0;
const float Kd = 1.0;
const int irPins[8] = {9, 8, 7, A0, A1, 6, A2, A3};
const int weights[8] = {-64, -32, -16, -8, 8, 16, 32, 64};
const int speed = 25;          //[0-255]
const int motorDerivative = 0; // -100 (right) to 100 (left)
const float correctionFactor = 3.0;

float normalizedValues[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
int irSensor[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int calibrationMin[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
int calibrationMax[8] = {0, 0, 0, 0, 0, 0, 0, 0};
bool calibrated = false;
boolean motorStop = false;
float position = 0.0;
float integral = 0;
float lastError = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(button, INPUT);
  pinMode(led, OUTPUT);
  for (int i = 0; i < 8; i++)
  {
    pinMode(irPins[i], INPUT);
  }
}

void loop()
{

  if (digitalRead(button))
  {
    if (!calibrated)
    {
      calibrate();
    }
    else if (!motorStop)
    {
      motorStop = true;
      drive(0, 0);
      delay(1000);
    }
    else
    {
      motorStop = false;
      delay(2000);
    }
  }

  if (calibrated && !motorStop)
  {
    digitalWrite(led, LOW);
    calculatePosition();
    updateMotors();
  }
  else
  {
    digitalWrite(led, HIGH);
  }
}

void calculatePosition()
{
  float positionSum = 0.0;
  float valueSum = 0.0;

  for (int i = 0; i < 8; i++)
  {
    irSensor[i] = analogRead(irPins[i]);

    if (calibrationMax[i] > calibrationMin[i])
    {
      normalizedValues[i] = (float)(irSensor[i] - calibrationMin[i]) / (calibrationMax[i] - calibrationMin[i]);
      if (normalizedValues[i] < 0)
        normalizedValues[i] = 0;
      if (normalizedValues[i] > 1)
        normalizedValues[i] = 1;
    }

    positionSum += normalizedValues[i] * weights[i];
    valueSum += normalizedValues[i];
  }

  position = (valueSum != 0) ? (float)positionSum / valueSum : 0;

  if (Serial)
  {
    Serial.print("Values: ");
    for (int i = 0; i < 8; i++)
    {
      Serial.print(normalizedValues[i], 3);
      if (i < 7)
        Serial.print("\t");
    }
    Serial.print(" | Position: ");
    Serial.println(position);
  }
}

void updateMotors()
{
  float error = position;
  float P = Kp * error;
  integral += error;
  float I = Ki * integral;
  float derivative = error - lastError;
  float D = Kd * derivative;
  float correction = (P + I + D) * correctionFactor;
  int leftSpeed = constrain(255 - correction, -255, 255);
  int rightSpeed = constrain(255 + correction, -255, 255);
  drive(leftSpeed, rightSpeed);

  if (Serial)
  {
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" | P: ");
    Serial.print(P);
    Serial.print(" | I: ");
    Serial.print(I);
    Serial.print(" | D: ");
    Serial.print(D);
    Serial.print(" | Correction: ");
    Serial.print(correction);
    Serial.print(" | Left Speed: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right Speed: ");
    Serial.println(rightSpeed);
  }

  lastError = error;
}

void calibrate()
{
  if (Serial)
  {
    Serial.println("Starting calibration");
  }
  unsigned long startTime = millis();
  unsigned long blinkTime = millis();
  bool ledState = false;
  while (millis() - startTime < 10000)
  {
    if (millis() - blinkTime >= 500)
    {
      ledState = !ledState;
      digitalWrite(led, ledState);
      blinkTime = millis();
    }
    for (int i = 0; i < 8; i++)
    {
      irSensor[i] = analogRead(irPins[i]);
      if (irSensor[i] > calibrationMax[i])
        calibrationMax[i] = irSensor[i];
      if (irSensor[i] < calibrationMin[i])
        calibrationMin[i] = irSensor[i];
    }
  }
  if (Serial)
  {
    Serial.println("Calibration finished");
    Serial.print("Calibration Min: ");
    for (int i = 0; i < 8; i++)
    {
      Serial.print(calibrationMin[i]);
      if (i < 7)
        Serial.print("\t");
    }
    Serial.println();
    Serial.print("Calibration Max: ");
    for (int i = 0; i < 8; i++)
    {
      Serial.print(calibrationMax[i]);
      if (i < 7)
        Serial.print("\t");
    }
    Serial.println();
  }
  calibrated = true;
}

void drive(int speedl, int speedr)
{
  if (speedl != 0)
  {
    speedl = constrain(speedl + motorDerivative, -speed, speed);
  }

  if (speedr != 0)
  {
    speedr = constrain(speedr - motorDerivative, -speed, speed);
  }

  digitalWrite(ain1, speedl >= 0);
  digitalWrite(ain2, speedl < 0);
  analogWrite(pwma, abs(speedl));

  digitalWrite(bin1, speedr >= 0);
  digitalWrite(bin2, speedr < 0);
  analogWrite(pwmb, abs(speedr));
}
