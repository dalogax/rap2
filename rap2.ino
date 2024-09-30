#define AIN1 7
#define AIN2 16
#define PWMA 10
#define BIN1 14
#define BIN2 15
#define PWMB 5
#define BUTTON 2

const int irPins[8] = {9, 8, 7, A0, A1, 6, A2, A3};
int irSensor[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int weights[8] = {-64, -32, -16, -8, 8, 16, 32, 64};

int calibrationMin[8] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023}; 
int calibrationMax[8] = {0, 0, 0, 0, 0, 0, 0, 0}; 

float normalizedValues[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

void setup() {
    Serial.begin(9600);
    pinMode(BUTTON, INPUT);
    for (int i = 0; i < 8; i++) {
        pinMode(irPins[i], INPUT);
    }
}

void loop() {
    
    if (digitalRead(BUTTON)){
      calibrate();
    }
  
    float positionSum = 0.0;
    float valueSum = 0.0;

    for (byte i = 0; i < 8; i++) {
        irSensor[i] = analogRead(irPins[i]);
        
        if (calibrationMax[i] > calibrationMin[i]) {
            normalizedValues[i] = (float)(irSensor[i] - calibrationMin[i]) / (calibrationMax[i] - calibrationMin[i]);
            if (normalizedValues[i] < 0) normalizedValues[i] = 0;
            if (normalizedValues[i] > 1) normalizedValues[i] = 1;
        }

        positionSum += normalizedValues[i] * weights[i];
        valueSum += normalizedValues[i];
    }

    float position = (valueSum != 0) ? (float)positionSum / valueSum : 0;

    Serial.print("Values: ");
    for (byte i = 0; i < 8; i++) {
        Serial.print(normalizedValues[i], 3);
        if (i < 7) Serial.print("\t");
    }
    
    Serial.print(" | Position: ");
    Serial.println(position);

    delay(500);
}

void calibrate() {
  Serial.println("Starting calibration");
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    for (byte i = 0; i < 8; i++) {
      irSensor[i] = analogRead(irPins[i]);
      if (irSensor[i] > calibrationMax[i]) calibrationMax[i] = irSensor[i];
      if (irSensor[i] < calibrationMin[i]) calibrationMin[i] = irSensor[i];
    }
    delay(100);
  }
  Serial.println("Calibration finished");
  Serial.print("Calibration Min: ");
  for (byte i = 0; i < 8; i++) {
      Serial.print(calibrationMin[i]);
      if (i < 7) Serial.print("\t");
  }
  Serial.println();
  Serial.print("Calibration Max: ");
  for (byte i = 0; i < 8; i++) {
      Serial.print(calibrationMax[i]);
      if (i < 7) Serial.print("\t");
  }
  Serial.println();
}


  //  drive(80,80);
  //  delay(1000); 
  //  drive(-80,80);
  //  delay(1000);   

void drive(int speedl, int speedr)
{
  speedl=constrain(speedl,-255,255);
  speedr=constrain(speedr,-255,255);
  
  digitalWrite(AIN1, speedl>=0);
  digitalWrite(AIN2, speedl<0);
  analogWrite(PWMA, abs(speedl));
  
  digitalWrite(BIN1, speedr>=0);
  digitalWrite(BIN2, speedr<0);
  analogWrite(PWMB, abs(speedr));
}
