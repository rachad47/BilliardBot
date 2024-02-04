#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <AccelStepper.h>

const char* ssid = "none";
const char* password = "12345678";

const int stepPinX = 14;
const int dirPinX = 25;
const int stepPinY = 27;
const int dirPinY = 33;
const int stepPinZ = 26;
const int dirPinZ = 32;
const int enablePin = 13;

const int charge = 23; 
const int fire = 5;

bool movementComplete = false;

AccelStepper stepperX(AccelStepper::DRIVER, stepPinX, dirPinX);
AccelStepper stepperY(AccelStepper::DRIVER, stepPinY, dirPinY);
AccelStepper stepperZ(AccelStepper::DRIVER, stepPinZ, dirPinZ);

// port 80
AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);

  pinMode(charge, OUTPUT);
  pinMode(fire, OUTPUT);

  stepperX.setMaxSpeed(500);
  stepperY.setMaxSpeed(500);
  stepperZ.setMaxSpeed(500);
  stepperX.setAcceleration(10000);
  stepperY.setAcceleration(10000);
  stepperZ.setAcceleration(10000);

  // Route for handling motor control
  server.on("/control", HTTP_GET, [](AsyncWebServerRequest *request){
    String stepsX, speedX, stepsY, speedY, stepsZ, speedZ;

    if (request->hasParam("stepsX") && request->hasParam("speedX") &&
        request->hasParam("stepsY") && request->hasParam("speedY") &&
        request->hasParam("stepsZ") && request->hasParam("speedZ")) {

      stepsX = request->getParam("stepsX")->value();
      speedX = request->getParam("speedX")->value();
      stepsY = request->getParam("stepsY")->value();
      speedY = request->getParam("speedY")->value();
      stepsZ = request->getParam("stepsZ")->value();
      speedZ = request->getParam("speedZ")->value();

      controlSteppers(stepsX.toInt(), speedX.toFloat(), stepsY.toInt(), speedY.toFloat(), stepsZ.toInt(), speedZ.toFloat());
      request->send(200, "text/plain", "Motors commanded successfully");
    } else {
      request->send(400, "text/plain", "Invalid parameters");
    }
  });

  server.on("/strike", HTTP_GET, [](AsyncWebServerRequest *request){
    int chargeDuration;

    if (request->hasParam("chargeDuration")) {
      chargeDuration = request->getParam("chargeDuration")->value().toInt();
    }
    strikeSequence(chargeDuration);
    request->send(200, "text/plain", "Strike sequence executed");
  });


  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    if (movementComplete) {
        request->send(200, "text/plain", "Movement complete");
        movementComplete = false; // Reset the flag
    } else {
        request->send(200, "text/plain", "Movement in progress");
    }
  });

  server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){
    stepperX.stop();
    stepperY.stop();
    stepperZ.stop();
    digitalWrite(enablePin, HIGH);
    request->send(200, "text/plain", "All motors stopped immediately");
  });

  server.begin();
}
  static bool motorsStopped = true;

void loop() {
  stepperX.run();
  stepperY.run();
  stepperZ.run();

   if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0 && stepperZ.distanceToGo() == 0) {
    if (!motorsStopped) {
      delay(500);
      digitalWrite(enablePin, HIGH);
      Serial.println("Motors stopped");
      motorsStopped = true;
      movementComplete = true;

    }
  } else {
    motorsStopped = false;
  }

}

void controlSteppers(int stepsX, float speedX, int stepsY, float speedY, int stepsZ, float speedZ) {
  digitalWrite(enablePin, LOW);
  movementComplete=false;

  stepperX.setMaxSpeed(speedX);
  stepperY.setMaxSpeed(speedY);
  stepperZ.setMaxSpeed(speedZ);

  stepperX.move(stepsX);
  stepperY.move(stepsY);
  stepperZ.move(stepsZ);
}



void strikeSequence(int chargeDuration) {
  digitalWrite(charge, HIGH);  
  delay(chargeDuration);        
  digitalWrite(charge, LOW);
  delay(100);
  digitalWrite(enablePin, LOW);  
  delay(100);
  digitalWrite(fire, HIGH);  
  delay(500);                 
  digitalWrite(fire, LOW);
  delay(700);
  digitalWrite(enablePin, HIGH);  
}