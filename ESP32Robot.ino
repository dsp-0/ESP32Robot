void startOTA();

void setup() {
  Serial.begin(115200);
  startOTA();
  Serial.println("Started");
}

void loop() {
}
