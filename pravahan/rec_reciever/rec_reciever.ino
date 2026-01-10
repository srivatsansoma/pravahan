int radio_recpins[] = {3, 21, 5, 17, 6, 4};
int numChannels = sizeof(radio_recpins) / sizeof(radio_recpins[0]);

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < numChannels; i++) {
    pinMode(radio_recpins[i], INPUT);
  }
}

void loop() {
  for (int i = 0; i < numChannels; i++) {
    unsigned long pwm = pulseIn(radio_recpins[i], HIGH, 25000);
    Serial.print(pwm);
    Serial.print(" ");
  }

  Serial.println();
  delay(100);
}
