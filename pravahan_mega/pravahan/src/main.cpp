#include <Arduino.h>

volatile uint32_t pwm_time_period_high[6] = {0};
volatile uint32_t pwm_time_period_high_temp[6] = {0};
volatile uint32_t pwm_time_period_low[6] = {0};
volatile uint32_t pwm_time_period_low_temp[6] = {0};

uint8_t last_pinB_state = 0;
uint8_t last_pinJ_state = 0;

#define SENSOR_TRIGGER_H 0xA5
#define SENSOR_TRIGGER_L 0x5A

uint32_t local[6];

void dyp_a21(){
  // Send trigger to sensor
  Serial1.write(SENSOR_TRIGGER_H);
  Serial1.write(SENSOR_TRIGGER_L);

  delay(50); // Wait for sensor to process

  // Only proceed if at least 4 bytes are available
  if (Serial1.available() >= 4) {
    uint8_t header = Serial1.read();

    if (header != 0xFF) {
      // Sync error, clear buffer
      while (Serial1.available()) Serial1.read();
      Serial.println("Sync error");
      return;
    }

    uint8_t dataH = Serial1.read();
    uint8_t dataL = Serial1.read();
    uint8_t checksum = Serial1.read();

    uint8_t sum = (0xFF + dataH + dataL) & 0xFF;

    if (sum == checksum) {
      int distance_mm = (dataH << 8) | dataL;

      // Optional: ignore out of range values
      if (distance_mm >= 30 && distance_mm <= 5000) {
        // Send in simple format for Python parsing
        Serial.println(distance_mm); // distance in mm
      } else {
        Serial.println("DIST:ERR"); // out of range
      }
    } else {
      Serial.println("DIST:ERR"); // checksum error
    }
  } else {
    // No response yet
    // Serial.println("No response"); // optional debug
  }
}

void setup() {
  Serial1.begin(115200);  // RX1=19, TX1=18 for DYP-A21
  while (Serial1.available()) Serial1.read();
  Serial.begin(115200);

  PCICR |= (1 << PCIE0) | (1 << PCIE1);
  PCMSK0 |= 0b00001111;
  PCMSK1 |= 0b00000011;

  DDRB &= ~0b00001111;
  DDRJ &= ~0b00000011;


  last_pinB_state = PINB;
  last_pinJ_state = PINJ;

  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // TCCR1A = B0;
  // TCCR1B = B0;
  // TCCR1B |= B010; //PRESCALER = 64
  // OCR1A = 2499; //CMP TICKS
  // TIMSK1 = B010; //ENABLE CMP INTERUPT
}

ISR(PCINT0_vect) {
  uint8_t curr = PINB;

  for (int i = 0; i < 4; i++) {
    if ((curr & (1 << i)) && !(last_pinB_state & (1 << i))) {
      pwm_time_period_high_temp[i] = micros();
      pwm_time_period_low[i] = micros() - pwm_time_period_low_temp[i];
    } else if (!(curr & (1 << i)) && (last_pinB_state & (1 << i))) {
      pwm_time_period_low_temp[i] = micros();
      pwm_time_period_high[i] = micros() - pwm_time_period_high_temp[i];
    }
  }

  last_pinB_state = curr;
}

ISR(PCINT1_vect) {
  uint8_t curr = PINJ;

  for (int i = 0; i < 2; i++) {
    if ((curr & (1 << i)) && !(last_pinJ_state & (1 << i))) {
      pwm_time_period_high_temp[4 + i] = micros();
      pwm_time_period_low[4 + i] = micros() - pwm_time_period_low_temp[4 + i];
    } else if (!(curr & (1 << i)) && (last_pinJ_state & (1 << i))) {
      pwm_time_period_low_temp[4 + i] = micros();
      pwm_time_period_high[4 + i] = micros() - pwm_time_period_high_temp[4 + i];
    }
  }

  last_pinJ_state = curr;
}

void loop() {
  dyp_a21();
  for (int i = 0; i < 6; i++) local[i] = pwm_time_period_high[i];
  Serial.print("/.");
  for (int i = 0; i < 6; i++){
    Serial.print(local[i]);
    Serial.print(" ");
  };
  Serial.println(".\\");
}
