int data;

void setup() {
  Serial.begin(9600);
}
void loop() {
  if (Serial.available() > 0) {
    
    data = Serial.read();
    Serial.print("Value 1:");
    Serial.println(data);
    delay(1);
    data = Serial.read();
    Serial.print("Value 2:");
    Serial.println(data+69);

  }
}

// Serial.write(VALUE) to send Data
