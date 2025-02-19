#include <Servo.h>
#include <SoftwareSerial.h>

// Define Bluetooth module RX/TX
SoftwareSerial bluetooth(A0, A1);  // RX, TX

Servo Servo1;  // Servo object
Servo Servo2;  // Servo object

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  
  Servo1.attach(12);  // Attach servo to pin 9
  Servo2.attach(11);  // Attach servo to pin 9

}

void loop() {
  if (bluetooth.available()) {
    char command = bluetooth.read(); // Read command from Bluetooth
    Serial.print("Received: ");
    Serial.println(command);

    // Move servo based on key pressed
    if (command == 'O') {   // Move servo left
      Servo1.write(160);
      Serial.println("Servo moved to 45°");
    }
    else if (command == 'C') {   // Move servo right
      Servo1.write(90);
      Serial.println("Servo moved to 135°");
    }
    else if (command == 'U') {   // Move servo to center
      Servo2.write(155);
      Serial.println("Servo moved to 90° (Center)");
    }

    else if (command == 'D') {   // Move servo left
      Servo2.write(50);
      Serial.println("Servo moved to 45°");
    }


    else if (command == 'o') {   // Move servo right
      Servo1.write(140);
      Serial.println("Servo moved to 135°");
    }
    else if (command == 'c') {   // Move servo to center
      Servo1.write(90);
      Serial.println("Servo moved to 90° (Center)");
    }
    
    else if (command == 'u') {   // Move servo right
      Servo2.write(70);
      Serial.println("Servo moved to 135°");
    }
    else if (command == 'd') {   // Move servo to center
      Servo2.write(50);
      Serial.println("Servo moved to 90° (Center)");
    }
  }
}