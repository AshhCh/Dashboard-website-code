// constants won't change
const int buttonPin = 4; // ESP32 connected to button's pin
const int LEDPin    = 3;  // ESP32 pin connected to LED's pin

// Button timing thresholds (in milliseconds)
const unsigned long SHORT_PRESS_MAX_DURATION = 1000; // Less than 1 second for a "short" press
const unsigned long MEDIUM_PRESS_MIN_DURATION = 4000; // At least 4 seconds for a "medium" press
const unsigned long MEDIUM_PRESS_MAX_DURATION = 7000; // Max 7 seconds for a "medium" press
const unsigned long LONG_PRESS_MIN_DURATION = 8000;  // At least 8 seconds for a "long" press

// variables will change:
int ledState = LOW;             // the current state of LED
int lastButtonState;            // the previous state of button
int currentButtonState;         // the current state of button

unsigned long buttonPressStartTime = 0; // To store the time when the button was pressed down
bool buttonPressedCurrently = false; // Flag to track if the button is currently held down

// State machine for recording
enum RecordingState {
  IDLE,
  RECORDING_NORMAL,
  RECORDING_SEIZURE,
  RECORDING_FALL
};
RecordingState currentRecordingState = IDLE;

void setup() {
  Serial.begin(115200);             // initialize serial
  pinMode(buttonPin, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  pinMode(LEDPin, OUTPUT);          // set ESP32 pin to output mode
  digitalWrite(LEDPin, ledState);   // Ensure LED starts in LOW state
  currentButtonState = digitalRead(buttonPin); // Read initial button state
  lastButtonState = currentButtonState; // Initialize lastButtonState
}

void loop() {
  lastButtonState    = currentButtonState;      // save the last state
  currentButtonState = digitalRead(buttonPin); // read new state

  // --- Button Press Detection ---
  // Button goes from HIGH (released) to LOW (pressed)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    buttonPressStartTime = millis(); // Record the time the button was pressed
    buttonPressedCurrently = true;
    Serial.println("Button Pressed Down");
  }

  // Button goes from LOW (pressed) to HIGH (released)
  if (lastButtonState == LOW && currentButtonState == HIGH) {
    buttonPressedCurrently = false;
    unsigned long pressDuration = millis() - buttonPressStartTime; // Calculate duration

    Serial.print("Button Released. Press Duration: ");
    Serial.print(pressDuration);
    Serial.println(" ms");

    // --- Action based on press duration ---
    if (pressDuration < SHORT_PRESS_MAX_DURATION) {
      Serial.println("Short Press Detected!");
      if (currentRecordingState == IDLE) {
        currentRecordingState = RECORDING_NORMAL;
        Serial.println("Started recording 'normal' data.");
      } else if (currentRecordingState == RECORDING_NORMAL) {
        currentRecordingState = IDLE;
        Serial.println("Stopped recording 'normal' data. (Would send CSV for normal)");
      } else {
        Serial.println("Cannot start/stop normal recording while another recording is active.");
      }
    } else if (pressDuration >= MEDIUM_PRESS_MIN_DURATION && pressDuration < MEDIUM_PRESS_MAX_DURATION) {
      Serial.println("Medium Press Detected!");
      if (currentRecordingState == IDLE) {
        currentRecordingState = RECORDING_SEIZURE;
        Serial.println("Started recording 'seizure' data.");
      } else if (currentRecordingState == RECORDING_SEIZURE) {
        currentRecordingState = IDLE;
        Serial.println("Stopped recording 'seizure' data. (Would send CSV for seizure)");
      } else {
        Serial.println("Cannot start/stop seizure recording while another recording is active.");
      }
    } else if (pressDuration >= LONG_PRESS_MIN_DURATION) {
      Serial.println("Long Press Detected!");
      if (currentRecordingState == IDLE) {
        currentRecordingState = RECORDING_FALL;
        Serial.println("Started recording 'fall' data.");
      } else if (currentRecordingState == RECORDING_FALL) {
        currentRecordingState = IDLE;
        Serial.println("Stopped recording 'fall' data. (Would send CSV for fall)");
      } else {
        Serial.println("Cannot start/stop fall recording while another recording is active.");
      }
    }
  }

  // --- LED Control (Optional: Can be used to indicate recording state) ---
  // For now, let's keep it toggling on short press (as per original code modified slightly)
  // Or you could make it indicate recording states:
  switch (currentRecordingState) {
    case IDLE:
      digitalWrite(LEDPin, LOW); // LED off when idle
      break;
    case RECORDING_NORMAL:
      digitalWrite(LEDPin, HIGH); // LED on solid for normal
      break;
    case RECORDING_SEIZURE:
      // You could make it blink fast for seizure
      // This would require more complex timing logic or a simple blink:
      if (millis() % 200 < 100) { // Blink every 200ms
        digitalWrite(LEDPin, HIGH);
      } else {
        digitalWrite(LEDPin, LOW);
      }
      break;
    case RECORDING_FALL:
      // You could make it blink slow for fall
      if (millis() % 500 < 250) { // Blink every 500ms
        digitalWrite(LEDPin, HIGH);
      } else {
        digitalWrite(LEDPin, LOW);
      }
      break;
  }


  // Optional: Print current recording state to serial for debugging
  // You might want to do this less frequently to avoid flooding the serial monitor
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 1000) { // Print state every second
    Serial.print("Current State: ");
    switch (currentRecordingState) {
      case IDLE: Serial.println("IDLE"); break;
      case RECORDING_NORMAL: Serial.println("RECORDING_NORMAL"); break;
      case RECORDING_SEIZURE: Serial.println("RECORDING_SEIZURE"); break;
      case RECORDING_FALL: Serial.println("RECORDING_FALL"); break;
    }
    lastPrintTime = millis();
  }


  delay(50); // Shorter delay for better responsiveness in button timing
}
