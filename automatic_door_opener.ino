// Define Pin Assignments
const int MOTOR_PIN_1 = 2;   // Motor control pin 1
const int MOTOR_PIN_2 = 3;   // Motor control pin 2
const int SWITCH_PIN_OPENING = 4; // Switch detecting when doors are opening (LOW when is opening)
const int SWITCH_PIN_OPEN_STOP = 5;   // End stop switch for door open position (LOW when opened)
const int SENSOR_PIN = A7;   // Analog pin for light sensor

// New: Manual Override Button Pins
const int BUTTON_PIN_OPEN = 6;  // Button to manually open the door (LOW when pressed)
const int BUTTON_PIN_CLOSE = 7; // Button to manually close the door (LOW when pressed)

// Define Door States
enum DoorState {
    DOOR_UNKNOWN,      // Initial state, before determining actual position
    DOOR_CLOSED,       // Door is fully closed
    DOOR_OPEN,         // Door is fully open
    OPENING_DOOR,      // Door is currently moving to open position
    CLOSING_DOOR,      // Door is currently moving to closed position
    ERROR_TIMEOUT      // Motor ran for too long without hitting a switch
};

// Global Variables for State Machine and Timing
DoorState currentDoorState = DOOR_UNKNOWN; // Initialize state to unknown
unsigned long motorStartTime = 0;       // Stores the millis() value when motor started
const unsigned long MOTOR_TIMEOUT_MS = 25000; // Max time motor is allowed to run (25 seconds)

// Light Sensor Hysteresis and Averaging
const int LIGHT_THRESHOLD_OPEN = 600; // If averaged light value goes ABOVE this, initiate opening
const int LIGHT_THRESHOLD_CLOSE = 200; // If averaged light value goes BELOW this, initiate closing
                                       // (LIGHT_THRESHOLD_CLOSE must be less than LIGHT_THRESHOLD_OPEN)

const int LIGHT_READING_COUNT = 10;    // Number of readings for averaging
int lightReadings[LIGHT_READING_COUNT]; // Array to store light sensor readings
int lightReadIndex = 0;                 // Current index in the readings array
long lightReadingsTotal = 0;           // Running sum of readings
int averagedLightValue = 0;             // The calculated average light value

const bool USE_LIGHT_SENSOR = true; // Set to false to disable light sensor functionality

// Variables for timed light-based state transitions (single timer)
unsigned long lightConditionStartTime = 0; // Time when light condition became stable for transition
const unsigned long LIGHT_STABLE_TIME_MS = 10000; // Light condition must be stable for 10 seconds

// Variables for non-blocking Serial printing
unsigned long lastSerialPrintTime = 0;
const unsigned long SERIAL_PRINT_INTERVAL_MS = 1000; // Print every 1 second

void setup() {
    // Set pin modes
    pinMode(MOTOR_PIN_1, OUTPUT);
    pinMode(MOTOR_PIN_2, OUTPUT);
    // Use INPUT_PULLUP for end switches and buttons, assuming they connect to GND when activated
    pinMode(SWITCH_PIN_OPENING, INPUT_PULLUP);
    pinMode(SWITCH_PIN_OPEN_STOP, INPUT_PULLUP);
    pinMode(BUTTON_PIN_OPEN, INPUT_PULLUP);  // Manual Open button
    pinMode(BUTTON_PIN_CLOSE, INPUT_PULLUP); // Manual Close button

    // Initialize Serial communication
    Serial.begin(115200);
    Serial.println("Chicken Coop Door Opener - Starting up...");

    // Initialize light sensor readings array
    for (int i = 0; i < LIGHT_READING_COUNT; i++) {
        lightReadings[i] = 0;
    }

    // Determine initial door state based on end switches
    // This is crucial for proper startup
    if (digitalRead(SWITCH_PIN_OPENING) == LOW) {
        currentDoorState = DOOR_CLOSED;
        Serial.println("Initial State: DOOR_CLOSED");
    } else if (digitalRead(SWITCH_PIN_OPEN_STOP) == LOW) {
        currentDoorState = DOOR_OPEN;
        Serial.println("Initial State: DOOR_OPEN");
    } else {
        // If neither switch is active, the door is in an unknown position.
        // It's safer to attempt to close it to a known state (closed) to be safe for night.
        currentDoorState = CLOSING_DOOR;
        digitalWrite(MOTOR_PIN_1, LOW);
        digitalWrite(MOTOR_PIN_2, HIGH); // Start closing motor
        motorStartTime = millis();
        Serial.println("Initial State: DOOR_UNKNOWN. Attempting to CLOSE to a known state.");
    }
    motorStop(); // Ensure motor is off initially until needed
}

// Function to stop the motor
void motorStop() {
    digitalWrite(MOTOR_PIN_1, LOW);
    digitalWrite(MOTOR_PIN_2, LOW);
}

// Function to read and average the light sensor value
void readLightSensor() {
    // Subtract the oldest reading from the total
    lightReadingsTotal = lightReadingsTotal - lightReadings[lightReadIndex];
    // Read the new value
    lightReadings[lightReadIndex] = analogRead(SENSOR_PIN);
    // Add the new reading to the total
    lightReadingsTotal = lightReadingsTotal + lightReadings[lightReadIndex];
    // Advance to the next position in the array
    lightReadIndex = (lightReadIndex + 1) % LIGHT_READING_COUNT;
    // Calculate the average
    averagedLightValue = lightReadingsTotal / LIGHT_READING_COUNT;
}

bool readStableSwitch(int pin, unsigned long debounceTime = 300) {
  if (digitalRead(pin) == HIGH) {
    unsigned long start = millis();
    while (millis() - start < debounceTime) {
      if (digitalRead(pin) == LOW) return false; // bounced back
    }
    return true; // stable high
  }
  return false;
}


void loop() {
    // Always read and average the light sensor value
    readLightSensor();

    // Read end switch states
    int switchOpeningState = digitalRead(SWITCH_PIN_OPENING); // LOW when is opening
    int switchOpenState = digitalRead(SWITCH_PIN_OPEN_STOP);     // LOW when open
    bool isDoorFullyClosed = readStableSwitch(SWITCH_PIN_OPENING);
    bool isDoorFullyOpened = (switchOpenState == LOW);

    //Serial.print("Switch states: ");
    //Serial.print("Closed: ");
    //Serial.print(switchOpeningState == LOW ? "ACTIVE" : "INACTIVE");
    //Serial.print(" | Open: ");
    //Serial.println(switchOpenState == LOW ? "ACTIVE" : "INACTIVE");


    // Read manual button states (LOW when pressed)
    int buttonOpenState = digitalRead(BUTTON_PIN_OPEN);
    int buttonCloseState = digitalRead(BUTTON_PIN_CLOSE);

    // State Machine Logic
    switch (currentDoorState) {
        case DOOR_UNKNOWN:
            // This state should primarily be handled in setup.
            // If we land here and motor is not running, transition to ERROR_TIMEOUT
            if (millis() - motorStartTime >= MOTOR_TIMEOUT_MS && motorStartTime != 0) {
                 motorStop();
                 currentDoorState = ERROR_TIMEOUT;
                 Serial.println("ERROR: DOOR_UNKNOWN Timeout during startup!");
            }
            // Allow manual override from unknown state
            if (buttonOpenState == LOW) {
                currentDoorState = OPENING_DOOR;
                digitalWrite(MOTOR_PIN_1, HIGH);
                digitalWrite(MOTOR_PIN_2, LOW);
                motorStartTime = millis();
                Serial.println("Transition: DOOR_UNKNOWN -> OPENING_DOOR (Manual Override)");
            } else if (buttonCloseState == LOW) {
                currentDoorState = CLOSING_DOOR;
                digitalWrite(MOTOR_PIN_1, LOW);
                digitalWrite(MOTOR_PIN_2, HIGH);
                motorStartTime = millis();
                Serial.println("Transition: DOOR_UNKNOWN -> CLOSING_DOOR (Manual Override)");
            }
            break;

        case DOOR_CLOSED:
            motorStop(); // Ensure motor is off when door is closed
            // Manual override takes precedence
            if (buttonOpenState == LOW) {
                currentDoorState = OPENING_DOOR;
                digitalWrite(MOTOR_PIN_1, HIGH); // Motor direction for opening
                digitalWrite(MOTOR_PIN_2, LOW);
                motorStartTime = millis();
                Serial.println("Transition: DOOR_CLOSED -> OPENING_DOOR (Manual Override)");
            }
            // Automatic open based on light sensor
            else if (USE_LIGHT_SENSOR) {
                if (averagedLightValue > LIGHT_THRESHOLD_OPEN) {
                    if (lightConditionStartTime == 0) {
                        lightConditionStartTime = millis(); // Start timer if condition just met
                        Serial.println("Light condition met for opening. Starting 10s timer...");
                    } else if (millis() - lightConditionStartTime >= LIGHT_STABLE_TIME_MS) {
                        currentDoorState = OPENING_DOOR;
                        digitalWrite(MOTOR_PIN_1, HIGH); // Motor direction for opening
                        digitalWrite(MOTOR_PIN_2, LOW);
                        motorStartTime = millis();
                        Serial.println("Transition: DOOR_CLOSED -> OPENING_DOOR (Light > Threshold for 10s)");
                        lightConditionStartTime = 0; // Reset timer after transition
                    }
                } else {
                    lightConditionStartTime = 0; // Reset timer if light drops below threshold
                }
            }
            // Reset light timer if light sensor is not enabled or door is not fully closed
            else {
                lightConditionStartTime = 0;
            }
            break;

        case DOOR_OPEN:
            motorStop(); // Ensure motor is off when door is open
            // Manual override takes precedence
            if (buttonCloseState == LOW) {
                currentDoorState = CLOSING_DOOR;
                digitalWrite(MOTOR_PIN_1, LOW);  // Motor direction for closing
                digitalWrite(MOTOR_PIN_2, HIGH);
                motorStartTime = millis();
                Serial.println("Transition: DOOR_OPEN -> CLOSING_DOOR (Manual Override)");
            }
            // Automatic close based on light sensor (with 10-second delay)
            else if (USE_LIGHT_SENSOR) {  // && isDoorFullyOpened) {
                if (averagedLightValue < LIGHT_THRESHOLD_CLOSE) {
                    if (lightConditionStartTime == 0) {
                        lightConditionStartTime = millis(); // Start timer if condition just met
                        Serial.println("Light condition met for closing. Starting 10s timer...");
                    } else if (millis() - lightConditionStartTime >= LIGHT_STABLE_TIME_MS) {
                        currentDoorState = CLOSING_DOOR;
                        digitalWrite(MOTOR_PIN_1, LOW);  // Motor direction for closing
                        digitalWrite(MOTOR_PIN_2, HIGH);
                        motorStartTime = millis();
                        Serial.println("Transition: DOOR_OPEN -> CLOSING_DOOR (Light < Threshold for 10s)");
                        lightConditionStartTime = 0; // Reset timer after transition
                    }
                } else {
                    lightConditionStartTime = 0; // Reset timer if light rises above threshold
                }
            }
            // Reset light timer if light sensor is not enabled or door is not fully open
            else {
                lightConditionStartTime = 0;
            }
            break;

        case OPENING_DOOR:
            // Manual override: pressing CLOSE button during opening
            if (buttonCloseState == LOW) {
                motorStop(); // Stop current motion
                currentDoorState = CLOSING_DOOR;
                digitalWrite(MOTOR_PIN_1, LOW); // Reverse direction
                digitalWrite(MOTOR_PIN_2, HIGH);
                motorStartTime = millis();
                Serial.println("Transition: OPENING_DOOR -> CLOSING_DOOR (Manual Override)");
                break; // Exit switch case to allow new state logic to run next loop
            }

            // Check if the door reached the open limit switch
            if (isDoorFullyOpened) { // Switch activated (pulled to LOW)
                motorStop();
                currentDoorState = DOOR_OPEN;
                Serial.println("Transition: OPENING_DOOR -> DOOR_OPEN (Switch Hit)");
            }
            // Check for motor timeout
            else if (millis() - motorStartTime >= MOTOR_TIMEOUT_MS) {
                motorStop();
                currentDoorState = ERROR_TIMEOUT;
                Serial.println("ERROR: OPENING_DOOR Timeout!");
            }
            break;

        case CLOSING_DOOR:
            // Manual override: pressing OPEN button during closing
            if (buttonOpenState == LOW) {
                motorStop(); // Stop current motion
                currentDoorState = OPENING_DOOR;
                digitalWrite(MOTOR_PIN_1, HIGH); // Reverse direction
                digitalWrite(MOTOR_PIN_2, LOW);
                motorStartTime = millis();
                Serial.println("Transition: CLOSING_DOOR -> OPENING_DOOR (Manual Override)");
                break; // Exit switch case to allow new state logic to run next loop
            }

            // Check if the door reached the closed limit switch
            // motor must run at least 2s before we check if doors are closed
            if (millis() - motorStartTime >= 2000 && isDoorFullyClosed) { // Switch activated (pulled to LOW)
                motorStop();
                currentDoorState = DOOR_CLOSED;
                Serial.println("Transition: CLOSING_DOOR -> DOOR_CLOSED (Switch Hit)");
            }
            // Check for motor timeout
            else if (millis() - motorStartTime >= MOTOR_TIMEOUT_MS) {
                motorStop();
                currentDoorState = ERROR_TIMEOUT;
                Serial.println("ERROR: CLOSING_DOOR Timeout!");
            }
            break;

        case ERROR_TIMEOUT:
            motorStop(); // Ensure motor is off
            // Allow manual override to clear error state and attempt movement
            if (buttonOpenState == LOW) {
                currentDoorState = OPENING_DOOR;
                digitalWrite(MOTOR_PIN_1, HIGH);
                digitalWrite(MOTOR_PIN_2, LOW);
                motorStartTime = millis();
                Serial.println("Transition: ERROR_TIMEOUT -> OPENING_DOOR (Manual Override)");
            } else if (buttonCloseState == LOW) {
                currentDoorState = CLOSING_DOOR;
                digitalWrite(MOTOR_PIN_1, LOW);
                digitalWrite(MOTOR_PIN_2, HIGH);
                motorStartTime = millis();
                Serial.println("Transition: ERROR_TIMEOUT -> CLOSING_DOOR (Manual Override)");
            }
            // Otherwise, stay in error state and log
            else {
                // Only print this periodically to avoid flooding serial
                static unsigned long lastErrorPrintTime = 0;
                if (millis() - lastErrorPrintTime >= 5000) { // Print error every 5 seconds
                    lastErrorPrintTime = millis();
                    Serial.println("ERROR_TIMEOUT state. Manual intervention via buttons or reset required.");
                }
            }
            break;
    }

    // Non-blocking Serial Debugging Output
    if (millis() - lastSerialPrintTime >= SERIAL_PRINT_INTERVAL_MS) {
        lastSerialPrintTime = millis();
        Serial.print("State: ");
        switch (currentDoorState) {
            case DOOR_UNKNOWN: Serial.print("DOOR_UNKNOWN"); break;
            case DOOR_CLOSED: Serial.print("DOOR_CLOSED"); break;
            case DOOR_OPEN: Serial.print("DOOR_OPEN"); break;
            case OPENING_DOOR: Serial.print("OPENING_DOOR"); break;
            case CLOSING_DOOR: Serial.print("CLOSING_DOOR"); break;
            case ERROR_TIMEOUT: Serial.print("ERROR_TIMEOUT"); break;
        }
        Serial.print(" | Light Avg: ");
        Serial.print(averagedLightValue);
        Serial.print(" | SwOpening: ");
        Serial.print(switchOpeningState == LOW ? "ACTIVE" : "INACTIVE");
        Serial.print(" | SwOpen: ");
        Serial.print(switchOpenState == LOW ? "ACTIVE" : "INACTIVE");
        Serial.print(" | BtnOpen: ");
        Serial.print(buttonOpenState == LOW ? "PRESSED" : "RELEASED");
        Serial.print(" | BtnClose: ");
        Serial.print(buttonCloseState == LOW ? "PRESSED" : "RELEASED");
        if (currentDoorState == OPENING_DOOR || currentDoorState == CLOSING_DOOR) {
            Serial.print(" | Motor Run Time: ");
            Serial.print((millis() - motorStartTime) / 1000);
            Serial.print("s / ");
            Serial.print(MOTOR_TIMEOUT_MS / 1000);
            Serial.print("s");
        }
        Serial.println();
    }
}
