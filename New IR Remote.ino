/*
    IR Receiver with EEPROM
    - All LEDs (red, orange, green) blink before resetting when "0xF20D0707" is received.
    - Orange LED blinks, turning off other LEDs.
    - Only one LED (orange or green) can be on at a time.
    - Reset the Arduino when "0xF20D0707" is received, blink all LEDs 3 times before resetting.
    - Orange and Green LED functionalities only occur if pin 8 is high.
    
    IR Transmitter - Send standard key 0xFA050707 every 3 seconds
*/

#include <Arduino.h>
#include <IRremote.hpp>
#include <EEPROM.h>
#include <avr/wdt.h>  // Include for watchdog timer (for resetting the Arduino)

// IR Receiver and Transmitter Pins
const int IR_RECEIVE_PIN = 7;   // Pin for receiving IR signals
const int IR_SEND_PIN = 3;      // Pin for the IR LED (connected to the transmitter)

// LED and Control Pins
int lv_counter = 0;
const int redPin = 10;          // Pin connected to the red LED
const int lowBatteryPin = 4;    // Define pin 4 for low battery indication
const int greenPin = 5;         // Pin connected to the green LED
const int orangePin = 6;        // Pin connected to the orange LED
const int controlPin = A1;       // Pin to check if functionalities should occur
const int pin9 = 9;             // Pin to be set HIGH for 2 seconds when the command is received
const int pin11 = 11;           // Pin to be set HIGH for 2 seconds when the standard key is received
const int batteryPin = A0;      // Pin connected to the battery for voltage measurement

// Standard key and timing
const int batteryCapacity = 1000; // Replace this with the actual battery capacity in mAh
unsigned long lastBatteryCheckTime = 0; // Track the last battery check time
const unsigned long batteryCheckInterval = 60000; // 60 seconds interval in milliseconds
unsigned long standardKey = 0xFA050707;  // Default standard key
const unsigned long transmitInterval = 150; // 3 seconds interval in milliseconds
unsigned long lastTransmitTime = 0;     // Track the last time the key was sent

bool isOrangeLedOn = false;     // Variable to store the orange LED state
bool isRedLedOn = false;        // Variable to store the red LED state
bool updateMode = false;        // Flag to indicate if we're in update mode
unsigned long updateStartTime = 0; // Time when update mode is entered
const unsigned long updateWaitTime = 2000; // 2 seconds wait time for new key
bool greenLedBlinking = false;  // Flag to control the green LED blinking
bool greenLedContinuous = false; // Flag for continuous green LED

// EEPROM addresses for saving LED states
const int EEPROM_ORANGE_LED_STATE = 4; // Address to store orange LED state
const int EEPROM_GREEN_LED_STATE = 5;  // Address to store green LED state
const int EEPROM_RED_LED_STATE = 6;    // Address to store red LED state

// Function to read the standard key from EEPROM
void readStandardKeyFromEEPROM() {
    EEPROM.get(0, standardKey); // Read the key stored at address 0
}

// Function to write the standard key to EEPROM
void writeStandardKeyToEEPROM(unsigned long key) {
    EEPROM.put(0, key); // Write the key to address 0
}

// Function to save LED states to EEPROM
void saveLedStatesToEEPROM() {
    EEPROM.write(EEPROM_ORANGE_LED_STATE, isOrangeLedOn);
    EEPROM.write(EEPROM_GREEN_LED_STATE, greenLedContinuous);
    EEPROM.write(EEPROM_RED_LED_STATE, isRedLedOn); // Save the red LED state
}

// Function to clear EEPROM and reset LED states
void clearEEPROMAndReset() {
    EEPROM.write(EEPROM_ORANGE_LED_STATE, 0);
    EEPROM.write(EEPROM_GREEN_LED_STATE, 0);
    EEPROM.write(EEPROM_RED_LED_STATE, 0); // Clear red LED state

    // Reset LED states
    isOrangeLedOn = false;
    greenLedContinuous = false;
    isRedLedOn = false; // Reset red LED state

    // Set LED pins to LOW
    digitalWrite(orangePin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, LOW); // Turn off red LED

    Serial.println("EEPROM cleared and LED states reset.");

    // Blink orange and green LEDs 3 times
    for (int i = 0; i < 3; i++) {
        digitalWrite(orangePin, HIGH);
        digitalWrite(greenPin, HIGH);
        delay(300);
        digitalWrite(orangePin, LOW);
        digitalWrite(greenPin, LOW);
        delay(300);
    }
}

// Function to blink the green LED when the key is updated
void blinkGreenLed(int times, int delayTime) {
    if (isOrangeLedOn) {
        digitalWrite(orangePin, LOW); // Turn off the orange LED
        isOrangeLedOn = false; // Reset orange LED state
        saveLedStatesToEEPROM(); // Save state
        Serial.println("Orange LED turned off as green LED starts blinking.");
    }

    for (int i = 0; i < times; i++) {
        digitalWrite(greenPin, HIGH); // Turn the green LED on
        delay(delayTime);             // Wait
        digitalWrite(greenPin, LOW);  // Turn the green LED off
        delay(delayTime);             // Wait
    }
}

// Function to blink all LEDs before resetting
void blinkAllLeds(int times, int delayTime) {
    for (int i = 0; i < times; i++) {
        digitalWrite(redPin, HIGH);
        digitalWrite(orangePin, HIGH);
        digitalWrite(greenPin, HIGH);
        delay(delayTime);  // Wait

        digitalWrite(redPin, LOW);
        digitalWrite(orangePin, LOW);
        digitalWrite(greenPin, LOW);
        delay(delayTime);  // Wait
    }
}

// Function to measure battery voltage and calculate percentage
float measureBatteryVoltage() {
    int rawValue = analogRead(batteryPin); // Read the raw ADC value
    float voltage = (rawValue / 1023.0) * 5.0; // Convert ADC value to voltage (assuming 5V reference)
    return voltage;
}

// Function to calculate battery percentage based on voltage
int calculateBatteryPercentage(float voltage) {
    // Assuming the battery voltage range is 3.0V to 4.2V
    if (voltage < 3.0) return 0;       // Below 3.0V = 0%
    if (voltage > 4.2) return 100;      // Above 4.2V = 100%
    return (int)((voltage - 3.0) / (4.2 - 3.0) * 100); // Calculate percentage
}

void setup() {
    Serial.begin(9600);

    // Initialize the IR receiver and transmitter
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Initialize the IR receiver
    IrSender.begin(IR_SEND_PIN);  // Initialize the IR transmitter

    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT); // Initialize green LED pin
    pinMode(orangePin, OUTPUT); // Initialize orange LED pin
    pinMode(controlPin, INPUT); // Initialize control pin as input
    pinMode(pin9, OUTPUT); // Set pin 9 as OUTPUT
    pinMode(pin11, OUTPUT); // Set pin 11 as OUTPUT
    digitalWrite(redPin, LOW);  // Ensure the red LED is off initially

    // 4 pin setup code
    pinMode(lowBatteryPin, OUTPUT); // Initialize pin 4 as output
    digitalWrite(lowBatteryPin, LOW); // Set pin 4 to LOW initially

    // Restore LED states from EEPROM
    isOrangeLedOn = EEPROM.read(EEPROM_ORANGE_LED_STATE);
    greenLedContinuous = EEPROM.read(EEPROM_GREEN_LED_STATE);
    isRedLedOn = EEPROM.read(EEPROM_RED_LED_STATE); // Read red LED state

    // Set the LEDs to their previous states
    digitalWrite(orangePin, isOrangeLedOn ? HIGH : LOW);
    digitalWrite(greenPin, greenLedContinuous ? HIGH : LOW);
    digitalWrite(redPin, isRedLedOn ? HIGH : LOW); // Set red LED state

    // Read the standard key from EEPROM
    readStandardKeyFromEEPROM();
    Serial.print("Initialized standard key from EEPROM: ");
    Serial.println(standardKey, HEX);

    Serial.println("IR transmitter initialized. Sending standard key 0xFA050707 every 3 seconds.");
}

void loop() {
    float pin8State = analogRead(controlPin);
    //Serial.print("pin state: ");
    Serial.println(pin8State);
    static unsigned long lastMeasurementTime = 0;
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastMeasurementTime >= 60000) {
        lastMeasurementTime = currentMillis;
        
        // Measure and display battery voltage
        float currentVoltage = measureBatteryVoltage();
        Serial.print("Battery Voltage: ");
        Serial.print(currentVoltage);
        Serial.println(" V");

        // Print the known battery capacity
        Serial.print("Battery Capacity: ");
        Serial.print(batteryCapacity); // Print the predefined capacity
        Serial.println(" mAh");

        // Calculate and display battery percentage
        float batteryPercentage = calculateBatteryPercentage(currentVoltage);
        Serial.print("Battery Percentage: ");
        Serial.print(batteryPercentage);
        Serial.println(" %");

        // Check if battery percentage is less than 25%
        if (batteryPercentage < 25) {
            digitalWrite(lowBatteryPin, HIGH); // Turn pin 4 HIGH if percentage is < 25%
            Serial.println("Battery low: pin 4 set HIGH");
        } else {
            digitalWrite(lowBatteryPin, LOW);  // Turn pin 4 LOW if percentage is >= 25%
        }
    }
    // Measure battery voltage and percentage every 60 seconds
/*if (millis() - lastBatteryCheckTime > batteryCheckInterval) {
    float voltage = measureBatteryVoltage();
    int batteryPercentage = calculateBatteryPercentage(voltage);
    Serial.print("Battery Voltage: ");
    Serial.print(voltage);
    Serial.print(batteryPercentage);
    Serial.println(" %");

    lastBatteryCheckTime = millis();  // Update the last battery check time
}*/
    if (pin8State < 1000) {
    lv_counter = lv_counter +1;
    if (lv_counter >5) {
        // Turn ON the red LED and turn off the other LEDs
        digitalWrite(greenPin, LOW);
        digitalWrite(orangePin, LOW);
        digitalWrite(redPin, HIGH); // Turn ON the red LED
        isOrangeLedOn = false;
        greenLedBlinking = false;
        greenLedContinuous = false;
        isRedLedOn = true; // Set the red LED state to ON
        saveLedStatesToEEPROM(); // Save the state

        // Only allow the "clear memory" command (0xF10E0707) when pin 8 is LOW
        if (IrReceiver.decode()) {
            Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
            IrReceiver.printIRResultShort(&Serial);

            // Check for the "clear memory" command
            if (IrReceiver.decodedIRData.decodedRawData == 0xF10E0707) {
                Serial.println("Clear EEPROM command received.");
                clearEEPROMAndReset(); // Clear EEPROM and reset LED states
            }

            return; // Prevent processing of any other commands
        }
    } else {
        // Pin 8 is HIGH, all other functionality remains the same
        if (IrReceiver.decode()) {
            Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);
            IrReceiver.printIRResultShort(&Serial);

            if (IrReceiver.decodedIRData.decodedRawData == 0xF9060707) {
                updateMode = true;
                updateStartTime = millis();
                Serial.println("Entered update mode. Send new key to update within 2 seconds.");
            }

            if (IrReceiver.decodedIRData.decodedRawData == 0xF30C0707) {
                greenLedBlinking = false;
                greenLedContinuous = true;
                digitalWrite(greenPin, HIGH);
                digitalWrite(orangePin, LOW);
                isOrangeLedOn = false;
                saveLedStatesToEEPROM(); // Save state
                Serial.println("Green LED turned on continuously. Orange LED turned off.");
            }

            if (IrReceiver.decodedIRData.decodedRawData == 0xF20D0707) {
                Serial.println("Reset signal received. Blinking all LEDs and resetting.");

                // Save the current LED states to EEPROM
                saveLedStatesToEEPROM();

                blinkAllLeds(3, 300);
                digitalWrite(pin9, HIGH);  // Turn on pin 9
                delay(2000);                // Wait for 2 seconds
                digitalWrite(pin9, LOW);   // Turn off pin 9   
                wdt_enable(WDTO_15MS);

                while (1);
            }

            if (IrReceiver.decodedIRData.decodedRawData == 0xF10E0707) {
                Serial.println("Clear EEPROM command received.");
                clearEEPROMAndReset(); // Clear EEPROM and reset LED states
                digitalWrite(pin9, HIGH);  // Turn on pin 9
                delay(2000);                // Wait for 2 seconds
                digitalWrite(pin9, LOW);   // Turn off pin 9    
            }

            if (updateMode) {
                if (millis() - updateStartTime < updateWaitTime) {
                    // Waiting for new key to be sent
                } else {
                    standardKey = IrReceiver.decodedIRData.decodedRawData;
                    writeStandardKeyToEEPROM(standardKey);
                    Serial.print("Updated standard key to: ");
                    Serial.println(standardKey, HEX);

                    greenLedBlinking = true;
                    greenLedContinuous = false;
                    blinkGreenLed(5, 200);

                    updateMode = false;
                }
            } else {
                if (IrReceiver.decodedIRData.decodedRawData == standardKey) {
                    if (!isOrangeLedOn) {
                        isOrangeLedOn = true;
                        digitalWrite(orangePin, HIGH);
                        digitalWrite(greenPin, LOW);
                        greenLedBlinking = false;
                        greenLedContinuous = false;
                        saveLedStatesToEEPROM(); // Save state
                        Serial.println("Orange LED turned on. Green LED turned off.");

                        // Set pin 9 HIGH for 2 seconds
                        digitalWrite(pin9, HIGH);  // Turn on pin 9
                        delay(2000);                // Wait for 2 seconds
                        digitalWrite(pin9, LOW);   // Turn off pin 9
                    }
                }
            }

            IrReceiver.resume(); // Prepare for the next IR signal
        }
    }
    }
    else{
      lv_counter = 0;
    }
    if (greenLedBlinking) {
        digitalWrite(greenPin, HIGH);
        delay(200);
        digitalWrite(greenPin, LOW);
        delay(200);
    }

    if (greenLedContinuous) {
        digitalWrite(greenPin, HIGH);
    }

    // IR Transmitter: Send the standard key every 3 seconds
    if (millis() - lastTransmitTime > transmitInterval) {
        IrSender.sendNEC(standardKey, 32);  // NEC protocol, sending 32 bits
        //Serial.print("Sent standard key: ");
        //Serial.println(standardKey, HEX);
        lastTransmitTime = millis();  // Update the last transmit time
    }
}
