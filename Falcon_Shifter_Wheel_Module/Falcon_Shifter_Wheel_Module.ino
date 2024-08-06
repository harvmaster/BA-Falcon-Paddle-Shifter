/*
  ESP32-C3 Button Reader with Voltage Delta Approach
  
  This code reads button presses from two analog inputs (Media and Cruise control buttons)
  on an ESP32-C3 microcontroller. It uses a voltage delta approach to determine which button
  was pressed, implementing debouncing and noise filtering for more reliable readings.

  Button Resistances and Voltage Readings:
  Media:
    - Seek: 1580ohms (2.5 DB voltage readings: 0.365v)
    - Volume Up: 573ohms (2.5 DB voltage readings: 0.145v)
    - Volume Down: 1.9ohms (2.5 DB voltage readings: 0v)
    - Mode: 3350ohms (2.5 DB voltage readings: 0.670v)

  Cruise Control:
    - Idle: 4160ohms (2.5 DB voltage readings: 0.785)
    - Res/Coast: 1520ohms (2.5 DB voltage readings: 0.385v)
    - Set +: 730ohms (2.5 DB voltage readings: 0.250v)
    - Set -: 370ohms (2.5 DB voltage readings: 0.070v)
    - Cruise: Between 2 and 4 ohms? (2.5 DB voltage readings: 0v)

  Horn:
    - Switched: Open/Closed
*/

#include <Arduino.h>
#include <map>
#include <Wire.h>

// Pin definitions for the analog inputs
const int MEDIA_PIN = 1;  // GPIO1 (ADC1_CH0) - Connected to the Media buttons
const int CRUISE_PIN = 2; // GPIO2 (ADC2_CH0) - Connected to the Cruise Control buttons

// I2C configuration
const int I2C_SDA_PIN = 8; // GPIO20 - I2C SDA pin
const int I2C_SCL_PIN = 9;  // GPIO21 - I2C SCL pin
const int I2C_ADDRESS = 0x04; // I2C address of the slave device

// ADC configuration - Set the ADC resolution to 12 bits for more precise readings
const uint8_t ADC_WIDTH_12BIT = 12;

// Voltage thresholds for button detection
// These maps associate voltage levels with button names for easy identification
std::map<float, String> mediaButtonMap = {
    {1.000, "Idle"},     // No button pressed
    {0.670, "Mode"},     // Mode button pressed
    {0.365, "Seek"},     // Seek button pressed
    {0.145, "Volume Up"},// Volume Up button pressed
    {0.000, "Volume Down"}// Volume Down button pressed
};

std::map<float, String> cruiseButtonMap = {
    {1.000, "Idle"},     // No button pressed
    {0.750, "Res/Coast"},// Resume/Coast button pressed
    {0.410, "Set +"},    // Set+ button pressed
    {0.170, "Set -"},    // Set- button pressed
    {0.000, "Cruise"}    // Cruise button pressed
};

// Debounce and voltage delta configuration
const unsigned long debounceDelay = 50; // Milliseconds to wait before registering a new button press
const float VOLTAGE_DELTA_THRESHOLD = 0.05; // Voltage change required to consider a new button press
const int STABLE_READINGS_REQUIRED = 3; // Number of consistent readings required to confirm a button press

// Button state structure to keep track of each button's current state and recent readings
struct ButtonState {
    unsigned long lastDebounceTime = 0; // Last time the button state was updated
    String currentButtonName = "Idle"; // Current detected button name
    float lastStableVoltage = 1.0; // Last stable voltage reading
    float recentVoltages[STABLE_READINGS_REQUIRED] = {1.0}; // Array to store recent voltage readings
    int stableReadingCount = 0; // Counter for consecutive stable readings
};

// Create instances of ButtonState for Media and Cruise buttons
ButtonState mediaButton;
ButtonState cruiseButton;

// Function to determine which button was pressed based on the measured voltage
String getButtonName(float voltage, const std::map<float, String>& buttonMap) {
    float minDiff = 1.0;
    String buttonName = "Unknown";
    
    // Iterate through the buttonMap to find the closest voltage match
    for (const auto& entry : buttonMap) {
        float diff = abs(voltage - entry.first);
        if (diff < minDiff) {
            minDiff = diff;
            buttonName = entry.second;
        }
    }
    
    return buttonName;
}

void setup() {
    // Initialize serial communication for debugging output
    Serial.begin(115200);
    delay(1000); // Short delay to ensure serial is ready

    // Configure ADC
    analogSetAttenuation(ADC_2_5db); // Set attenuation for 2.5V max input
    analogContinuousSetWidth(ADC_WIDTH_12BIT); // Set ADC resolution to 12 bits

    // Set pin modes for input pins
    pinMode(MEDIA_PIN, INPUT);
    pinMode(CRUISE_PIN, INPUT);

    // Initialize I2C communication
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    Serial.println("ESP32-C3 Button Reader with Voltage Delta Approach");
}

// Function to read ADC values and detect button presses using voltage delta approach
void readAndPrintADC(const char* name, int pin, ButtonState& state, const std::map<float, String>& buttonMap) {
    // Read ADC value and convert to voltage
    int adcValue = analogReadMilliVolts(pin);
    float voltage = (float)adcValue / 1000.0; // Convert millivolts to volts
    
    unsigned long currentTime = millis();
    
    // Shift recent voltage readings to make room for the new reading
    for (int i = STABLE_READINGS_REQUIRED - 1; i > 0; i--) {
        state.recentVoltages[i] = state.recentVoltages[i-1];
    }
    state.recentVoltages[0] = voltage; // Add the new reading
    
    // Check if we have stable readings by comparing recent voltages
    bool isStable = true;
    for (int i = 1; i < STABLE_READINGS_REQUIRED; i++) {
        if (abs(state.recentVoltages[i] - state.recentVoltages[0]) > VOLTAGE_DELTA_THRESHOLD) {
            isStable = false;
            break;
        }
    }
    
    // Update the stable reading count
    if (isStable) {
        state.stableReadingCount++;
    } else {
        state.stableReadingCount = 0; // Reset if not stable
    }
    
    // Check if we have enough stable readings and significant voltage change to consider a button press
    if (state.stableReadingCount >= STABLE_READINGS_REQUIRED &&
        abs(voltage - state.lastStableVoltage) > VOLTAGE_DELTA_THRESHOLD &&
        (currentTime - state.lastDebounceTime) > debounceDelay) {
        
        // Determine the new button state based on the current voltage
        String newButtonName = getButtonName(voltage, buttonMap);
        
        // If the button state has changed, update and print the new state
        if (newButtonName != state.currentButtonName) {
            Serial.print(name);
            Serial.print(" Button Change: ");
            Serial.print(state.currentButtonName);
            Serial.print(" -> ");
            Serial.println(newButtonName);
            
            // Update the button state
            state.currentButtonName = newButtonName;
            state.lastStableVoltage = voltage;
            state.lastDebounceTime = currentTime;
            
            // Send the button state change over I2C
            Wire.beginTransmission(I2C_ADDRESS);
            Wire.write(name[0]); // Send the button type (M or C)

            // Loop through the button name and send each character
            for (int i = 0; i < newButtonName.length(); i++) {
                Wire.write(newButtonName[i]);
            }

            // Finish transmission
            Wire.endTransmission();
        }
    }
}

void loop() {
    // Read and process Media buttons
    readAndPrintADC("Media", MEDIA_PIN, mediaButton, mediaButtonMap);
    
    // Read and process Cruise buttons
    readAndPrintADC("Cruise", CRUISE_PIN, cruiseButton, cruiseButtonMap);
    
    // Small delay to prevent excessive loop iterations and reduce power consumption
    delay(10);
}
