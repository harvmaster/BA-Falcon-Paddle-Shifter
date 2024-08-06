#include <Arduino.h>
#include <Wire.h>

const int I2C_ADDRESS = 0x04; // I2C address of this device

void setup() {
    // Initialize serial communication for debugging output
    Serial.begin(115200);
    delay(1000); // Short delay to ensure serial is ready

    // Initialize I2C communication
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveEvent);
}

void loop() {
    // Nothing to do here, we're just listening for I2C events
}

void receiveEvent(int bytesReceived) {
    char buttonType = Wire.read(); // Read the button type (M or C)
    int buttonNameLength = bytesReceived - 1; // Calculate the length of the button name
    char* buttonName = new char[buttonNameLength + 1]; // Allocate memory for the button name with space for a null termination character

    Wire.readBytes(buttonName, buttonNameLength); // Read the button name
    buttonName[buttonNameLength] = '\0'; // Add a null termination character to the end of the button name

    // Act on the button state change
    Serial.print("Received button state change: ");
    Serial.print(buttonType);
    Serial.print(" -> ");
    Serial.println(buttonName);

    delete[] buttonName; // Free the allocated memory
}

