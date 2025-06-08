#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

/*
NRF24L01 Pin Connections with NodeMCU:
----------------------------------------
NRF24L01    |    NodeMCU (ESP8266)
----------------------------------------
VCC         |    3.3V
GND         |    GND
CE          |    D4 (GPIO2)
CSN/CS      |    D8 (GPIO15)
SCK         |    D5 (GPIO14)
MOSI        |    D7 (GPIO13)
MISO        |    D6 (GPIO12)
----------------------------------------
*/

// Define the CE and CSN pins for NRF24L01
#define CE_PIN D4
#define CSN_PIN D8

// Create an RF24 object
RF24 radio(CE_PIN, CSN_PIN);

// Address through which two modules communicate
const byte address[6] = "00001";

// Buffer for message reconstruction
#define MAX_PACKETS 10
#define MAX_PACKET_SIZE 32
#define HEADER_SIZE 3

struct PacketBuffer {
    char data[MAX_PACKET_SIZE];
    bool received;
    uint8_t size;
};

PacketBuffer packets[MAX_PACKETS];
uint8_t expectedTotalPackets = 0;
unsigned long lastPacketTime = 0;
const unsigned long PACKET_TIMEOUT = 1000; // 1 second timeout

void resetPacketBuffer() {
    for (int i = 0; i < MAX_PACKETS; i++) {
        packets[i].received = false;
        packets[i].size = 0;
    }
    expectedTotalPackets = 0;
}

void printCompleteMessage() {
    Serial.println("\n=== Complete Message ===");
    for (int i = 0; i < expectedTotalPackets; i++) {
        if (packets[i].received) {
            for (int j = 0; j < packets[i].size; j++) {
                Serial.print(packets[i].data[j]);
            }
        }
    }
    Serial.println("\n====================");
}

bool allPacketsReceived() {
    if (expectedTotalPackets == 0) return false;
    for (int i = 0; i < expectedTotalPackets; i++) {
        if (!packets[i].received) return false;
    }
    return true;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        // Wait for serial port to connect
    }
    
    Serial.println("NRF24L01 Receiver Test");
    
    if (!radio.begin()) {
        Serial.println("Radio hardware not responding!");
        while (1) {}
    }
    
    Serial.println("Radio receiver initialized successfully");
    
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_1MBPS);
    radio.setChannel(108);
    radio.startListening();
    
    resetPacketBuffer();
    Serial.println("Listening for incoming messages...");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Check for timeout and reset if needed
    if (expectedTotalPackets > 0 && (currentTime - lastPacketTime) > PACKET_TIMEOUT) {
        Serial.println("Packet timeout - resetting buffer");
        resetPacketBuffer();
    }
    
    if (radio.available()) {
        uint8_t buffer[32];
        radio.read(&buffer, 32);
        
        // Extract header information
        uint8_t packetNum = buffer[0];
        uint8_t totalPackets = buffer[1];
        uint8_t payloadSize = buffer[2];
        
        // Validate packet
        if (totalPackets > MAX_PACKETS || payloadSize > (MAX_PACKET_SIZE - HEADER_SIZE)) {
            Serial.println("Invalid packet received");
            return;
        }
        
        // Update expected total if this is a new message
        if (expectedTotalPackets == 0) {
            expectedTotalPackets = totalPackets;
        }
        
        // Store packet data
        if (packetNum < totalPackets) {
            lastPacketTime = currentTime;
            packets[packetNum].received = true;
            packets[packetNum].size = payloadSize;
            
            // Copy payload data
            for (int i = 0; i < payloadSize; i++) {
                packets[packetNum].data[i] = (char)buffer[i + HEADER_SIZE];
            }
            
            // Debug output
            Serial.print("Received packet ");
            Serial.print(packetNum + 1);
            Serial.print("/");
            Serial.println(totalPackets);
            
            // If all packets received, print complete message
            if (allPacketsReceived()) {
                printCompleteMessage();
                resetPacketBuffer();
            }
        }
    }
    
    delay(10);
}