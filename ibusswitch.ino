/*
 * iBus RC Switch Decoder für ESP32
 * Basiert auf WMuCpp iBus Command Callback
 * 
 * Liest iBus RC-Signale ein und zeigt die codierten Schalter auf LEDs an
 */

// Keine zusätzlichen Libraries erforderlich - HardwareSerial ist bereits verfügbar

// Konfiguration
#define IBUS_RX_PIN 16       // Pin für iBus Datenempfang (RX2)
#define IBUS_TX_PIN 17       // Pin für iBus TX (nicht verwendet, aber für Serial2 erforderlich)
#define IBUS_BAUD 115200     // iBus Baudrate
#define DEFAULT_ADDRESS 0    // Standard-Adresse für Schalter
#define NUM_SWITCHES 8       // Anzahl der Schalter
#define FIRST_LED_PIN 2      // Erster LED Pin (2, 4, 5, 18, 19, 21, 22, 23 für 8 LEDs)

// LED Pin Array für ESP32 (sichere GPIO Pins)
const uint8_t ledPins[NUM_SWITCHES] = {2, 4, 5, 18, 19, 21, 22, 23};

// iBus Protokoll Konstanten
#define IBUS_FRAME_SIZE 32
#define IBUS_HEADER1 0x20
#define IBUS_HEADER2 0x40

class IBusSwitchDecoder {
private:
    uint8_t mLastCommand;
    uint8_t mAddress;
    uint8_t frameBuffer[IBUS_FRAME_SIZE];
    uint8_t frameIndex;
    bool frameStarted;
    
public:
    IBusSwitchDecoder(uint8_t address = DEFAULT_ADDRESS) 
        : mLastCommand(0), mAddress(address), frameIndex(0), frameStarted(false) {
        
        // LED Pins als Ausgänge konfigurieren
        for(int i = 0; i < NUM_SWITCHES; i++) {
            pinMode(ledPins[i], OUTPUT);
            digitalWrite(ledPins[i], LOW);
        }
    }
    
    void setAddress(uint8_t address) {
        mAddress = address;
    }
    
    void setLED(uint8_t switchNum, bool state) {
        if(switchNum < NUM_SWITCHES) {
            digitalWrite(ledPins[switchNum], state ? HIGH : LOW);
        }
    }
    
    void processData(uint8_t data) {
        // Suche nach Frame-Start (0x20 0x40)
        if(!frameStarted) {
            if(frameIndex == 0 && data == IBUS_HEADER1) {
                frameBuffer[frameIndex++] = data;
            }
            else if(frameIndex == 1 && data == IBUS_HEADER2) {
                frameBuffer[frameIndex++] = data;
                frameStarted = true;
            }
            else {
                frameIndex = 0;
                if(data == IBUS_HEADER1) {
                    frameBuffer[frameIndex++] = data;
                }
            }
            return;
        }
        
        // Frame-Daten sammeln
        frameBuffer[frameIndex++] = data;
        
        if(frameIndex >= IBUS_FRAME_SIZE) {
            // Frame vollständig - verarbeiten
            processFrame();
            frameIndex = 0;
            frameStarted = false;
        }
    }
    
private:
    void processFrame() {
        // Checksumme prüfen
        if(!verifyChecksum()) {
            Serial.println("Checksum error");
            return;
        }
        
        // Kanal 6 (Index 5) für Schalter-Kommandos verwenden
        // iBus Datenformat: Jeder Kanal verwendet 2 Bytes, little endian
        // Kanäle beginnen ab Byte 2 (nach den Headers)
        
        const uint8_t channelIndex = 5; // Kanal 6 (0-basiert)
        const uint8_t dataOffset = 2 + (channelIndex * 2); // Offset im Frame
        
        if(dataOffset + 1 >= IBUS_FRAME_SIZE - 2) return; // Sicherheitscheck
        
        // Kanal 6 Wert lesen (little endian)
        uint16_t ch6_value = frameBuffer[dataOffset] | (frameBuffer[dataOffset + 1] << 8);
        
        // Dekodierung basierend auf dem ursprünglichen Code
        // Aber angepasst für den direkten Kanalwert
        decodeCommand(ch6_value);
    }
    
    void decodeCommand(uint16_t ch6_value) {
        static constexpr uint16_t value0 = 988; // Basis-Offset
        
        // Normalisierung des Wertes
        const uint16_t n = (ch6_value >= value0) ? (ch6_value - value0 + 1) : 0;
        const uint8_t v = (n >> 4);
        
        // Kommando-Bits extrahieren
        const uint8_t address = (v >> 4) & 0b11;      // Adress-Bits
        const uint8_t sw = (v >> 1) & 0b111;          // Schalter-Nummer (0-7)
        const uint8_t state = v & 0b1;                // Schalter-Zustand (0/1)
        
        // Nur verarbeiten wenn sich das Kommando geändert hat
        if(v != mLastCommand) {
            mLastCommand = v;
            
            // Debug-Ausgabe
            Serial.print("CH6: ");
            Serial.print(ch6_value);
            Serial.print(", Addr: ");
            Serial.print(address);
            Serial.print(", Switch: ");
            Serial.print(sw);
            Serial.print(", State: ");
            Serial.println(state);
            
            // LED setzen wenn Adresse übereinstimmt
            if(address == mAddress) {
                const bool on = (state == 1);
                setLED(sw, on);
                
                Serial.print("LED ");
                Serial.print(sw);
                Serial.print(" -> ");
                Serial.println(on ? "ON" : "OFF");
            }
        }
    }
    
    bool verifyChecksum() {
        // iBus Checksumme: 0xFFFF - Summe aller Datenbytes
        uint16_t sum = 0;
        for(int i = 0; i < IBUS_FRAME_SIZE - 2; i++) {
            sum += frameBuffer[i];
        }
        
        uint16_t receivedChecksum = frameBuffer[IBUS_FRAME_SIZE - 2] | 
                                   (frameBuffer[IBUS_FRAME_SIZE - 1] << 8);
        uint16_t calculatedChecksum = 0xFFFF - sum;
        
        return (receivedChecksum == calculatedChecksum);
    }
};

// Globale Objekte
HardwareSerial iBusSerial(2); // UART2 verwenden
IBusSwitchDecoder decoder;

void setup() {
    Serial.begin(115200);
    Serial.println("iBus RC Switch Decoder für ESP32 gestartet");
    
    // iBus Serial initialisieren (UART2)
    iBusSerial.begin(IBUS_BAUD, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);
    
    // LED Test beim Start
    Serial.println("LED Test...");
    for(int i = 0; i < NUM_SWITCHES; i++) {
        decoder.setLED(i, true);
        delay(100);
    }
    for(int i = 0; i < NUM_SWITCHES; i++) {
        decoder.setLED(i, false);
        delay(100);
    }
    
    Serial.println("Warte auf iBus Daten...");
    Serial.println("Verkabelung:");
    Serial.print("iBus RX -> GPIO ");
    Serial.println(IBUS_RX_PIN);
    Serial.println("LEDs an GPIOs: 2, 4, 5, 18, 19, 21, 22, 23");
}

void loop() {
    // iBus Daten lesen und verarbeiten
    while(iBusSerial.available()) {
        uint8_t data = iBusSerial.read();
        decoder.processData(data);
    }
    
    // Kleine Pause um den Prozessor zu entlasten
    delay(1);
}

/*
 * Kommando-Format:
 * - Bits 7-6: Adresse (0-3)
 * - Bits 5-3: Schalter-Nummer (0-7) 
 * - Bit 0: Schalter-Zustand (0=AUS, 1=EIN)
 * 
 * Die Standard-Adresse ist 0. Ändern mit decoder.setAddress().
 */
