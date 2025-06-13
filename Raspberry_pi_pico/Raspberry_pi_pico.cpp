// Hardware Connections:
//
// INMP441 Microphone:
// - VDD  -> 3.3V (Pin 36 on Pico)
// - GND  -> GND  (Pin 38 on Pico)
// - SD   -> GP20 (Pin 26 on Pico) [DATA]
// - SCK  -> GP18 (Pin 24 on Pico) [BCLK]
// - WS   -> GP19 (Pin 25 on Pico) [LRCLK]
// - L/R  -> GND  (for left channel)
//
// LED Connection:
// - Anode   -> GP13 (Pin 17 on Pico) through a 220Ω resistor
// - Cathode -> GND  (any GND pin on Pico)
//
// FTDI Module Connection:
// - FTDI RX -> GP0 (Pin 1 on Pico, UART0 TX)
// - FTDI TX -> GP1 (Pin 2 on Pico, UART0 RX)
// - FTDI VCC -> 3.3V (Pin 36 on Pico)
// - FTDI GND -> GND (Pin 38 on Pico)
//
// NEO-6M GPS Module Connection:
// - VCC -> 3.3V (Pin 36 on Pico)
// - GND -> GND (Pin 38 on Pico)
// - TX  -> GP9 (Pin 11 on Pico, UART1 RX)
// - RX  -> GP8 (Pin 12 on Pico, UART1 TX)
//
// NRF24L01 Module Connection:
// - VCC -> 3.3V (Pin 36 on Pico)
// - GND -> GND (Pin 38 on Pico)
// - SCK -> GP2 (Pin 4 on Pico)
// - MOSI -> GP3 (Pin 5 on Pico)
// - MISO -> GP4 (Pin 6 on Pico)
// - CSN -> GP5 (Pin 7 on Pico)
// - CE -> GP6 (Pin 9 on Pico)
// - IRQ -> GP7 (Pin 10 on Pico) (optional, can be used for interrupts)
//
// Power Control MH-CD42 Module:
// - Key -> GP17 (Pin 22 on Pico) (PWM) [Very importent, used to power to Pico]
//

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>  // For strncmp
#include <math.h>    // For fabs
#include <time.h>    // For time functions
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/mutex.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/uart.h"
#include "hardware/spi.h"
#include "inmp441.pio.h"
#include "lib/ml_model/model_handler.h" 
#include "lib/Pico_NRF24L01/NRF24/NRF24.h"  // Include the NRF24L01 library
#include "lib/Pico_NRF24L01/NRF24/NRF24_config.h"  // Include the NRF24 config
#include "power_control.pio.h"  // This will be generated from power_control.pio


// Configuration defines for UART and GPIO
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define UART_ID uart0
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

// GPS Configuration
#define GPS_UART_ID uart1
#define GPS_UART_TX_PIN 8
#define GPS_UART_RX_PIN 9
#define GPS_BAUD_RATE 9600
#define GPS_DATA_BITS 8
#define GPS_STOP_BITS 1
#define GPS_PARITY    UART_PARITY_NONE
#define GPS_BUFFER_SIZE 256

// GPIO Configuration
#define INMP441_SCK_PIN 18  // BCLK
#define INMP441_WS_PIN  19  // LRCLK/WS
#define INMP441_SD_PIN  20  // DATA

// LED Configuration
#define LED_INBUILT_IN 25   // Built-in LED
#define LED_PIN 13         // External LED for gunshot indication

// Audio Configuration
#define SAMPLE_RATE 16000
#define SAMPLE_BUFFER_SIZE 1024
#define DMA_CHANNEL 0
#define INMP441_PIO pio0
#define INMP441_SM  0
#define DEBUG_SAMPLES 32
#define MIC_TEST_DURATION_MS 5000

// NRF24L01 Configuration
#define NRF_SPI_PORT spi0
#define NRF_SCK_PIN 2  // SPI Clock
#define NRF_MOSI_PIN 3 // SPI MOSI
#define NRF_MISO_PIN 4 // SPI MISO
#define NRF_CSN_PIN 5  // SPI Chip Select
#define NRF_CE_PIN 6   // Chip Enable
#define NRF_IRQ_PIN 7  // IRQ pin (optional)

// MD-CD42 Power Control Configuration
#define POWER_CONTROL_PIN 17   // Key pin for MH-CD42 module
#define POWER_CONTROL_PIO pio1 // Using pio1 to avoid conflicts
#define POWER_CONTROL_SM 1     // State machine 1
#define POWER_FREQ 2           //  frequency
#define POWER_DUTY_CYCLE 30    // 50% duty cycle

// NRF24L01 Register Definitions
#define NRF_CONFIG      0x00
#define NRF_STATUS      0x07
#define NRF_RF_SETUP    0x06
#define NRF_RF_CH       0x05
#define NRF_TX_ADDR     0x10
#define NRF_RX_ADDR_P0  0x0A
#define NRF_FIFO_STATUS 0x17

// Register Read/Write commands
#define NRF_R_REGISTER    0x00
#define NRF_W_REGISTER    0x20
#define NRF_NOP           0xFF

// Data rates (RF_SETUP register values)
#define RF_DR_2MBPS     0x08  // 2 Mbps mode (default)
#define RF_DR_1MBPS     0x00  // 1 Mbps mode
#define RF_DR_250KBPS   0x20  // 250 kbps mode

// NRF24L01 Power Levels
#define RF_PWR_0DBM    0x06  // Maximum power: 0dBm
#define RF_PWR_NEG_6DBM  0x04  // -6dBm
#define RF_PWR_NEG_12DBM 0x02  // -12dBm
#define RF_PWR_NEG_18DBM 0x00  // Minimum power: -18dBm

#define NODE_ID "01"    // Node identifier
#define MAX_PAYLOAD_SIZE 32  // NRF24L01 max payload size

// GPS location structure
struct gps_location_t {
    bool valid;
    float latitude;
    float longitude;
} g_current_location = {false, 0.0f, 0.0f};

// Message structure for transmitting data
typedef struct {
    char node_id[3];        // 2 chars + null terminator
    char date[11];          // DD/MM/YYYY + null terminator
    char time[9];           // HH:MM:SS + null terminator
    char latitude[12];      // Including decimal point and sign
    char longitude[12];     // Including decimal point and sign
    float confidence;
} gunshot_message_t;


// GPS Message types we're interested in
#define NMEA_GPRMC "$GPRMC"
#define NMEA_GPGGA "$GPGGA"

// Global variables and mutex
static mutex_t printf_mutex;
volatile bool core1_ready = false;
volatile bool gunshot_detected = false;

// GPS globals
static char gps_buffer[GPS_BUFFER_SIZE];
static uint line_pos = 0;  // Current position in line buffer
static mutex_t gps_mutex;
static bool start_of_sentence = false;
static uint32_t last_valid_sentence = 0;
static uint32_t sentence_count = 0;
static bool valid_gps_data = false;
static float latitude = 0.0f;
static float longitude = 0.0f;

// Audio buffer
static int32_t audio_buffer[SAMPLE_BUFFER_SIZE];

// NRF24L01 instance
NRF24 radio;  // Global radio instance

// Forward declarations
void init_sync(void);
void safe_uart_puts(const char* str);
void safe_printf(const char* format, ...);
void init_uart(void);
bool init_microphone(void);
void core1_entry(void);
bool test_microphone(void);
static inline void configure_dma(void);
void init_gps(void);
void process_gps(void);
bool wait_for_gps_fix(uint32_t timeout_ms);
bool init_nrf24l01(void);
void nrf24l01_send_message(const char* message);
void create_json_message(char* buffer, size_t buffer_size, const gunshot_message_t* msg);
bool init_power_control(void);
void update_power_control(float freq, float duty_cycle);


// Convert NMEA coordinate format to decimal degrees
float nmea_to_decimal_degrees(float nmea_coord, char direction) {
    // Extract degrees (before decimal) and minutes (after decimal)
    int degrees = (int)(nmea_coord / 100.0f);  // First 2 or 3 digits
    float minutes = nmea_coord - (degrees * 100.0f);  // Remaining digits
    float decimal = degrees + (minutes / 60.0f);  // Convert minutes to decimal degrees    
    return (direction == 'S' || direction == 'W') ? -decimal : decimal;
}

// Parse GPRMC sentence Returns true if parsing successful and fix is valid
bool parse_gprmc(const char* sentence, float* out_lat, float* out_lon, bool* out_valid) {
    char field_buffer[32];  // Increased buffer for safety
    const char* ptr = sentence;
    int field = 0;
    int pos = 0;
    float nmea_lat = 0, nmea_lon = 0;
    char lat_dir = 'N', lon_dir = 'E';
    *out_valid = false;

    // Skip $GPRMC
    while (*ptr && *ptr != ',') ptr++;
    if (!*ptr) return false;
    ptr++;

    // Parse fields
    while (*ptr && field < 7) {  // We only need first 7 fields
        if (*ptr == ',' || *ptr == '*') {
            field_buffer[pos] = '\0';
            switch(field) {
                case 1:  // Status
                    *out_valid = (field_buffer[0] == 'A');
                    break;
                case 2:  // Latitude DDMM.MMMMM
                    nmea_lat = atof(field_buffer);
                    break;
                case 3:  // N/S
                    lat_dir = field_buffer[0];
                    break;
                case 4:  // Longitude DDDMM.MMMMM
                    nmea_lon = atof(field_buffer);
                    break;
                case 5:  // E/W
                    lon_dir = field_buffer[0];
                    break;
            }
            pos = 0;
            field++;
            ptr++;
            continue;
        }
        if (pos < sizeof(field_buffer) - 1) {
            field_buffer[pos++] = *ptr;
        }
        ptr++;
    }

    if (*out_valid) {
        *out_lat = nmea_to_decimal_degrees(nmea_lat, lat_dir);
        *out_lon = nmea_to_decimal_degrees(nmea_lon, lon_dir);
        return true;
    }
    
    safe_uart_puts("Invalid fix\r\n");
    return false;
}

// DMA configuration
static inline void configure_dma(void) {
    safe_uart_puts("   Starting DMA configuration...\r\n");
    
    // Get default channel configuration
    dma_channel_config c = dma_channel_get_default_config(DMA_CHANNEL);
    
    char buf[64];
    snprintf(buf, sizeof(buf), "   Got default DMA config for channel %d\r\n", DMA_CHANNEL);
    safe_uart_puts(buf);
    
    // Configure channel
    channel_config_set_read_increment(&c, false);  // Don't increment read address (reading from same PIO FIFO)
    channel_config_set_write_increment(&c, true);  // Do increment write address (writing to buffer)
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);  // Transfer 32-bit words
    channel_config_set_dreq(&c, pio_get_dreq(INMP441_PIO, INMP441_SM, false));  // Pace transfers based on PIO

    dma_channel_configure(
        DMA_CHANNEL,
        &c,
        audio_buffer,                    // Destination pointer
        &INMP441_PIO->rxf[INMP441_SM],  // Source pointer
        SAMPLE_BUFFER_SIZE,              // Number of transfers
        true                            // Start immediately
    );
}

// Initialize mutex for printf synchronization
void init_sync(void) {
    mutex_init(&printf_mutex);
}

// Safe UART output function
void safe_uart_puts(const char* str) {
    mutex_enter_blocking(&printf_mutex);
    uart_puts(UART_ID, str);
    mutex_exit(&printf_mutex);
}

// Safe printf function
void safe_printf(const char* format, ...) {
    mutex_enter_blocking(&printf_mutex);
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    mutex_exit(&printf_mutex);
}

// Initialize UART for FTDI communication
void init_uart(void) {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);
    uart_puts(UART_ID, "\r\n\nUART Test - If you see this, UART is working!\r\n");
    
    // Initialize model handler
    int init_status = model_init();
    if (init_status != 0) {
        safe_uart_puts("ERROR: Failed to initialize model handler\r\n");
    } else {
        safe_uart_puts("Model handler initialized successfully\r\n");
    }
}

// Override putchar for printf redirection to UART with mutex protection
int putchar(int ch) {
    mutex_enter_blocking(&printf_mutex);
    uart_putc(UART_ID, (char)ch);
    mutex_exit(&printf_mutex);
    return ch;
}

// Test microphone connectivity
bool test_microphone() {
    safe_uart_puts("\r\nStarting microphone test...\r\n");
    safe_uart_puts("\r\nTesting INMP441 Microphone:\r\n");
    safe_uart_puts("1. Checking PIO configuration...\r\n");
    safe_uart_puts("Checking PIO program size...\r\n");
    
    if (!pio_can_add_program(INMP441_PIO, &inmp441_program)) {
        safe_uart_puts("ERROR: Cannot load PIO program - insufficient space\r\n");
        return false;
    }
    
    safe_uart_puts("2. Loading PIO program...\r\n");
    uint offset = pio_add_program(INMP441_PIO, &inmp441_program);
    
    char buf[128];
    snprintf(buf, sizeof(buf), "   Program loaded at offset %u\r\n", offset);
    safe_uart_puts(buf);
    
    safe_uart_puts("3. Initializing GPIO pins...\r\n");
    snprintf(buf, sizeof(buf), "   SCK (BCLK): GPIO %d\r\n", INMP441_SCK_PIN);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "   WS (LRCLK): GPIO %d\r\n", INMP441_WS_PIN);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "   SD (DATA):  GPIO %d\r\n", INMP441_SD_PIN);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "   Sample Rate: %d Hz\r\n", SAMPLE_RATE);
    safe_uart_puts(buf);
    
    // Initialize the PIO state machine
    snprintf(buf, sizeof(buf), "   Initializing PIO state machine %d on PIO%d\r\n", 
                INMP441_SM, (INMP441_PIO == pio0) ? 0 : 1);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "   Clock frequency: %lu Hz\r\n", clock_get_hz(clk_sys));
    safe_uart_puts(buf);
    
    inmp441_program_init(INMP441_PIO, INMP441_SM, offset, SAMPLE_RATE, INMP441_SD_PIN, INMP441_SCK_PIN);
    safe_uart_puts("   PIO initialization complete\r\n");
    
    safe_uart_puts("4. Configuring DMA...\r\n");
    snprintf(buf, sizeof(buf), "   DMA Channel: %d\r\n", DMA_CHANNEL);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "   Buffer size: %d samples\r\n", SAMPLE_BUFFER_SIZE);
    safe_uart_puts(buf);
    configure_dma();
    safe_uart_puts("   DMA configuration complete\r\n");
    
    safe_uart_puts("5. Starting audio capture test...\r\n");
    safe_uart_puts("   Please make some noise near the microphone!\r\n\r\n");
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    bool received_audio = false;
    int32_t min_sample = 0x7FFFFFFF;
    int32_t max_sample = -0x7FFFFFFF;
    
    safe_uart_puts("   Waiting for DMA transfer...\r\n");

    while (to_ms_since_boot(get_absolute_time()) - start_time < MIC_TEST_DURATION_MS) {
        dma_channel_wait_for_finish_blocking(DMA_CHANNEL);
        
        
        // Process samples
        for (int i = 0; i < DEBUG_SAMPLES; i++) {
            int32_t sample = audio_buffer[i] >> 8;  // Convert to 24-bit
            if (sample != 0) {
                received_audio = true;
            }
            if (sample < min_sample) min_sample = sample;
            if (sample > max_sample) max_sample = sample;
        }
        
        // Restart DMA
        dma_channel_set_write_addr(DMA_CHANNEL, audio_buffer, true);
        
        if (received_audio) {
            safe_uart_puts("   DMA transfer complete\r\n");
            break;  // Got some samples, no need to wait longer
        }
    }
    
    safe_uart_puts("\r\nMicrophone Test Results:\r\n");
    snprintf(buf, sizeof(buf), "- Audio data received: %s\r\n", received_audio ? "YES" : "NO");
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "- Min sample value: %ld\r\n", min_sample);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "- Max sample value: %ld\r\n", max_sample);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "- Sample range: %ld\r\n", max_sample - min_sample);
    safe_uart_puts(buf);
    
    if (!received_audio) {
        safe_uart_puts("\r\nERROR: No audio data received. Please check:\r\n");
        safe_uart_puts("1. All connections are secure\r\n");
        safe_uart_puts("2. VDD and GND are properly connected\r\n");
        safe_uart_puts("3. L/R pin is connected to GND\r\n");
        return false;
    }
    
    if (max_sample - min_sample < 1000) {
        safe_uart_puts("\r\nWARNING: Very low audio range detected.\r\n");
        safe_uart_puts("Try making louder sounds near the microphone.\r\n");
    } else {
        safe_uart_puts("\r\nMicrophone test PASSED! ✓\r\n");
    }
    
    return true;
}

// Initialize INMP441 microphone
bool init_microphone() {
    // First run the test
    if (!test_microphone()) {
        safe_uart_puts("\r\nMicrophone initialization failed! Check connections and try again.\r\n");
        return false;
    }
    
    // No need to reconfigure since test_microphone already set everything up
    return true;
}

// Initialize UART for GPS
void gps_uart_irq_handler() {
    while (uart_is_readable(GPS_UART_ID)) {
        uint8_t ch = uart_getc(GPS_UART_ID);
        
        // // Echo character to debug UART
        // uart_putc(UART_ID, ch);
        
        // Store in buffer for NMEA sentence processing
        if (line_pos < GPS_BUFFER_SIZE - 1) {
            gps_buffer[line_pos++] = ch;
            
            // Check for line ending
            if (ch == '\n' || ch == '\r') {
                if (line_pos > 1) {  // Ensure we have more than just the newline
                    gps_buffer[line_pos] = '\0';  // Null terminate
                    process_gps();  // Process the complete NMEA sentence
                }
                line_pos = 0;  // Reset for next sentence
            }
        } else {
            // Buffer overflow - reset
            line_pos = 0;
        }
    }
}

void init_gps(void) {
    safe_uart_puts("\r\n=== GPS Initialization Starting ===\r\n");
    
    // Initialize UART for GPS
    uart_init(GPS_UART_ID, GPS_BAUD_RATE);
    uart_set_hw_flow(GPS_UART_ID, false, false);
    uart_set_format(GPS_UART_ID, GPS_DATA_BITS, GPS_STOP_BITS, GPS_PARITY);
    
    // Set UART pins
    gpio_set_function(GPS_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_UART_RX_PIN, GPIO_FUNC_UART);
    
    // Force immediate RX, no buffering
    uart_set_fifo_enabled(GPS_UART_ID, false);
    
    // Reset UART and clear any pending data
    uart_is_readable(GPS_UART_ID);  // Clear RX flag
    while (uart_is_readable(GPS_UART_ID)) {
        uart_getc(GPS_UART_ID);  // Clear any pending data
    }
    
    // Initialize mutex
    mutex_init(&gps_mutex);
    
    // Set up and enable interrupt handler
    int GPS_UART_ID_IRQ = UART1_IRQ;  // UART1 for GPS
    irq_set_exclusive_handler(GPS_UART_ID_IRQ, gps_uart_irq_handler);
    irq_set_enabled(GPS_UART_ID_IRQ, true);
    
    // Enable UART to send interrupts - RX only
    uart_set_irq_enables(GPS_UART_ID, true, false);
    
    safe_uart_puts("GPS UART initialized with interrupt handling\r\n");
    safe_uart_puts("All received characters will be echoed:\r\n\r\n");
}

bool wait_for_gps_fix(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    const uint32_t PRINT_INTERVAL = 1000;   // Status update every second
    const uint32_t STATUS_INTERVAL = 100;   // Detailed status every 100ms

    char buf[256];  // Buffer for formatting messages

    // Print initial status
    snprintf(buf, sizeof(buf), "\r\n=== Starting GPS Fix Wait ===\r\n");
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "Timeout set to: %lu ms\r\n", timeout_ms);
    safe_uart_puts(buf);

    int attempts = 0;
    while (!g_current_location.valid) {
        attempts++;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Check timeout
        if (now - start >= timeout_ms) {
            snprintf(buf, sizeof(buf), "\r\n*** GPS Fix Timeout ***\r\n");
            safe_uart_puts(buf);
            snprintf(buf, sizeof(buf), "- Time elapsed: %d seconds\r\n", timeout_ms / 1000);
            safe_uart_puts(buf);
            snprintf(buf, sizeof(buf), "- Total attempts: %d\r\n", attempts);
            safe_uart_puts(buf);
            snprintf(buf, sizeof(buf), "- Last known state:\r\n");
            safe_uart_puts(buf);
            snprintf(buf, sizeof(buf), "  Valid: %s\r\n", g_current_location.valid ? "YES" : "NO");
            safe_uart_puts(buf);
            snprintf(buf, sizeof(buf), "  Latitude: %.6f\r\n", g_current_location.latitude);
            safe_uart_puts(buf);
            snprintf(buf, sizeof(buf), "  Longitude: %.6f\r\n", g_current_location.longitude);
            safe_uart_puts(buf);
            return false;
        }
        
        sleep_ms(50);  // Check more frequently but don't hog CPU
    }
    
    // GPS fix obtained!
    uint32_t end = to_ms_since_boot(get_absolute_time());
    snprintf(buf, sizeof(buf), "\r\n=== GPS Fix Obtained! ===\r\n");
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "- Time taken: %lu ms\r\n", end - start);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "- Total attempts: %d\r\n", attempts);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "- Position:\r\n");
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "  Latitude: %.6f\r\n", g_current_location.latitude);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "  Longitude: %.6f\r\n", g_current_location.longitude);
    safe_uart_puts(buf);
    snprintf(buf, sizeof(buf), "=========================\r\n\r\n");
    safe_uart_puts(buf);
    return true;
}

// Initialize NRF24L01 module
bool init_nrf24l01(void) {
    safe_uart_puts("\r\nInitializing NRF24L01...\r\n");
    
    // Configure SPI first
    spi_init(NRF_SPI_PORT, 4000000);  // 4 MHz
    gpio_set_function(NRF_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(NRF_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(NRF_MISO_PIN, GPIO_FUNC_SPI);
    
    // Setup pinout for radio
    _NRF_SPI_PINOUT pinout = {
        .ce = NRF_CE_PIN,
        .csn = NRF_CSN_PIN,
        .irq = NRF_IRQ_PIN
    };
    
    // Initialize radio with module ID 0 and pins
    radio.Init(0, pinout);
    
    // Power cycle the radio
    radio.PowerDown();
    sleep_ms(100);  // Longer delay for stable power down
    radio.PowerUp();
    sleep_ms(100);  // Longer delay for stable power up
    
    // More thorough check for NRF24L01 presence:
    // 1. Check if we can write and read back a value to the RF channel register
    radio.WriteReg(RF_CH, 83);  // Write a specific value
    uint8_t readback = radio.ReadReg(RF_CH);
    if (readback != 83) {
        safe_uart_puts("ERROR: NRF24L01 not detected (RF_CH register test failed)\r\n");
        safe_printf("Expected: 83, Got: %d\r\n", readback);
        return false;
    }
    
    // 2. Check if STATUS register has a valid value (should be 0x0E or 0x0F on power up)
    uint8_t status = radio.ReadReg(STATUS);
    if (status == 0 || status == 0xFF) {
        safe_uart_puts("ERROR: NRF24L01 not detected (invalid STATUS register)\r\n");
        safe_printf("STATUS register: 0x%02X\r\n", status);
        return false;
    }
    
    safe_uart_puts("NRF24L01 detected and responding!\r\n");
    
    // Enhanced configuration
    radio.WriteReg(EN_AA, 0x01);       // Enable auto-acknowledgment for pipe 0
    radio.WriteReg(SETUP_RETR, 0x4F);  // 1500us delay (0x4), 15 retries (0xF)
    
    // Configure radio settings
    uint8_t address[5] = {'0','0','0','0','1'};  // 5-byte address
    radio.OpenWritingPipe(address);     // Set transmit address
    radio.SetChannel(108);              // Set RF channel to 108
    radio.SetPayloadSize(32);           // Set payload size to max
    
    // Configure RF setup - Data rate and Power
    uint8_t rf_setup = RF_DR_1MBPS  | RF_PWR_0DBM;  // 250kbps and maximum power (0dBm)
    radio.WriteReg(RF_SETUP, rf_setup);
    
    // Clear interrupt flags
    radio.WriteReg(STATUS, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
    
    // Configure for enhanced reliability
    radio.WriteReg(CONFIG, 0x0E);       // Enable CRC, 2-byte CRC, Power up, TX mode
    radio.FlushTX();                    // Clear TX FIFO
    radio.FlushRX();                    // Clear RX FIFO
    
    // Wait for radio to power up and stabilize
    sleep_ms(5);
    
    // Verify configuration
    if (radio.ReadReg(CONFIG) != 0x0E) {
        safe_uart_puts("ERROR: Failed to configure NRF24L01\r\n");
        return false;
    }
    
    safe_uart_puts("NRF24L01 module initialized with enhanced settings\r\n");
    safe_uart_puts("- Address: 00001\r\n");
    safe_uart_puts("- Channel: 108\r\n");
    safe_uart_puts("- Data Rate: 1Mbps\r\n");
    safe_uart_puts("- Power Level: 0dBm (Maximum)\r\n");
    safe_uart_puts("- Auto-ACK: Enabled\r\n");
    safe_uart_puts("- CRC: 2 bytes\r\n");
    safe_uart_puts("- Retransmit: 15 attempts, 1500us delay\r\n");
    
    return true;
}

// Utility function to print packet contents
void print_packet(const uint8_t* packet, size_t size) {
    char debug_buf[128];
    // Print header info
    snprintf(debug_buf, sizeof(debug_buf), "Packet Header: [%02X %02X %02X] (packet_num=%d, total=%d, size=%d)\r\n",
             packet[0], packet[1], packet[2], packet[0], packet[1], packet[2]);
    safe_uart_puts(debug_buf);
    uart_tx_wait_blocking(UART_ID);

    // Print payload in ASCII
    safe_uart_puts("\r\nPayload (ASCII): ");
    uart_tx_wait_blocking(UART_ID);
    for (size_t i = 3; i < size; i++) {
        // Print printable characters, dot for non-printable
        char c = (packet[i] >= 32 && packet[i] <= 126) ? packet[i] : '.';
        snprintf(debug_buf, sizeof(debug_buf), "%c", c);
        safe_uart_puts(debug_buf);
        uart_tx_wait_blocking(UART_ID);
    }
    safe_uart_puts("\r\n");
    uart_tx_wait_blocking(UART_ID);
}

// Send message via NRF24L01 with fragmentation support
void nrf24l01_send_message(const char* message) {
    // First verify the module is still responding and in proper state
    uint8_t config = radio.ReadReg(CONFIG);
    if (config == 0 || config == 0xFF) {
        safe_uart_puts("ERROR: NRF24L01 not responding - check connections\r\n");
        uart_tx_wait_blocking(UART_ID);
        return;
    }

    size_t total_len = strlen(message);
    const size_t PACKET_HEADER_SIZE = 3; // [packet_num, total_packets, payload_size]
    const size_t MAX_PACKET_PAYLOAD = MAX_PAYLOAD_SIZE - PACKET_HEADER_SIZE;
    size_t total_packets = (total_len + MAX_PACKET_PAYLOAD - 1) / MAX_PACKET_PAYLOAD;

    char debug_buf[128];
    snprintf(debug_buf, sizeof(debug_buf), "\r\nStarting message transmission:\r\n- Total length: %d bytes\r\n- Packets needed: %d\r\n- Max payload per packet: %d bytes\r\n\r\n", 
             total_len, total_packets, MAX_PACKET_PAYLOAD);
    safe_uart_puts(debug_buf);
    uart_tx_wait_blocking(UART_ID);
    
    // Ensure we're in TX mode
    radio.SetTXMode(true);
    sleep_ms(2);
    
    // Break message into packets and send each one
    for (size_t packet_num = 0; packet_num < total_packets; packet_num++) {
        size_t offset = packet_num * MAX_PACKET_PAYLOAD;
        size_t remaining = total_len - offset;
        size_t payload_size = (remaining > MAX_PACKET_PAYLOAD) ? MAX_PACKET_PAYLOAD : remaining;
        
        // Prepare packet with header
        uint8_t packet[MAX_PAYLOAD_SIZE];
        packet[0] = packet_num;        // Current packet number
        packet[1] = total_packets;     // Total number of packets
        packet[2] = payload_size;      // Size of payload in this packet
        memcpy(&packet[PACKET_HEADER_SIZE], message + offset, payload_size);
        
        snprintf(debug_buf, sizeof(debug_buf), "\r\n=== Packet %d/%d ===\r\n", packet_num + 1, total_packets);
        safe_uart_puts(debug_buf);
        uart_tx_wait_blocking(UART_ID);
        
        // Print packet contents before sending
        print_packet(packet, payload_size + PACKET_HEADER_SIZE);
        uart_tx_wait_blocking(UART_ID);
        
        // Send packet with enhanced retry logic
        const int MAX_RETRIES = 5;
        const int RETRY_DELAY_MS = 50;
        int retries = MAX_RETRIES;
        bool sent = false;

        while (retries > 0 && !sent) {
            // Clear any pending interrupts
            radio.WriteReg(STATUS, radio.ReadReg(STATUS) | _BV(MAX_RT) | _BV(TX_DS));
            radio.FlushTX();
            
            // Attempt to send
            sent = radio.Write(packet, payload_size + PACKET_HEADER_SIZE);
            
            if (!sent) {
                snprintf(debug_buf, sizeof(debug_buf), "Transmission failed, retry %d of %d...\r\n", 
                        MAX_RETRIES - retries + 1, MAX_RETRIES);
                safe_uart_puts(debug_buf);
                uart_tx_wait_blocking(UART_ID);
                
                // Check if MAX_RT was triggered
                uint8_t status = radio.ReadReg(STATUS);
                if (status & _BV(MAX_RT)) {
                    safe_uart_puts("Maximum retransmissions reached, resetting...\r\n");
                    uart_tx_wait_blocking(UART_ID);
                    radio.WriteReg(STATUS, status | _BV(MAX_RT));
                    radio.FlushTX();
                }
                
                // Power cycle on persistent failure
                if (retries == 2) {
                    safe_uart_puts("Attempting radio reset...\r\n");
                    uart_tx_wait_blocking(UART_ID);
                    radio.PowerDown();
                    sleep_ms(5);
                    radio.PowerUp();
                    sleep_ms(5);
                    radio.FlushTX();
                }
                
                sleep_ms(RETRY_DELAY_MS * (MAX_RETRIES - retries + 1));
                retries--;
            }
        }
        
        if (!sent) {
            safe_uart_puts("\r\nFATAL: Packet transmission failed after all retries\r\n");
            uart_tx_wait_blocking(UART_ID);
            return;
        }
        
        snprintf(debug_buf, sizeof(debug_buf), "Packet %d/%d sent successfully\r\n", packet_num + 1, total_packets);
        safe_uart_puts(debug_buf);
        uart_tx_wait_blocking(UART_ID);
        sleep_ms(20);  // Small delay between packets
    }
    
    snprintf(debug_buf, sizeof(debug_buf), "\r\nComplete message sent successfully (%d packets)\r\n", total_packets);
    safe_uart_puts(debug_buf);
    uart_tx_wait_blocking(UART_ID);
}

// Create JSON message from data
void create_json_message(char* buffer, size_t buffer_size, const gunshot_message_t* msg) {
    snprintf(buffer, buffer_size, 
            "{\"node_id\":\"%s\",\"date\":\"%s\",\"time\":\"%s\",\"latitude\":\"%s\",\"longitude\":\"%s\",\"confidence\":%.2f}",
            msg->node_id, msg->date, msg->time, msg->latitude, msg->longitude, msg->confidence);
}

// Process GPS data - Now called from interrupt handler for complete sentences
void process_gps() {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    
    // Check for GPRMC sentence
    if (strncmp(gps_buffer, "$GPRMC", 6) == 0) {
        sentence_count++;
        float lat, lon;
        bool valid;
        
        if (parse_gprmc(gps_buffer, &lat, &lon, &valid)) {
            mutex_enter_blocking(&gps_mutex);
            g_current_location.valid = valid;
            if (valid) {
                g_current_location.latitude = lat;
                g_current_location.longitude = lon;
                valid_gps_data = true;
                latitude = lat;
                longitude = lon;
                last_valid_sentence = now;
            }
            mutex_exit(&gps_mutex);
        }
    }
    
    // Print periodic status if we haven't had a valid sentence in a while
    static uint32_t last_status = 0;
    if (now - last_status >= 5000) {  // Every 5 seconds
        safe_printf("\r\n[GPS] Status: Sentences: %lu, Last valid: %lu ms ago\r\n", 
                   sentence_count,
                   now - last_valid_sentence);
        last_status = now;
    }
}

// Core 1 entry point - Audio processing
void core1_entry() {
    safe_uart_puts("\r\nCore 1: Starting...\r\n");
    sleep_ms(100); // Give time for UART message
    
    safe_uart_puts("Core 1: Initializing microphone...\r\n");
    sleep_ms(100); // Give time for UART message
    
    // Initialize microphone
    safe_uart_puts("Core 1: Starting microphone initialization...\r\n");
    bool init_success = init_microphone();
    safe_uart_puts("Core 1: Microphone initialization attempt complete\r\n");
    
    if (!init_success) {
        safe_uart_puts("Core 1: Microphone initialization failed\r\n");
        safe_uart_puts("Core 1: Check hardware connections and try again\r\n");
        core1_ready = true;  // Signal core0 even on failure so it doesn't hang
        while (true) {
            gpio_put(LED_PIN, true);   // Error indication pattern
            sleep_ms(100);
            gpio_put(LED_PIN, false);
            sleep_ms(900);             // Blink once per second to show core is alive
        }
    }
    
    safe_uart_puts("Core 1: Microphone initialized successfully\r\n");
    core1_ready = true;  // Signal core0 that we're ready
    
    while (true) {
        // Wait for DMA transfer to complete
        dma_channel_wait_for_finish_blocking(DMA_CHANNEL);
        
        // Process audio data using the model
        bool detection = model_process_audio(audio_buffer, SAMPLE_BUFFER_SIZE);
        float confidence = model_get_confidence();
        
        // Print amplitude level every 1000ms (1 second)
        static uint32_t last_print = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print > 1000) {
            char buf[64];
            snprintf(buf, sizeof(buf), "Audio Confidence: %.2f\r\n", confidence);
            safe_uart_puts(buf);
            last_print = now;
        }

        // Map confidence to LED brightness
        uint8_t led_brightness = (uint8_t)(confidence * 255);
        
        gpio_put(LED_PIN, led_brightness > 90);  // Turn on LED if confidence > 0.5
        // If detection threshold exceeded, trigger gunshot detection
        if (detection) {
            gunshot_detected = true;
            char buf[64];
            snprintf(buf, sizeof(buf), "GUNSHOT DETECTED! Confidence: %.2f\r\n", confidence);
            safe_uart_puts(buf);
            snprintf(buf, sizeof(buf), "Latitude: %.6f, Longitude: %.6f\r\n", latitude, longitude);
            safe_uart_puts(buf);
            
            // Prepare and send message via NRF24L01
            gunshot_message_t msg;
            strncpy(msg.node_id, NODE_ID, sizeof(msg.node_id) - 1);
            msg.node_id[sizeof(msg.node_id) - 1] = '\0';  // Ensure null termination
            
            // Get current time
            time_t now = time(NULL);
            struct tm *t = localtime(&now);
            strftime(msg.date, sizeof(msg.date), "%d/%m/%Y", t);
            strftime(msg.time, sizeof(msg.time), "%H:%M:%S", t);
            
            // Set latitude, longitude, and confidence
            snprintf(msg.latitude, sizeof(msg.latitude), "%.6f", latitude);
            snprintf(msg.longitude, sizeof(msg.longitude), "%.6f", longitude);
            msg.confidence = confidence;
            
            // Create JSON message
            char json_buffer[1024];
            create_json_message(json_buffer, sizeof(json_buffer), &msg);
            
            // Send message
            nrf24l01_send_message(json_buffer);
            
            sleep_ms(1000); // Prevent multiple detections
            gunshot_detected = false;
        }
        
        // Restart DMA transfer
        dma_channel_set_write_addr(DMA_CHANNEL, audio_buffer, true);
    }
}

// Core 0 entry point - LED control
int main() {
    init_sync();
    init_uart();
    sleep_ms(20);
    
    // Initialize power control first to ensure system stays powered
    if (!init_power_control()) {
        // If power control fails, try to indicate error with LED
        gpio_init(LED_INBUILT_IN);
        gpio_set_dir(LED_INBUILT_IN, GPIO_OUT);
        while(1) {
            gpio_put(LED_INBUILT_IN, !gpio_get(LED_INBUILT_IN));
            sleep_ms(100); // Fast blink to indicate error
        }
    }
    
    // Initialize LED for visual debugging
    gpio_init(LED_INBUILT_IN);
    gpio_set_dir(LED_INBUILT_IN, GPIO_OUT);
    gpio_put(LED_INBUILT_IN, true);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);  // Turn off LED initially
    
    safe_uart_puts("\r\n=== Program Starting (GPS Debug Mode) ===\r\n");
    safe_uart_puts("Initializing...\r\n");

    // Initialize GPS first
    safe_uart_puts("Initializing GPS...\r\n");
    init_gps();
    safe_uart_puts("Starting GPS monitoring...\r\n");
    
    // Wait for initial GPS fix with timeout
    safe_uart_puts("Waiting for initial GPS fix (timeout: 60 seconds)...\r\n");
    if (wait_for_gps_fix(60000)) {  // 60 second timeout for first fix
        safe_uart_puts("GPS fix acquired successfully!\r\n");
    } else {
        safe_uart_puts("GPS fix timeout - continuing without initial fix.\r\n");
    }

    // Initialize NRF24L01
    if (!init_nrf24l01()) {
        safe_uart_puts("Failed to initialize NRF24L01 - check connections\r\n");
        // Continue anyway, but radio won't work
    }


    // Launch Core 1 for audio processing
    safe_uart_puts("Starting Core 1...\r\n");
    multicore_launch_core1(core1_entry);
    
    // Wait for Core 1 to initialize with timeout
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while (!core1_ready) {
        // Blink LED while waiting
        gpio_put(LED_INBUILT_IN, true);
        sleep_ms(100);
        gpio_put(LED_INBUILT_IN, false);
        sleep_ms(100);
        
        // Timeout after 10 seconds
        if (to_ms_since_boot(get_absolute_time()) - start_time > 10000) {
            safe_uart_puts("ERROR: Core 1 initialization timeout!\r\n");
            break;
        }
    }
    
    if (core1_ready) {
        safe_uart_puts("Core 1 initialized successfully\r\n");
    }
    

    
    // Main loop - GPS debug mode
    safe_uart_puts("\r\n=== Entering GPS Debug Mode ===\r\n");
    safe_uart_puts("Monitoring GPS data...\r\n\r\n");
    
    while (true) {
        // Heartbeat LED - now we can use a longer delay since GPS is handled by interrupts
        gpio_put(LED_INBUILT_IN, !gpio_get(LED_INBUILT_IN));  // Toggle LED
        sleep_ms(500);  // 1Hz blink to show main loop is running
    }
    
    return 0;  // Never reached
}

// Initialize power control
bool init_power_control(void) {
    safe_uart_puts("Initializing power control...\n");
    
    // Load power control program into PIO memory
    PIO pio = POWER_CONTROL_PIO;
    uint sm = POWER_CONTROL_SM;
    
    if (!pio_can_add_program(pio, &power_control_program)) {
        safe_uart_puts("Failed to load power control program - no memory\n");
        return false;
    }
    
    uint offset = pio_add_program(pio, &power_control_program);
    
    // Calculate clock divider for desired frequency (1 kHz)
    // The period will be 2^16, so to get 1kHz:
    // CPU_freq / (desired_freq * 2^16) = clk_div
    uint clk_div = clock_get_hz(clk_sys) / (POWER_FREQ * (1u << 16));
    
    // Initialize the PWM program
    power_control_program_init(pio, sm, offset, POWER_CONTROL_PIN, clk_div);
    
    // Set the period (using full 16-bit resolution)
    uint32_t period = (1u << 16) - 1;
    uint32_t level = period / 2;  // 50% duty cycle
    
    power_control_set_period(pio, sm, period);
    power_control_set_level(pio, sm, level);
    
    safe_uart_puts("Power control initialized successfully\n");
    return true;
}

// Update power control settings (0-100 for duty cycle)
void update_power_control(float freq, float duty_cycle) {
    PIO pio = POWER_CONTROL_PIO;
    uint sm = POWER_CONTROL_SM;
    
    // Clamp duty cycle to valid range
    if (duty_cycle < 0.0f) duty_cycle = 0.0f;
    if (duty_cycle > 100.0f) duty_cycle = 100.0f;
    
    // Convert duty cycle percentage to level
    uint32_t period = (1u << 16) - 1;
    uint32_t level = (uint32_t)((duty_cycle / 100.0f) * period);
    
    // Update the PWM
    power_control_set_level(pio, sm, level);
}
