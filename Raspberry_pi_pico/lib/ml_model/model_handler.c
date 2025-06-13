#include "model_handler.h"
#include <stdlib.h>

// Global variables
static float confidence_score = 0.0f;

// Initialize the model 
int model_init(void) {
    // For now, just initialize the confidence score
    confidence_score = 0.0f;
    return 0; // Success
}

// Process audio data for gunshot detection
bool model_process_audio(const int32_t* audio_buffer, size_t buffer_size) {
    // Temporary implementation using amplitude threshold
    int32_t max_amplitude = 0;
    for (size_t i = 0; i < buffer_size; i++) {
        int32_t sample = audio_buffer[i] >> 8;  // Convert to 24-bit
        int32_t amplitude = abs(sample);
        if (amplitude > max_amplitude) {
            max_amplitude = amplitude;
        }
    }
    
    // Store max amplitude as confidence (normalized to 0-1)
    confidence_score = max_amplitude / 1000000.0f;
    
    // Return true if confidence exceeds threshold
    return confidence_score > 0.8f;
}

// Get the current confidence score
float model_get_confidence(void) {
    return confidence_score;
}
