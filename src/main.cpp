#include <Arduino.h>
#include <driver/adc.h>

#define ADC_PIN_CHANNEL ADC1_CHANNEL_0  // GPIO 1 (Receiver)
#define TX_PIN          2               // GPIO 2 (Transmitter)
#define NUM_SAMPLES     1024

const uint8_t SYNC_HEADER[4] = {0xAA, 0xBB, 0xCC, 0xDD};
uint16_t adc_buffer[NUM_SAMPLES];
const int pwmChannel = 0;

void setup() {
    Serial.begin(115200);
    Serial.setRxBufferSize(4096);
    Serial.setTxBufferSize(4096);

    // 40 kHz hardware PWM
    ledcSetup(pwmChannel, 40000, 8);
    ledcAttachPin(TX_PIN, pwmChannel);
    ledcWrite(pwmChannel, 0);

    // 12-bit ADC on GPIO 1
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_PIN_CHANNEL, ADC_ATTEN_DB_12);
}

void loop() {
    // 1. Transmit high-energy burst (40 cycles of 40 kHz = 1000 us)
    ledcWrite(pwmChannel, 128);
    delayMicroseconds(1000);
    ledcWrite(pwmChannel, 0);

    // 2. Continuous high-speed sampling
    for (int i = 0; i < NUM_SAMPLES; i++) {
        adc_buffer[i] = (uint16_t)adc1_get_raw(ADC_PIN_CHANNEL);
        delayMicroseconds(8);
    }

    // 3. Stream binary packet
    Serial.write(SYNC_HEADER, 4);
    Serial.write((const uint8_t*)adc_buffer, sizeof(adc_buffer));
    Serial.flush();

    delay(60);
}