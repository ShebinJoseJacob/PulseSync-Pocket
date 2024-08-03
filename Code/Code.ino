#include <DFRobot_MLX90614.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "thingProperties.h"
#include "icons.h"
#include <Digital_Stethoscope_inferencing.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2s.h"

#define BUTTON_PIN_BITMASK 0x200000000 // 2^33 in hex
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

DFRobot_MLX90614_I2C mlx90614Sensor;
MAX30105 particleSensor;

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read

#define MAX_BRIGHTNESS 255

// Define constants and variables
const int BUTTON_PIN = D1;
bool buttonPressed = false;
int mode = 0; // Start mode from 0


/** Audio buffers, pointers and selectors */
typedef struct {
    signed short *buffers[2];
    unsigned char buf_select;
    unsigned char buf_ready;
    unsigned int buf_count;
    unsigned int n_samples;
} inference_t;

static inference_t inference;
static const uint32_t sample_buffer_size = 2048;
static signed short sampleBuffer[sample_buffer_size];
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);
static bool record_status = true;


void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Hello");

  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);

  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  // Initialize MLX90614 sensor
  while (NO_ERR != mlx90614Sensor.begin()) {
    //Serial.println("Communication with MLX90614 sensor failed, please check connection");
    delay(3000);
  }

  // Initialize MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    //Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  particleSensor.setup(60, 4, 2, 100, 411, 4096); // Example settings, adjust as needed
  
    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: ");
    ei_printf_float((float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf(" ms.\n");
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));*/

    run_classifier_init();
    //ei_printf("\nStarting continious inference in 2 seconds...\n");
    //ei_sleep(2000);

    if (microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE) == false) 
    {
        //ei_printf("ERR: Could not allocate audio buffer (size %d), this could be due to the window length of your model\r\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT);
        return;
    }

    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    //Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
}

void loop() 
{
  //ArduinoCloud.update();
  
  // Button debouncing
  static unsigned long lastDebounceTime = 0;
  static unsigned long debounceDelay = 500; // Adjust debounce delay as needed

  if (digitalRead(BUTTON_PIN) == LOW && (millis() - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = millis();
    while (digitalRead(BUTTON_PIN) == LOW); // Wait for button release
    buttonPressed = true;
  }

  if (buttonPressed) {
    mode = (mode + 1) % 3; // Cycle through modes 0, 1, 2 , 

    switch (mode) {
      case 0:
        Serial.println("Mode 0: MLX90614 Sensor");
        display.display();
        delay(1000);
        display.clearDisplay();
        display.drawBitmap(0, 0, epd_bitmap_temperature, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
        display.display();
        //displayDataFromMlx90614Sensor();
        break;
      case 1:
        Serial.println("Mode 1: MAX30105 Sensor");
        display.display();
        delay(1000);
        display.clearDisplay();
        display.drawBitmap(0, 0, epd_bitmap_spo2, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
        display.display();
        //displayDataFromMax30102Sensor();
        break;
      case 2:
        Serial.println("Mode 2: Steth");
        display.display();
        delay(1000);
        display.clearDisplay();
        display.drawBitmap(0, 0, epd_bitmap_ai, SCREEN_WIDTH, SCREEN_HEIGHT, WHITE);
        display.display();
        //displayDataFromsteth();
        break;
    }

    buttonPressed = false; // Reset button press flag
  }
}

void displayDataFromMlx90614Sensor() {
  float temp = 0;
  for (int i=0; i<20; i++){
    float ambientTemp = mlx90614Sensor.getAmbientTempCelsius();
    float objectTemp = mlx90614Sensor.getObjectTempCelsius();
  
    // Print measured data in Celsius to //Serial monitor
    //Serial.println("MLX90614 Sensor Data:");
    //Serial.print("Ambient temperature: ");
    //Serial.print(ambientTemp);
    //Serial.println(" C");
    //Serial.print("Object temperature: ");
    //Serial.print(objectTemp);
    //Serial.println(" C");
  
    temp += objectTemp;
  }
  oT = 35.8; //temp/20;
}

void displayDataFromMax30102Sensor() 
{
  bufferLength = 100;

  // Read the first 100 samples and determine the signal range
  for (byte i = 0; i < bufferLength; i++) {
    while (!particleSensor.available())
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();

    //Serial.print(F("red="));
    //Serial.print(redBuffer[i], DEC);
    //Serial.print(F(", ir="));
    //Serial.println(irBuffer[i], DEC);
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Continuously take samples from MAX30102. Heart rate and SpO2 are calculated every 1 second
  for (byte i = 25; i < 100; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25] = irBuffer[i];
  }

  for (byte i = 75; i < 100; i++) {
    while (!particleSensor.available())
      particleSensor.check();

    digitalWrite(readLED, !digitalRead(readLED));

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.print(irBuffer[i], DEC);

    Serial.print(F(", HR="));
    Serial.print(heartRate, DEC);

    Serial.print(F(", HRvalid="));
    Serial.print(validHeartRate, DEC);

    Serial.print(F(", SPO2="));
    Serial.print(spo2, DEC);

    Serial.print(F(", SPO2Valid="));
    Serial.println(validSPO2, DEC);

    ArduinoCloud.update();
  }

  bufferLength = 100; // Reset buffer length
}

void displayDataFromsteth()
{  
   int j=0;
    while ( j <10){
      bool m = microphone_inference_record();
      if (!m) {
          ei_printf("ERR: Failed to record audio...\n");
          return;
      }
  
      signal_t signal;
      signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
      signal.get_data = &microphone_audio_signal_get_data;
      ei_impulse_result_t result = {0};
  
      EI_IMPULSE_ERROR r = run_classifier_continuous(&signal, &result, debug_nn);
      if (r != EI_IMPULSE_OK) {
          ei_printf("ERR: Failed to run classifier (%d)\n", r);
          return;
      }
  
      if (++print_results >= (EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)) {
          // print the predictions
          //ei_printf("Predictions ");
          //ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
          //    result.timing.dsp, result.timing.classification, result.timing.anomaly);
          //ei_printf(": \n");
          for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
              //ei_printf("    %s: ", result.classification[ix].label);
              //ei_printf_float(result.classification[ix].value);
              //ei_printf("\n");
              if (result.classification[ix].value > 0.3){
                hS = result.classification[ix].label;
              }
          }
  
      print_results = 0;
    }
    j++;
    }
}

static void audio_inference_callback(uint32_t n_bytes)
{
    for(int i = 0; i < n_bytes>>1; i++) {
        inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];

        if(inference.buf_count >= inference.n_samples) {
            inference.buf_select ^= 1;
            inference.buf_count = 0;
            inference.buf_ready = 1;
        }
    }
}

static void capture_samples(void* arg) {

  const int32_t i2s_bytes_to_read = (uint32_t)arg;
  size_t bytes_read = i2s_bytes_to_read;

  while (record_status) {

    /* read data at once from i2s */
    i2s_read((i2s_port_t)1, (void*)sampleBuffer, i2s_bytes_to_read, &bytes_read, 100);

    if (bytes_read <= 0) {
      ei_printf("Error in I2S read : %d", bytes_read);
    }
    else {
        if (bytes_read < i2s_bytes_to_read) {
        ei_printf("Partial I2S read");
        }

        // scale the data (otherwise the sound is too quiet)
        for (int x = 0; x < i2s_bytes_to_read/2; x++) {
            sampleBuffer[x] = (int16_t)(sampleBuffer[x]) * 8;
        }

        if (record_status) {
            audio_inference_callback(i2s_bytes_to_read);
        }
        else {
            break;
        }
    }
  }
  vTaskDelete(NULL);
}

/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffers[0] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[0] == NULL) {
        return false;
    }

    inference.buffers[1] = (signed short *)malloc(n_samples * sizeof(signed short));

    if (inference.buffers[1] == NULL) {
        ei_free(inference.buffers[0]);
        return false;
    }

    inference.buf_select = 0;
    inference.buf_count = 0;
    inference.n_samples = n_samples;
    inference.buf_ready = 0;

    if (i2s_init(EI_CLASSIFIER_FREQUENCY)) {
        ei_printf("Failed to start I2S!");
    }

    ei_sleep(100);

    record_status = true;

    xTaskCreate(capture_samples, "CaptureSamples", 1024 * 32, (void*)sample_buffer_size, 10, NULL);

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    bool ret = true;

    if (inference.buf_ready == 1) {
        ei_printf(
            "Error sample buffer overrun. Decrease the number of slices per model window "
            "(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW)\n");
        ret = false;
    }

    while (inference.buf_ready == 0) {
        delay(1);
    }

    inference.buf_ready = 0;
    return true;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    i2s_deinit();
    ei_free(inference.buffers[0]);
    ei_free(inference.buffers[1]);
}


static int i2s_init(uint32_t sampling_rate) {
  // Start listening for audio: MONO @ 8/16KHz
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
      .sample_rate = sampling_rate,
      .bits_per_sample = (i2s_bits_per_sample_t)16,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_I2S,
      .intr_alloc_flags = 0,
      .dma_buf_count = 8,
      .dma_buf_len = 512,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = -1,
  };
  i2s_pin_config_t pin_config = {
      .bck_io_num = D3,    // IIS_SCLK
      .ws_io_num = D6,     // IIS_LCLK
      .data_out_num = -1,  // IIS_DSIN
      .data_in_num = D7,   // IIS_DOUT
  };
  esp_err_t ret = 0;

  ret = i2s_driver_install((i2s_port_t)1, &i2s_config, 0, NULL);
  if (ret != ESP_OK) {
    ei_printf("Error in i2s_driver_install");
  }

  ret = i2s_set_pin((i2s_port_t)1, &pin_config);
  if (ret != ESP_OK) {
    ei_printf("Error in i2s_set_pin");
  }

  ret = i2s_zero_dma_buffer((i2s_port_t)1);
  if (ret != ESP_OK) {
    ei_printf("Error in initializing dma buffer with 0");
  }

  return int(ret);
}

static int i2s_deinit(void) {
    i2s_driver_uninstall((i2s_port_t)1); //stop & destroy i2s driver
    return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif


void onECGChange() {
  
}
void onHRChange() {

}


void onSPO2Change() {
}

void onOTChange() {
  
}

void onHSChange(){
  
}
