#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "esp_adc/adc_oneshot.h"
#include <driver/i2c.h>
#include <esp_log.h>
#include "bme280.h"
#include "led_strip/led_strip.h"
#include <math.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <esp_sleep.h>
#include <driver/touch_pad.h>

// GPIO assignment
#define LED_STRIP_BLINK_GPIO  0
// Numbers of the LED in the strip
#define LED_STRIP_LED_NUMBERS 16
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

#define GPIO_MOTOR_PIN 23
#define GPIO_MOTOR_PIN_SEL (1ULL << GPIO_MOTOR_PIN)
#define GPIO_BUTTON_PIN 22
#define GPIO_BUTTON_PIN_SEL (1ULL << GPIO_BUTTON_PIN)

#define SDA_PIN GPIO_NUM_2
#define SCL_PIN GPIO_NUM_15

#define TAG_BME280 "BME280"
#define TAG_LED_STRIP "LED_STRIP"
#define TAG_TERRUX "Terrux"

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

#define SAMPLE_CNT 32
#define CURRENT_SENSOR_CHANNEL ADC_CHANNEL_6
#define MOSITURE_SENSOR_CHANNEL ADC_CHANNEL_7
#define WATER_SENSOR_CHANNEL ADC_CHANNEL_4
#define LIGHT_SENSOR_CHANNEL ADC_CHANNEL_5

#define TOUCH_THRESH_NO_USE 0

static SemaphoreHandle_t adc1_mutex_handle;
static SemaphoreHandle_t sensorData_mutex_handle;

typedef struct Led_Strip_Data_t
{
    int red;
    int green;
    int blue;
    int count;
} Led_Strip_Command_t;

typedef struct Sensor_Data_t{
  double temperature;
  double pressure;
  double humidity;
  int light;
  int soilMoisture;
  int waterLevel;
  int motorCurrent;
} Sensor_Data_t;

typedef struct Sensor_Transform_Factor_t{
  float temperature;
  float humidity;
  float light;
  float soilMoisture;
  float waterLevel;
} Sensor_Transform_Factor_t;

Sensor_Transform_Factor_t TransformBy = {
  0.4,
  0.16,
  1,
  1,
  1,
};

int8_t currentDisplayIndex = 0;
adc_oneshot_unit_handle_t adc1_handle;

led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
#else
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
#endif
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG_LED_STRIP, "Created LED strip object with RMT backend");
    return led_strip;
}

static void calibrate_touch_pad(touch_pad_t pad)
{
    int avg = 0;
    const size_t calibration_count = 128;
    for (int i = 0; i < calibration_count; ++i) {
        uint16_t val;
        touch_pad_read(pad, &val);
        avg += val;
    }
    avg /= calibration_count;
    const int min_reading = 300;
    if (avg < min_reading) {
        printf("Touch pad #%d average reading is too low: %d (expecting at least %d). "
               "Not using for deep sleep wakeup.\n", pad, avg, min_reading);
        touch_pad_config(pad, 0);
    } else {
        int threshold = avg - 100;
        printf("Touch pad #%d average: %d, wakeup threshold set to %d.\n", pad, avg, threshold);
        touch_pad_config(pad, threshold);
    }
}

static void init_hw(void)
{

  //-------------ADC1 Init---------------//
  adc_oneshot_unit_init_cfg_t init_config1 = {
       .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

  //-------------ADC1 Config---------------//
  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_11,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, MOSITURE_SENSOR_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, CURRENT_SENSOR_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, WATER_SENSOR_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, LIGHT_SENSOR_CHANNEL, &config));

  gpio_config_t io_conf;

  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = GPIO_MOTOR_PIN_SEL;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);

  // Initialize touch pad peripheral.
  // The default fsm mode is software trigger mode.
  ESP_ERROR_CHECK(touch_pad_init());
  // If use touch pad wake up, should set touch sensor FSM mode at 'TOUCH_FSM_MODE_TIMER'.
  touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
  // Set reference voltage for charging/discharging
  // In this case, the high reference valtage will be 2.4V - 1V = 1.4V
  // The low reference voltage will be 0.5
  // The larger the range, the larger the pulse count value.
  touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
  //init RTC IO and mode for touch pad.
  touch_pad_config(TOUCH_PAD_NUM0, TOUCH_THRESH_NO_USE);
  calibrate_touch_pad(TOUCH_PAD_NUM0);

  printf("Enabling touch pad wakeup\n");
  ESP_ERROR_CHECK(esp_sleep_enable_touchpad_wakeup());
  ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON));

  i2c_config_t i2c_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = SDA_PIN,
      .scl_io_num = SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 1000000};
  i2c_param_config(I2C_NUM_0, &i2c_config);
  i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

  ESP_LOGI(TAG_TERRUX, "Hardware initialized");
}

s8 BME280_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  s32 iError = BME280_INIT_VALUE;

  esp_err_t espRc;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_write(cmd, reg_data, cnt, true);
  i2c_master_stop(cmd);

  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
  if (espRc == ESP_OK)
  {
    iError = SUCCESS;
  }
  else
  {
    iError = ERROR;
  }
  i2c_cmd_link_delete(cmd);

  return (s8)iError;
}

s8 BME280_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
  s32 iError = BME280_INIT_VALUE;
  esp_err_t espRc;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

  if (cnt > 1)
  {
    i2c_master_read(cmd, reg_data, cnt - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, reg_data + cnt - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
  if (espRc == ESP_OK)
  {
    iError = SUCCESS;
  }
  else
  {
    iError = ERROR;
  }

  i2c_cmd_link_delete(cmd);

  return (s8)iError;
}

void BME280_delay_msek(u32 msek)
{
  vTaskDelay(msek / portTICK_PERIOD_MS);
}

void task_bme280_forced_mode(Sensor_Data_t *dev)
{
  struct bme280_t bme280 = {
      .bus_write = BME280_I2C_bus_write,
      .bus_read = BME280_I2C_bus_read,
      .dev_addr = BME280_I2C_ADDRESS2,
      .delay_msec = BME280_delay_msek};

  s32 com_rslt = -1;
  s32 v_uncomp_pressure_s32;
  s32 v_uncomp_temperature_s32;
  s32 v_uncomp_humidity_s32;
  while (com_rslt != SUCCESS)
  {
    com_rslt = bme280_init(&bme280);

    com_rslt += bme280_set_oversamp_pressure(BME280_OVERSAMP_1X);
    com_rslt += bme280_set_oversamp_temperature(BME280_OVERSAMP_1X);
    com_rslt += bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);

    com_rslt += bme280_set_filter(BME280_FILTER_COEFF_OFF);
    if (com_rslt == SUCCESS)
    {
        com_rslt = bme280_get_forced_uncomp_pressure_temperature_humidity(
        &v_uncomp_pressure_s32, &v_uncomp_temperature_s32, &v_uncomp_humidity_s32);

        if (com_rslt == SUCCESS)
        {
          while(1){
            if(xSemaphoreTake(sensorData_mutex_handle, 0) == pdPASS){

              dev->temperature = bme280_compensate_temperature_double(v_uncomp_temperature_s32);
              dev->humidity = bme280_compensate_humidity_double(v_uncomp_humidity_s32);
              dev->pressure = bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100;

              xSemaphoreGive(sensorData_mutex_handle);
              vTaskDelete(NULL);
            }
          }

          /*ESP_LOGI(TAG_BME280, "%.2f degC / %.3f hPa / %.3f %%",
                   bme280_compensate_temperature_double(v_uncomp_temperature_s32),
                   bme280_compensate_pressure_double(v_uncomp_pressure_s32) / 100, // Pa -> hPa
                   bme280_compensate_humidity_double(v_uncomp_humidity_s32));*/
        }
        else
        {
          ESP_LOGE(TAG_BME280, "measure error. code: %d", com_rslt);
        }
    }
    else
    {
      ESP_LOGE(TAG_BME280, "init or setting error. code: %d", com_rslt);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}

void vGetSoilMoisture(Sensor_Data_t *dev)
{ 
  int adc_raw;
  int adc_val = 0;

  if(xSemaphoreTake(adc1_mutex_handle, 0) == pdPASS){
    for (int i = 0; i < SAMPLE_CNT; i++)
    {
      ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, MOSITURE_SENSOR_CHANNEL, &adc_raw));
      adc_val += adc_raw;
    }
    xSemaphoreGive(adc1_mutex_handle);

    adc_val /= SAMPLE_CNT;
      
    while(1){
      if(xSemaphoreTake(sensorData_mutex_handle, 0) == pdPASS){

        dev->soilMoisture = adc_val;

        xSemaphoreGive(sensorData_mutex_handle);
        vTaskDelete(NULL);
      }
    }
    //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, MOSITURE_SENSOR_CHANNEL, adc_val);
  }

  vTaskDelete(NULL);
}

void vGetWaterLevel(Sensor_Data_t *dev)
{ 
  int adc_raw;
  int adc_val = 0;

  if(xSemaphoreTake(adc1_mutex_handle, 0) == pdPASS){
    for (int i = 0; i < SAMPLE_CNT; i++)
    {
      ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, WATER_SENSOR_CHANNEL, &adc_raw));
      adc_val += adc_raw;
    }
    xSemaphoreGive(adc1_mutex_handle);

    adc_val /= SAMPLE_CNT;

    while(1){
      if(xSemaphoreTake(sensorData_mutex_handle, 0) == pdPASS){

        dev->waterLevel = (int)(adc_val * 0.094);

        xSemaphoreGive(sensorData_mutex_handle);
        vTaskDelete(NULL);
      }
    }

    //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, WATER_SENSOR_CHANNEL, (int)(adc_val * 0.094));
  }

  vTaskDelete(NULL);
}

void vGetMotorCurrent(Sensor_Data_t *dev)
{
  int adc_raw;
  int adc_val = 0;
  const char *TAG = "Current Sensor";

  while (1)
  {
    if(xSemaphoreTake(adc1_mutex_handle, 0) == pdPASS){
      for (int i = 0; i < SAMPLE_CNT; i++)
      {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, CURRENT_SENSOR_CHANNEL, &adc_raw));
        adc_val += adc_raw;
      }
      xSemaphoreGive(adc1_mutex_handle);

      adc_val /= SAMPLE_CNT;

      //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %.2f", ADC_UNIT_1 + 1, CURRENT_SENSOR_CHANNEL, 3.05 * (float)adc_val - 2077.7);
      ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, CURRENT_SENSOR_CHANNEL, adc_val);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  vTaskDelete(NULL);
}

void vGetLight(Sensor_Data_t *dev)
{
  int adc_raw;
  int adc_val = 0;

  if(xSemaphoreTake(adc1_mutex_handle, 0) == pdPASS){
    for (int i = 0; i < SAMPLE_CNT; i++)
    {
      ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, LIGHT_SENSOR_CHANNEL, &adc_raw));
      adc_val += adc_raw;
    }
    xSemaphoreGive(adc1_mutex_handle);

    adc_val /= SAMPLE_CNT;

    while(1){
      if(xSemaphoreTake(sensorData_mutex_handle, 0) == pdPASS){

        dev->light = (int)(adc_val / 4096.0);

        xSemaphoreGive(sensorData_mutex_handle);
        vTaskDelete(NULL);
      }
    }

    //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %.2f", ADC_UNIT_1 + 1, LIGHT_SENSOR_CHANNEL, adc_val / 4096.0);
  }


  vTaskDelete(NULL);
}

void vRunMotor(void *parameter)
{
  int runTime;
  runTime = (int)parameter;
  gpio_set_level(GPIO_MOTOR_PIN, 1);
  vTaskDelay(runTime / portTICK_PERIOD_MS);
  gpio_set_level(GPIO_MOTOR_PIN, 0);

  vTaskDelete(NULL);
}

void vBlinkLEDStrip(void *xStruct){
  Led_Strip_Command_t * command = (Led_Strip_Command_t *) xStruct;

  led_strip_handle_t led_strip = configure_led();

  for (int i = 0; i < command->count + 1; i++) {
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, command->red, command->green, command->blue));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    vTaskDelay(pdMS_TO_TICKS(70));
  }

  vTaskDelete(NULL);
    
}

void vClearLEDStrip(void *ignore){

  led_strip_handle_t led_strip = configure_led();

  led_strip_clear(led_strip);

  vTaskDelete(NULL);

}

void app_main()
{

  init_hw();

  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );

  // Open
  printf("\n");
  printf("Opening Non-Volatile Storage (NVS) handle... ");
  nvs_handle_t my_handle;
  err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
    printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
    printf("Done\n");

    // Read
    printf("Reading restart counter from NVS ... ");
    err = nvs_get_i8(my_handle, "dispIndex", &currentDisplayIndex);
    switch (err) {
      case ESP_OK:
        printf("Done\n");
        printf("Restart counter = %" PRId8 "\n", currentDisplayIndex);
        break;
      case ESP_ERR_NVS_NOT_FOUND:
        printf("The value is not initialized yet!\n");
        break;
      default:
        printf("Error (%s) reading!\n", esp_err_to_name(err));
    }
  }

  /*Led_Strip_Command_t test = {
    255,
    0,
    255,
    10
  };*/

  Sensor_Data_t dev;

  adc1_mutex_handle = xSemaphoreCreateMutex();
  sensorData_mutex_handle = xSemaphoreCreateMutex();

  xTaskCreate(&task_bme280_forced_mode, "bme280_forced_mode", 2048, (void *) &dev, 1, NULL);
  //xTaskCreate(&vGetMotorCurrent, "Run get motor current", 2048, (void *) &dev, 1, NULL);
  xTaskCreate(&vGetWaterLevel, "Run get water level", 2048, (void *) &dev, 1, NULL);
  xTaskCreate(&vGetSoilMoisture, "Run get moisture", 2048, (void *) &dev, 1, NULL);
  xTaskCreate(&vGetLight, "Run get light", 2048, (void *) &dev, 1, NULL);

  //xTaskCreate(&vBlinkLEDStrip, "Run Blink Led Strip", 4096, (void *) &test, 1, NULL);

  vTaskDelay(pdMS_TO_TICKS(1000));

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TOUCHPAD)
    {
      //xTaskCreate(&vBlinkLEDStrip, "Run Blink Led Strip", 4096, (void *) &test, 1, NULL);
      //xTaskCreate(&vRunMotor, "Run Motor", 4096, (void *) 1000, 1, NULL);

      switch (currentDisplayIndex)
      {
      case 0:
        Led_Strip_Command_t temperature = {
          255,
          104,
          0,
          (int)(dev.temperature * TransformBy.temperature)
        };
        ESP_LOGI(TAG_TERRUX, "Temperature: %d", (int)(dev.temperature * TransformBy.temperature));
        xTaskCreate(&vBlinkLEDStrip, "Run Blink Led Strip", 4096, (void *) &temperature, 1, NULL);
        break;
      case 1:
        Led_Strip_Command_t humidity = {
          0,
          255,
          232,
          (int)(dev.humidity * TransformBy.humidity)
        };
        ESP_LOGI(TAG_TERRUX, "Humidity: %d", (int)(dev.humidity * TransformBy.humidity));
        xTaskCreate(&vBlinkLEDStrip, "Run Blink Led Strip", 4096, (void *) &humidity, 1, NULL);
        break;
      case 2:
        ESP_LOGI(TAG_TERRUX, "Water Lvl: %d", dev.waterLevel);
        break;
      case 3:
        ESP_LOGI(TAG_TERRUX, "Light: %d", dev.light);
        break;
      case 4:
        ESP_LOGI(TAG_TERRUX, "Soil: %d", dev.soilMoisture);
        break;
      default:
        break;
      }
      
      // Write
      printf("Updating restart counter in NVS ... ");
      currentDisplayIndex = (currentDisplayIndex + 1) % 5;
      err = nvs_set_i8(my_handle, "dispIndex", currentDisplayIndex);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
      printf(esp_err_to_name(err));

      // Commit written value.
      // After setting any values, nvs_commit() must be called to ensure changes are written
      // to flash storage. Implementations may write to storage at other times,
      // but this is not guaranteed.
      printf("Committing updates in NVS ... ");
      err = nvs_commit(my_handle);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

      // Close
      nvs_close(my_handle);

      vTaskDelay(pdMS_TO_TICKS(5000));

      xTaskCreate(&vClearLEDStrip, "Clear Led Strip", 4096, (void *) NULL, 1, NULL);

      vTaskDelay(pdMS_TO_TICKS(1000));

      ESP_LOGI(TAG_TERRUX, "Going to deep sleep");
      esp_deep_sleep_start();
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  
  ESP_LOGI(TAG_TERRUX, "Going to deep sleep");
  esp_deep_sleep_start();
}