#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "esp_log.h"

#define LCD_20X04 1
#define I2C_ACK_CHECK_EN 1
#define I2C_ADDRESS_LCD 0x27
#define I2C_SCL_LCD 22
#define I2C_SDA_LCD 21
/* Comandos de la LCD */
#define CLEAR_DISPLAY 0x01
#define RETURN_HOME_UNSHIFT 0x02
#define CURSOR_RIGHT_NO_SHIFT 0x04
#define CURSOR_RIGHT_SHIFT 0x05
#define CURSOR_RIGHT_NO_SHIFT_LEFT 0x06
#define CURSOR_RIGHT_SHIFT_LEFT 0x07
#define DISPLAY_OFF 0x08
#define DISPLAY_ON_CURSOR_OFF 0x0C
#define DISPLAY_ON_CURSOR_ON_STEADY 0x0E
#define DISPLAY_ON_CURSOR_ON_BLINK 0x0F
#define SHIFT_CURSOR_LEFT 0x10
#define SHIFT_CURSOR_RIGHT 0x14
#define SHIFT_DISPLAY_LEFT 0x18
#define SHIFT_DISPLAY_RIGHT 0x1C
#define SET_4BIT_MODE 0x28
#define RETURN_HOME 0x80
/* PCF8574 */
#define PCF8574_RS 0
#define PCF8574_RW 1
#define PCF8574_EN 2
#define PCF8574_BL 3
#define LCD_RS_CMD 0
#define LCD_RS_DATA 1

/*TOUCH PINS*/
#define TOUCH_THRESH_PERCENT 80
#define TOUCH_THRESH_NO_USE 0
#define TOUCH_FILTER_MODE_EN 1
#define TOUCHPAD_FILTER_TOUCH_PERIOD 10
#define NUM_TPINS 6
#define TOUCH_START 4

#define MAX_DUTY 4096
#define INC 20

/*EVENT GROUPS*/
#define DATA_READY  (BIT0)
#define DATA_SENT   (BIT1)
#define PWM_MOTOR_DATA_READY (BIT2)
#define PWM_SERVO_DATA_READY (BIT3)

typedef struct{
  uint8_t i2c_address;
  uint8_t i2c_port;
  uint8_t screen_size;
  uint8_t screen_backlight;
} lcd_i2c_device_t;

static uint8_t s_pad_activated[NUM_TPINS];
static uint32_t s_pad_init_val[NUM_TPINS];
static uint64_t lastPress=0;

char bloque [1000][8];
char anguloServo[16];
char velocidadMotor[16];

EventGroupHandle_t adcData;
EventGroupHandle_t lcdData;
QueueHandle_t adc;
QueueHandle_t gpio_queue;

void touch_init(void);
static void IRAM_ATTR touch_rtc_intr(void* args); 
void set_touch_thresholds(void);

void init_pwm(void);
void init_gpio(void);
static void IRAM_ATTR gpio_isr_handler(void* args);

void i2c_init(void);
void lcd_init(lcd_i2c_device_t *lcd);
void lcd_i2c_write_byte(lcd_i2c_device_t *lcd, uint8_t data);
void lcd_i2c_write_command(lcd_i2c_device_t *lcd, uint8_t register_select, uint8_t cmd);
void lcd_set_cursor(lcd_i2c_device_t *lcd, uint8_t column, uint8_t row);
void lcd_i2c_write_custom_char(lcd_i2c_device_t *lcd, uint8_t char_address, const uint8_t *pixels);
void lcd_i2c_print_msg(lcd_i2c_device_t *lcd, char *msg);

void touchTask(void* args);
void adcTask(void *args);
void uartAdcTask(void *args);
void lcdTask(void* args);


void app_main(void){
  adcData=xEventGroupCreate();
  lcdData=xEventGroupCreate();
  adc=xQueueCreate(1000,16*sizeof(uint8_t));
  gpio_queue=xQueueCreate(10,sizeof(uint8_t));
  init_gpio();
  i2c_init();
  lcd_i2c_device_t my_lcd = {
      .i2c_port = I2C_NUM_1,
      .i2c_address = I2C_ADDRESS_LCD,
      .screen_size = LCD_20X04,
      .screen_backlight = 1,
  };
  vTaskDelay(20 / portTICK_PERIOD_MS);
  lcd_init(&my_lcd);
  xTaskCreatePinnedToCore(touchTask,"TOUCH",4096,NULL,10,NULL,1);
  xTaskCreatePinnedToCore(adcTask,"ADC",4096,NULL,5,NULL,1);
  xTaskCreatePinnedToCore(lcdTask,"LCD",4096,&my_lcd,5,NULL,0);
  xTaskCreatePinnedToCore(uartAdcTask,"UART",4096*8,NULL,4,NULL,0);
  while(1)
    vTaskDelay(10/portTICK_PERIOD_MS);
}

void adcTask(void *args){
  float temp;
  static int adc_raw;

  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config1 = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));
  static uint16_t idx=0;
  while(1){
    while(idx<1000){
      adc_oneshot_read(adc1_handle,ADC_CHANNEL_6,&adc_raw);
      temp=(adc_raw*100.0)/4095;
      sprintf(bloque[idx++],"%.2f\n",temp);
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    idx=0;
    xQueueSend(adc,bloque,20/portTICK_PERIOD_MS);
    xEventGroupWaitBits(adcData,DATA_SENT,false,true,portMAX_DELAY);
  }
}

void uartAdcTask(void *args){
  static uint64_t lastSent=0;
  uint8_t btn;
  uart_config_t conf={
    .baud_rate=115200,
    .stop_bits=UART_STOP_BITS_1,
    .data_bits=UART_DATA_8_BITS,
    .parity=UART_PARITY_DISABLE,
    .source_clk=UART_SCLK_DEFAULT,
    .flow_ctrl=UART_HW_FLOWCTRL_DISABLE,
  };
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 8192, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_17,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &conf));
  while(1){
    if( xQueueReceive(gpio_queue,&btn,20/portTICK_PERIOD_MS) || ((esp_timer_get_time()-lastSent)>=30000000)){
      for (uint16_t idx = 0; idx < 1000; idx++){
          uart_write_bytes(UART_NUM_1, bloque[idx], strlen(bloque[idx]));
      }
      //ESP_LOGW("UART","DATA SENT");
      lastSent=esp_timer_get_time();
      xEventGroupSetBits(adcData,DATA_SENT);
    }
  }
}

void lcdTask(void* args){
  lcd_i2c_device_t* lcd =(lcd_i2c_device_t*)args;
  char* msg="Velocidad:";
  char* msg2="Angulo Servo:";
  while(1){
    xEventGroupWaitBits(adcData,DATA_SENT,true,true,portMAX_DELAY);
    lcd_set_cursor(lcd,0,0);
    lcd_i2c_print_msg(lcd,msg);
    lcd_i2c_print_msg(lcd,velocidadMotor);
    lcd_set_cursor(lcd,0,1);
    lcd_i2c_print_msg(lcd,msg2);
    lcd_i2c_print_msg(lcd,anguloServo);
    vTaskDelay(250/portTICK_PERIOD_MS);
  }
}

void touchTask(void* args){
  static uint16_t motorDutyCycle=1024;
  static uint16_t servoDutyCycle=1024;
  static char anguloDCStr[16];
  static char velDCStr[16];
  init_pwm();
  ESP_ERROR_CHECK(touch_pad_init());
  touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
  touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
  touch_init();
  touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
  set_touch_thresholds();
  touch_pad_isr_register((intr_handler_t)touch_rtc_intr, NULL);  
  touch_pad_intr_enable();
  while (1){
    if(s_pad_activated[4]==1){ //Stop Servo
      servoDutyCycle=1024;
      //ESP_LOGI("TOUCH", "T4 activated!");
      ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,servoDutyCycle));
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1));
      vTaskDelay(200 / portTICK_PERIOD_MS);
      s_pad_activated[4] = 0;
    }
    else if(s_pad_activated[5]==1){// Decrement Duty Cycle
      servoDutyCycle=(servoDutyCycle>1034)?(servoDutyCycle-INC):(1024);
      //ESP_LOGI("TOUCH", "T5 activated!");
      vTaskDelay(200 / portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,servoDutyCycle));
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1));
      s_pad_activated[5] = 0;
    }
    else if(s_pad_activated[6]==1){// Increment Duty Cycle
      servoDutyCycle=(servoDutyCycle<MAX_DUTY)?(servoDutyCycle+INC):(MAX_DUTY);
      //ESP_LOGI("TOUCH", "T6 activated!");
      vTaskDelay(200 / portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1,servoDutyCycle));
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_1));
      s_pad_activated[6] = 0;
    }
    else if(s_pad_activated[7]==1){// Stop Motor
      motorDutyCycle=1024;
      //ESP_LOGI("TOUCH", "T7 activated!");
      //ESP_LOGW("TOUCH VAR","%d",motorDutyCycle);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,motorDutyCycle));
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0));
      s_pad_activated[7] = 0;
    }
    else if(s_pad_activated[8]==1){// Decrement Motor Duty Cycle
      motorDutyCycle=(motorDutyCycle>1034)?(motorDutyCycle-INC):(1024);
      //ESP_LOGI("TOUCH", "T8 activated!");
      //ESP_LOGW("TOUCH VAR","%d",motorDutyCycle);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,motorDutyCycle));
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0));
      s_pad_activated[8] = 0;
    }
    else if(s_pad_activated[9]==1){// Increment Motor Duty Cycle
      motorDutyCycle=(motorDutyCycle<MAX_DUTY)?(motorDutyCycle+INC):(MAX_DUTY);
      //ESP_LOGI("TOUCH", "T9 activated!");
      //ESP_LOGW("TOUCH VAR","%d",motorDutyCycle);
      vTaskDelay(200 / portTICK_PERIOD_MS);
      ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,motorDutyCycle));
      ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0));
      s_pad_activated[9] = 0;
    }
    sprintf(anguloServo,"%d",(int)((180*(servoDutyCycle-1024))/3072));
    sprintf(velocidadMotor,"%d",(int)(1250+(3750/4096)*motorDutyCycle));
    sprintf(anguloDCStr,"S%d\n",servoDutyCycle);
    sprintf(velDCStr,"M%d\n",motorDutyCycle);
    uart_write_bytes(UART_NUM_1,(const char*)anguloDCStr,strlen((const char*)anguloDCStr));
    uart_write_bytes(UART_NUM_1,(const char*)velDCStr,strlen((const char*)velDCStr));
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

static void gpio_isr_handler(void* args){
  uint8_t pinNumber = (uint8_t)(uintptr_t)args;
  if((esp_timer_get_time()-lastPress)>=50000){
		lastPress=esp_timer_get_time();
	  xQueueSendFromISR(gpio_queue, &pinNumber, NULL);
	}

}

void init_gpio(void){
  gpio_install_isr_service(0);
  gpio_set_direction(GPIO_NUM_35, GPIO_MODE_INPUT);
  gpio_set_intr_type(GPIO_NUM_35, GPIO_INTR_POSEDGE);
  gpio_set_pull_mode(GPIO_NUM_35, GPIO_PULLDOWN_DISABLE);
  gpio_isr_handler_add(GPIO_NUM_35, gpio_isr_handler, (void*)GPIO_NUM_35);
}

void init_pwm(void){
  ledc_timer_config_t config={
    .speed_mode=LEDC_LOW_SPEED_MODE,
    .timer_num=LEDC_TIMER_0,
    .duty_resolution=LEDC_TIMER_12_BIT,
    .freq_hz=400,
    .clk_cfg=LEDC_AUTO_CLK,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&config));
  ledc_channel_config_t motor_channel={
    .speed_mode=LEDC_LOW_SPEED_MODE,
    .channel=LEDC_CHANNEL_0,
    .timer_sel=LEDC_TIMER_0,
    .intr_type=LEDC_INTR_DISABLE,
    .gpio_num=GPIO_NUM_26,//Blanco
    .duty=1024,
    .hpoint=0,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&motor_channel));
  ledc_channel_config_t servo_channel={
    .speed_mode=LEDC_LOW_SPEED_MODE,
    .channel=LEDC_CHANNEL_1,
    .timer_sel=LEDC_TIMER_0,
    .intr_type=LEDC_INTR_DISABLE,
    .gpio_num=GPIO_NUM_25,//Amarillo
    .duty=1024,
    .hpoint=0,
  };
  ESP_ERROR_CHECK(ledc_channel_config(&servo_channel));
}

void touch_init(void){
  for (uint8_t idx =TOUCH_START; idx<10; idx++)
    touch_pad_config(idx, TOUCH_THRESH_NO_USE);
}

void touch_rtc_intr(void* args){
  uint32_t pad_intr = touch_pad_get_status();
  touch_pad_clear_status();
  for (uint8_t idx =TOUCH_START; idx <10; idx++){
    if ((pad_intr >> idx) & 0x01)
      s_pad_activated[idx] = 1;
  }
} 

void set_touch_thresholds(void){
  uint16_t touch_value;
  for (uint8_t idx = TOUCH_START; idx<10; idx++){
      touch_pad_read_filtered(idx, &touch_value);
      s_pad_init_val[idx] = touch_value;
      ESP_ERROR_CHECK(touch_pad_set_thresh(idx, touch_value * 2 / 3));    
  }
}

void i2c_init(void){
  i2c_config_t i2c_config = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA_LCD,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = I2C_SCL_LCD,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = 400000,
  };
  esp_err_t error = i2c_param_config(I2C_NUM_1, &i2c_config);
  if (error != ESP_OK)
    while (1);
  i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
}

void lcd_init(lcd_i2c_device_t *lcd){
  lcd_i2c_write_command(lcd, LCD_RS_CMD, RETURN_HOME_UNSHIFT);
  lcd_i2c_write_command(lcd, LCD_RS_CMD, SET_4BIT_MODE);
  lcd_i2c_write_command(lcd, LCD_RS_CMD, CLEAR_DISPLAY);
  lcd_i2c_write_command(lcd, LCD_RS_CMD, DISPLAY_ON_CURSOR_OFF);
  lcd_i2c_write_command(lcd, LCD_RS_CMD, CURSOR_RIGHT_NO_SHIFT_LEFT);
  vTaskDelay(20 / portTICK_PERIOD_MS);
}

void lcd_i2c_write_byte(lcd_i2c_device_t *lcd, uint8_t data){
  i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
  i2c_master_start(cmd_handle);
  i2c_master_write_byte(cmd_handle, (lcd->i2c_address << 1) | I2C_MASTER_WRITE, I2C_ACK_CHECK_EN);
  i2c_master_write_byte(cmd_handle, data, 1);
  i2c_master_stop(cmd_handle);
  i2c_master_cmd_begin(lcd->i2c_port, cmd_handle, 100 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd_handle);
}

void lcd_i2c_write_command(lcd_i2c_device_t *lcd, uint8_t register_select, uint8_t cmd){
  uint8_t config = (register_select) ? (1 << PCF8574_RS) : 0;
  config |= (lcd->screen_backlight) ? (1 << PCF8574_BL) : 0;
  config |= (config & 0x0F) | (0xF0 & cmd);
  config |= (1 << PCF8574_EN);
  lcd_i2c_write_byte(lcd, config);
  ets_delay_us(10);
  config &= ~(1 << PCF8574_EN);
  lcd_i2c_write_byte(lcd, config);
  ets_delay_us(50);
  config = (config & 0x0F) | (cmd << 4);
  config |= (1 << PCF8574_EN);
  lcd_i2c_write_byte(lcd, config);
  ets_delay_us(10);
  config &= ~(1 << PCF8574_EN);
  lcd_i2c_write_byte(lcd, config);
  ets_delay_us(50);
  if (cmd == CLEAR_DISPLAY)
    vTaskDelay(20 / portTICK_PERIOD_MS);
}

void lcd_set_cursor(lcd_i2c_device_t *lcd, uint8_t column, uint8_t row){
  switch (row){
  case 0:
    lcd_i2c_write_command(lcd, LCD_RS_CMD, 0x80 + column);
    break;
  case 1:
    lcd_i2c_write_command(lcd, LCD_RS_CMD, 0xC0 + column);
    break;
  case 2:
    lcd_i2c_write_command(lcd, LCD_RS_CMD, 0x94 + column);
    break;
  case 3:
    lcd_i2c_write_command(lcd, LCD_RS_CMD, 0xD4 + column);
    break;
  default:
    break;
  }
}

void lcd_i2c_write_custom_char(lcd_i2c_device_t *lcd, uint8_t address, const uint8_t *pixels){
  lcd_i2c_write_command(lcd, LCD_RS_CMD, 0x40 | (address << 3));
  for (uint8_t i = 0; i < 8; i++)
    lcd_i2c_write_command(lcd, LCD_RS_DATA, pixels[i]);
  lcd_i2c_write_command(lcd, LCD_RS_CMD, RETURN_HOME);
}

void lcd_i2c_print_msg(lcd_i2c_device_t *lcd, char *msg){
  uint8_t i = 0;
  while (msg[i])
    lcd_i2c_write_command(lcd, LCD_RS_DATA, msg[i++]);
}