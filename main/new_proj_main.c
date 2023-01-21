#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "new_proj_main.h"
#include "string.h"
#include "math.h"
#include "soc/dport_reg.h"

#define I2C_MASTER_FREQ_HZ 40000
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_OUTPUT_IO_18 18
#define GPIO_INPUT_IO_18 18
#define GPIO_MEASURE_PIN 19
#define GPIO_OUTPUT_PIN_SEL ((1ULL << GPIO_OUTPUT_IO_18))
#define GPIO_INPUT_PIN_SEL (1ULL<<GPIO_INPUT_IO_18)
#define GPIO_MEASURE_PIN_SEL (1ULL<<GPIO_MEASURE_PIN)
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

portMUX_TYPE gpio_spinlock = portMUX_INITIALIZER_UNLOCKED;
static esp_adc_cal_characteristics_t* adc_chars;
static xQueueHandle gpio_event_queue = NULL;
static xQueueHandle gpio_measure_event_queue = NULL;
uint32_t int_counter = 0;
uint32_t int_counter2 = 0;
typedef struct {
    uint64_t timer_value;
    uint32_t gpio_level;
    uint32_t int_num;
} communication_snapshot;

static void IRAM_ATTR gpio_isr_handler() {
    communication_snapshot s;
    s.timer_value = timer_group_get_counter_value_in_isr(0,0);
    s.gpio_level = gpio_get_level(GPIO_INPUT_IO_18);
    s.int_num  = int_counter;
    int_counter++;
    xQueueSendFromISR(gpio_event_queue, &s, NULL);
}
static void IRAM_ATTR gpio_measure_isr_handler() {
    communication_snapshot s;
    s.timer_value = timer_group_get_counter_value_in_isr(1,1);
    s.gpio_level = gpio_get_level(GPIO_MEASURE_PIN);
    s.int_num  = int_counter2;
    int_counter2++;
    xQueueSendFromISR(gpio_measure_event_queue, &s, NULL);

}
void print_measure_timer() {
    communication_snapshot s;
    uint64_t previous_timer_value = 0;
    for(;;) {
        xQueueReceive(gpio_measure_event_queue, &s, portMAX_DELAY);
        uint32_t time_difference_micros = ((s.timer_value - previous_timer_value)*25)/1000;
        previous_timer_value = s.timer_value;
        printf("-----measure port time_difference_micros: %d, level: %d, message number: %d\n",time_difference_micros,s.gpio_level,s.int_num);
    }
}
void print_timer() {
    printf("starting timer print task\n");
    communication_snapshot s;
    uint64_t previous_timer_value = 0;
    uint8_t temp = 0;
    uint8_t temp_decimal = 0;
    uint8_t humid = 0;
    uint8_t humid_decimal = 0;
    uint8_t checksum = 0;
    uint64_t raw = 0;
    uint8_t counter = 0;
    for(;;){
        counter = 0;
        raw=0;
        bool message_complete = false;
        while(counter < 40) {
            if(xQueueReceive(gpio_event_queue, &s, portMAX_DELAY)){
                uint32_t time_difference_micros = ((s.timer_value - previous_timer_value)*25)/1000;
                printf("time_difference_micros: %d, level: %d, message number: %d\n",time_difference_micros,s.gpio_level,s.int_num);
                previous_timer_value = s.timer_value;
                if(time_difference_micros > 15 && time_difference_micros < 30) {
                    raw = raw << 1;
                    if (counter == 39) {
                        message_complete = true;
                    }
                    counter++;
                }
                else if (time_difference_micros > 60 && time_difference_micros < 80) {
                    raw = raw << 1;
                    raw |= 1;
                    if (counter == 39) {
                        message_complete = true;
                    }
                    counter++;
                }
                if (message_complete) {
                    humid = *(((uint8_t*)(&raw))+4);
                    humid_decimal = *(((uint8_t*)(&raw))+3);
                    temp = *(((uint8_t*)(&raw))+2);
                    temp_decimal = *(((uint8_t*)(&raw))+1);
                    checksum = *(((uint8_t*)(&raw))+0);
                    if (humid + humid_decimal + temp + temp_decimal == checksum) {
                        float ftemp = ((float)temp + (float)temp_decimal / 10) * 9 / 5 + 32;
                        printf("temp: %.0fF, humidity: %d%%\n",ftemp,humid);
                    } else {
                        printf("data invalid, raw: %llx\n",raw);

                    }
                }
            }
            else {
                printf("xQueueReceive failed\n"); 
            }
        }
    }
}
void set_gpio_to_output() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}
void set_gpio_to_input() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
}
void set_gpio_measure() {
    gpio_config_t measure_configuration = {};
    measure_configuration.intr_type = GPIO_INTR_ANYEDGE;
    measure_configuration.mode = GPIO_MODE_INPUT;
    measure_configuration.pin_bit_mask = GPIO_MEASURE_PIN_SEL;
    measure_configuration.pull_up_en = 1;
    measure_configuration.pull_down_en = 0;
    gpio_config(&measure_configuration);

}
void timer_setup() {
    timer_config_t timer_config = {};
    timer_config.divider = 2;
    timer_config.counter_dir = TIMER_COUNT_UP;
    timer_config.counter_en = TIMER_START;
    timer_config.alarm_en = 0;
    timer_init(0,0,&timer_config);
    timer_config.divider = 2;
    timer_config.counter_dir = TIMER_COUNT_UP;
    timer_config.counter_en = TIMER_START;
    timer_config.alarm_en = 0;
    timer_init(1,1,&timer_config);
}
void print_hex(uint32_t hex_value) {
    uint32_t mask = 0x80000000;
    for(int i = 0; i < (sizeof hex_value) * 8; i++) {
        if (hex_value & mask) {
            printf("1 ");
        } else {
            printf("0 "); 
        }
        hex_value = hex_value << 1;
    }
    printf("\n");
}
void adc_register_dump(){
    uint32_t* SENS_SAR_START_FORCE_REG = 0x3FF4882C;
    uint32_t* SENS_SAR_READ_CTRL_REG = 0x3FF48800;



    uint32_t* SENS_SAR_ATTEN1_REG = 0x3FF48834;
    uint32_t* APB_SARADC_CTRL_REG = 0x60002610;
    uint32_t* APB_SARADC_CTRL2_REG = 0x60002614;
    uint32_t* APB_SARADC_FSM_REG = 0x60002618;
    uint32_t* APB_SARADC_SAR1_PATT_TAB1_REG = 0x6000261C;
    uint32_t* APB_SARADC_SAR1_PATT_TAB2_REG = 0x60002620;
    uint32_t* APB_SARADC_SAR1_PATT_TAB3_REG = 0x60002624;
    uint32_t* APB_SARADC_SAR1_PATT_TAB4_REG = 0x60002628;

    uint32_t sar_control = *SENS_SAR_START_FORCE_REG;
    uint32_t sar_sampling = *SENS_SAR_READ_CTRL_REG;
    uint32_t sar_status = *(uint32_t*)SENS_SAR_MEAS_START1_REG;
    uint32_t sar_config = *APB_SARADC_CTRL_REG;
    uint32_t sar_config2 = *APB_SARADC_CTRL2_REG;
    uint32_t sar_cycles = *APB_SARADC_FSM_REG;
    uint32_t pattern_Table_1 = *APB_SARADC_SAR1_PATT_TAB1_REG;
    uint32_t pattern_Table_2 = *APB_SARADC_SAR1_PATT_TAB2_REG;
    uint32_t pattern_Table_3 = *APB_SARADC_SAR1_PATT_TAB3_REG;
    uint32_t pattern_Table_4 = *APB_SARADC_SAR1_PATT_TAB4_REG;

    /*adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten(ADC_CHANNEL_6,ADC_ATTEN_DB_11);
      adc_chars = calloc(1, sizeof(*adc_chars));
      esp_adc_cal_characterize(ADC_UNIT_1,ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adc_chars);
      adc1_get_raw((adc_channel_t)ADC_CHANNEL_6);*/

    printf("SENS_SAR_START_FORCE_REG: ");
    print_hex(*SENS_SAR_START_FORCE_REG);
    printf("SENS_SAR_READ_CTRL_REG: ");
    print_hex(*SENS_SAR_READ_CTRL_REG);
    printf("SENS_SAR_MEAS_START1_REG: ");
    print_hex(*(uint32_t*)SENS_SAR_MEAS_START1_REG);
    printf("SENS_SAR_ATTEN1_REG: ");
    print_hex(SENS_SAR_ATTEN1_REG);
    printf("APB_SARADC_CTRL_REG: ");
    print_hex(*APB_SARADC_CTRL_REG);
    printf("APB_SARADC_CTRL2_REG: ");
    print_hex(*APB_SARADC_CTRL2_REG);
    printf("APB_SARADC_FSM_REG: ");
    print_hex(*APB_SARADC_FSM_REG);
    printf("APB_SARADC_SAR1_PATT_TAB1_REG: ");
    print_hex(*APB_SARADC_SAR1_PATT_TAB1_REG);
    printf("APB_SARADC_SAR1_PATT_TAB2_REG: ");
    print_hex(*APB_SARADC_SAR1_PATT_TAB2_REG);
    printf("APB_SARADC_SAR1_PATT_TAB3_REG: ");
    print_hex(*APB_SARADC_SAR1_PATT_TAB3_REG);
    printf("APB_SARADC_SAR1_PATT_TAB4_REG: ");
    print_hex(*APB_SARADC_SAR1_PATT_TAB4_REG);
    //print_hex(sar_sampling);
}
#define NUMBER_OF_CHANNELS 1
#define ADC_DIGITAL_CONFIG_ALL_CHANNELS_MASK (1 << 3 | 1<<4 | 1<<5 | 1<<6 | 1<<7 | 1 << 0)
#define ADC_DIGITAL_CONFIG_CHANNEL_0_MASK (1 << 0)
void adc_digital_setup() {
    adc_digi_init_config_t adc_conf = {
        .max_store_buf_size = 1000,
        .conv_num_each_intr = 32,
        .adc1_chan_mask = ADC_DIGITAL_CONFIG_CHANNEL_0_MASK,
        .adc2_chan_mask = 0x0,
    };
    adc_digi_pattern_config_t* channel_patterns = malloc(NUMBER_OF_CHANNELS * sizeof *channel_patterns);
    channel_patterns[0].atten = 3;
    channel_patterns[0].bit_width = 12;
    channel_patterns[0].channel = 0;
    channel_patterns[0].unit = 0;
    channel_patterns[1].atten = 3;
    channel_patterns[1].bit_width = 12;
    channel_patterns[1].channel = 3;
    channel_patterns[1].unit = 0;

    channel_patterns[2].atten = 3;
    channel_patterns[2].bit_width = 12;
    channel_patterns[2].channel = 4;
    channel_patterns[2].unit = 0;

    channel_patterns[3].atten = 3;
    channel_patterns[3].bit_width = 12;
    channel_patterns[3].channel = 5;
    channel_patterns[3].unit = 0;

    channel_patterns[4].atten = 3;
    channel_patterns[4].bit_width = 12;
    channel_patterns[4].channel = 6;
    channel_patterns[4].unit = 0;

    channel_patterns[5].atten = 3;
    channel_patterns[5].bit_width = 12;
    channel_patterns[5].channel = 7;
    channel_patterns[5].unit = 0;

    adc_digi_configuration_t adc_controller_conf = {
        .adc_pattern = channel_patterns,
        .conv_limit_en = 1,
        .conv_limit_num = 1000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
        .pattern_num = NUMBER_OF_CHANNELS,
        .sample_freq_hz = 1,
    };
    adc_digi_initialize(&adc_conf);
    adc_digi_controller_configure(&adc_controller_conf);
    adc_digi_start();
    uint8_t* adc_results = malloc(1000 * sizeof *adc_results);
    uint32_t length_max = 1000;
    uint32_t timeout_ms = 10000;
    uint32_t* bytes_read = malloc(1*sizeof*bytes_read);
    adc_digi_read_bytes(adc_results, length_max, bytes_read,timeout_ms);
    printf("-------adc digital read-----------\n");
    printf("bytes read: %u\n",*bytes_read);
    for(int i = 0; i < *bytes_read; i++) {
        printf("%u: %d\n",i,adc_results[i]);
    }
    printf("-------adc digital read-----------\n");


}
#define SENS_SAR1_STOP_BIT (23)
#define SENS_SAR_START_FORCE_REG 0x3FF4882C
#define SENS_SAR_READ_CTRL_REG 0x3FF48800
#define SENS_SAR1_DIG_FORCE (27)
#define SENS_SAR_ATTEN1_REG 0x3FF48834
#define ADC1_CH0_ATTEN_11
void adc_register_set() {
    uint32_t reg = *(uint32_t*)SENS_SAR_START_FORCE_REG;
    reg &= 0x0000000f;
    reg |= 1 << SENS_SAR1_STOP_BIT;
    *(uint32_t*)SENS_SAR_START_FORCE_REG = reg;

    reg = *(uint32_t*)SENS_SAR_READ_CTRL_REG;
    reg &= 0x00070902;
    reg |= 1 << SENS_SAR1_DIG_FORCE;
    *(uint32_t*)SENS_SAR_READ_CTRL_REG = reg;

    reg = *(uint32_t*)SENS_SAR_ATTEN1_REG;
    reg &= 0x0;
}
#define ADC_BASE_ADDRESS  0x3FF48800
#define ADC_RANGE 0
#define PERIPHERAL_CONF_END_ADDRESS 0x3FF7FFFF
#define PERIPHERAL_CONF_START_ADDRESS 0x3FF00000
void peripheral_register_dump() {
    for (uint32_t i = PERIPHERAL_CONF_START_ADDRESS; i < PERIPHERAL_CONF_END_ADDRESS; i+=4) {
        uint32_t reg = *(uint32_t*)i;
        if(reg != 0) {
            printf("%x, ",i);
            print_hex(reg);
        }
    }
}
void photoresistor_setup() {
    UBaseType_t stack_remaining = uxTaskGetStackHighWaterMark(NULL);
    float Vo;
    float Vin = 3.3;

    float R1 = 10000;
    float logR2, R2, T;
    float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
    uint32_t reading;
    uint32_t reading2;
    uint32_t reading3;
    for(;;){
        reading =  adc1_get_raw(ADC1_CHANNEL_0);
        reading2 =  adc1_get_raw(ADC1_CHANNEL_3);
        reading3 =  adc1_get_raw(ADC1_CHANNEL_6);
        Vo = (float)reading2 / 4095 * 3.3;
        R2 = R1 * (Vin / (float)Vo - 1.0);
        logR2 = log(R2);
        T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
        T = T - 273.15;
        T = (T * 9.0)/ 5.0 + 32.0; 

        printf("thermistor temp: %f\n", T);
        printf("ch0: %d, ch3: %d, ch6 %d\n",reading,reading2,reading3);
        vTaskDelay(1000/portTICK_RATE_MS);
        stack_remaining = uxTaskGetStackHighWaterMark(NULL);
        printf("photoresistor_setup stack remaining: %d\n", stack_remaining);
    } 
}
void adc_watcher() {
    volatile uint32_t adc_conversion_register;
    uint32_t adc_conversion_register_previous = 0;
    for(;;){
        adc_conversion_register = *(uint32_t*)SENS_SAR_MEAS_START1_REG;
        if (adc_conversion_register != adc_conversion_register_previous){
            printf("adc conversion register: ");
            print_hex(adc_conversion_register);
            adc_conversion_register_previous = adc_conversion_register;
        }
    }
}
#define GPIO_OUTPUT_SOURCE_SOFTWARE 0x100
#define GPIO_FUNCn_OEN_SEL 10
#define IO_MUX_PIN_BASE 0x3FF49000
#define GPIO_FUNCTION_2 7
#define MCU_SEL 12
#define IO_MUX_x_REG_XOR_MASK 0x00007fff
#define FUN_DRV 10
#define FUN_DRV_MAX 3
#define FUN_DRV_DEFAULT 2
#define FUN_DRV_LOW 2
#define GPIO_FUNCn_OUT_SEL_CFG_REG 0x530
#define GPIO_FUNCn_IN_SEL_CFG_REG 0x130
#define GPIO_FUNCn_IN_SEL_CFG_REG_MASK 0xffffff00
#define GPIO_SIGy_IN_SEL 7
#define IO_MUX_x_REG 0x10
#define FUN_IE 9
void set_gpio_output(uint32_t pin_num){
    //dr_reg_GPIO_BA
    uint32_t* address = (uint32_t*)(DR_REG_GPIO_BASE + pin_num * sizeof(uint32_t) + GPIO_FUNCn_OUT_SEL_CFG_REG); 
    printf("address: %x\n",(uint32_t)address);
    uint32_t reg = *address;
    reg |= GPIO_OUTPUT_SOURCE_SOFTWARE | 1 << GPIO_FUNCn_OEN_SEL;
    *address= reg;
    address = (uint32_t*)GPIO_ENABLE_REG;
    reg = *address;
    reg |= 1 << pin_num;
    *address = reg;
    address = (uint32_t*)(IO_MUX_PIN_BASE + 0x10 * pin_num);
    reg = *address; 
    reg &= IO_MUX_x_REG_XOR_MASK ^ 0xffffffff; 
    reg |= FUN_DRV_DEFAULT << FUN_DRV;
    *address= reg;
}
#define NOP() asm volatile ("nop")
uint32_t out_reg = 0;
void set_gpio_input(void* pn){
    uint32_t pin_num = *(uint32_t*)pn; 
    printf("starting read_led\n");
    uint32_t reg;
    uint32_t* address;
    /*reg = (uint32_t*)(DR_REG_GPIO_BASE + pin_num * GPIO_FUNCn_OUT_SEL_CFG_REG); 
      reg &= 0xfffffc00;
      reg |= 1 << 10;
     *(uint32_t*)GPIO_FUNC19_OUT_SEL_CFG_REG = reg;*/
    address = (uint32_t*)GPIO_ENABLE_REG;
    reg = *address;
    reg &= (uint32_t)(1 << pin_num) ^ 0xffffffff;
    *address = reg;
    address =(uint32_t*)(DR_REG_GPIO_BASE + pin_num * sizeof(uint32_t) + GPIO_FUNCn_IN_SEL_CFG_REG); 
    printf("in config calculated address: %u\n",(unsigned int)address);
    reg = *address;  
    reg &= GPIO_FUNCn_IN_SEL_CFG_REG_MASK;
    reg |= 1 << GPIO_SIGy_IN_SEL | pin_num;
    *address = reg;
    address =(uint32_t*)( IO_MUX_PIN_BASE + IO_MUX_x_REG + sizeof(uint32_t) * pin_num);
    reg = *address;
    reg &= IO_MUX_x_REG_XOR_MASK ^ 0xffffffff; 
    reg |= GPIO_FUNCTION_2 << MCU_SEL | 1 << FUN_IE | FUN_DRV_DEFAULT << FUN_DRV; 
    *address = reg;
    uint32_t mask = 1 << 19;
    bool previous_led = false;
    bool led_on = false;
    printf("mask: %x\n",mask);
    /*while(1){
        reg = *(uint32_t*)GPIO_IN_REG; 
        //printf("reg: %x\n",reg);
        //printf("input reg: %x\n",reg);
        NOP();
        if (mask & reg) {
            led_on = true;
        } else {
            led_on = false;
        }
        if (led_on ^ previous_led) {
            printf("led: %d\n",led_on);
            previous_led = led_on;
        }*/
}
uint32_t int_call = 0;
void IRAM_ATTR gpio_intr_service(void* arg) {
    uint32_t reg =*(uint32_t*)GPIO_STATUS_REG; 
    uint32_t mask = 1 << 19;
    int_call++;
    if (reg & mask) {
    } 
    *(uint32_t*)GPIO_STATUS_W1TC_REG = 1 << 19; 
}
void app_main(void) {
    printf("starting app main\n");
    //xTaskCreate(blink_led,"blink_led", 5000, NULL, 10, NULL);
    uint32_t pin_num;
    //xTaskCreate(set_gpio_input,"read_led",5000, &pin_num,10,NULL);
    volatile uint32_t counter = 0;
    /*portENTER_CRITICAL(&gpio_spinlock);
      uint32_t reg; 
      reg = *(uint32_t*)DPORT_PRO_GPIO_INTERRUPT_MAP_REG;
      reg &= 0xffffffe0;
     *(uint32_t*)DPORT_PRO_GPIO_INTERRUPT_MAP_REG = reg;
     reg =*(uint32_t*)GPIO_PIN19_REG;
     reg &= (uint32_t)(0x1f << 13 | 0xf << 7 | 1 << 2) ^ 0xffffffff;
     reg |= 1 << 16 | 4 << 7;
     *(uint32_t*)GPIO_PIN19_REG = reg;
     portEXIT_CRITICAL(&gpio_spinlock);
     gpio_isr_handler_add(19,gpio_intr_service,NULL);*/

    set_gpio_output(19);
    *(uint32_t*)GPIO_OUT_W1TS_REG |= 1 << 19;
    set_gpio_output(18); 
    pin_num = 18;
    set_gpio_input(&pin_num);
    while(1){
        vTaskDelay(1000/portTICK_RATE_MS);
        printf("interrupt calls: %d\n",int_call);
        counter++;
        volatile uint32_t reg = 0; 
        uint32_t* address;
            bool led_on = false;
            bool previous_led = false;
            uint32_t mask = 1 << 19;
        while(1) {
            address = (uint32_t*)GPIO_ENABLE_REG;
            reg = *address;
            reg |= 1 << pin_num;
            *address = reg;
            address = (uint32_t*)GPIO_OUT_W1TS_REG;
            *address |= 1 << pin_num;
            vTaskDelay(1000/portTICK_RATE_MS);
            address = (uint32_t*)GPIO_OUT_W1TC_REG;
            *address |= 1 << pin_num;
            address = (uint32_t*)GPIO_ENABLE_REG;
            reg = *address;
            reg &= (uint32_t)(1 << pin_num) ^ 0xffffffff;
            *address = reg;
            address = (uint32_t*)(IO_MUX_PIN_BASE + 0x10 * pin_num);
            reg = *address;
             reg &= (uint32_t)(3<<FUN_DRV) ^ 0x0007ffff;

             *address= reg;
            vTaskDelay(1000/portTICK_RATE_MS);
            for(int i = 0; i < 100000; i++){
                reg = *(uint32_t*)GPIO_IN_REG; 
                //printf("reg: %x\n",reg);
                //printf("input reg: %x\n",reg);
                NOP();
                if (mask & reg) {
                    led_on = true;
                } else {
                    led_on = false;
                }
                if (led_on ^ previous_led) {
                    printf("led: %d\n",led_on);
                    previous_led = led_on;
                }
            }
        }
    }

    //peripheral_register_dump();
    //printf("here\n");
    //adc_digital_setup();
    //peripheral_register_dump();
    //adc_register_set();
    //adc_register_dump();
    //adc_register_set();
    //gpio_event_queue = xQueueCreate(100, sizeof(communication_snapshot));
    //gpio_measure_event_queue = xQueueCreate(100, sizeof(communication_snapshot));
    /*if (xTaskCreate(photoresistor_setup, "photoresistor_setup", 5000,NULL,10,NULL) == pdFAIL) {
      printf("failed to stat photoresistor_setup\n");
      }
      if (xTaskCreate(adc_watcher, "adc_watcher", 10000, NULL, 10, NULL) == pdFAIL){
      printf("failed to start adc watcher\n");
      }*/
    //xTaskCreate(print_measure_timer,"print_measure_timer",10000, NULL,10,NULL);

    /*timer_setup();
      set_gpio_measure();
      gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT); 
      gpio_isr_handler_add(19,gpio_measure_isr_handler,NULL);
      gpio_isr_handler_add(18,gpio_isr_handler,NULL);
      if (xTaskCreate(print_timer, "print timer value", 5000, NULL, 10, NULL) == pdFAIL) {
      printf("failed to create task\n");
      }

      volatile uint32_t counter = 0;
      while(1){
      vTaskDelay(2000/portTICK_RATE_MS);
      gpio_set_level(18,0);
      vTaskDelay(20/portTICK_RATE_MS);
    //gpio_set_level(18,1);
    set_gpio_to_input();
    vTaskDelay(40/portTICK_RATE_MS);
    set_gpio_to_output();
    gpio_set_level(18,1);
    }*/
}
