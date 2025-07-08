#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lvgl_port.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define LCD_H_RES       240
#define LCD_W_RES       320

#define LCD_HOST        SPI3_HOST
#define PIN_NUM_MOSI    GPIO_NUM_23
#define PIN_NUM_CLK     GPIO_NUM_18
#define PIN_NUM_CS      GPIO_NUM_5
#define PIN_NUM_DC      GPIO_NUM_2
#define PIN_NUM_RST     GPIO_NUM_4

enum {SLIDER_R = 0, SLIDER_G, SLIDER_B};

typedef struct {
    uint8_t slider_type;
    lv_obj_t* label;
    lv_obj_t* slider;
}rgb_mixer_t;

static const char* TAG = "LVGL_ST7789";
static esp_lcd_panel_handle_t panel_handle = NULL;
static esp_lcd_panel_io_handle_t io_handle = NULL;
static lv_obj_t* rect;
static uint8_t r_value = 0, g_value = 0, b_value = 0;
static rgb_mixer_t r, g, b;
static SemaphoreHandle_t rgb_mutex;
static QueueHandle_t button_queue;

void init_st7789(){
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_CLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadhd_io_num = -1,
        .quadwp_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_W_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = PIN_NUM_CS,
        .dc_gpio_num = PIN_NUM_DC,
        .spi_mode = 0,
        .pclk_hz = 40 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    // esp_lcd_panel_swap_xy(panel_handle, true);
    // esp_lcd_panel_mirror(panel_handle, true, false);
    // esp_lcd_panel_disp_on_off(panel_handle, true);
}

void init_lvgl(){
    lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    lvgl_port_display_cfg_t discfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .control_handle = panel_handle,
        .hres = LCD_W_RES,
        .vres = LCD_H_RES,
        .buffer_size = LCD_W_RES * 40 * sizeof(uint16_t),
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation.swap_xy = 1, 
        .rotation.mirror_x = 1,
    };
    lv_disp_t *disp = lvgl_port_add_disp(&discfg);
    if(disp == NULL){
        ESP_LOGI(TAG, "Error in creating lv_disp");
        return;
    }
    esp_lcd_panel_swap_xy(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, true, false);
    esp_lcd_panel_disp_on_off(panel_handle, true);
}

void rgb_mixer_create_ui(){

    lvgl_port_lock(portMAX_DELAY);
    r.slider_type = SLIDER_R;
    g.slider_type = SLIDER_G;
    b.slider_type = SLIDER_B;

    r.slider = lv_slider_create(lv_scr_act());
    g.slider = lv_slider_create(lv_scr_act());
    b.slider = lv_slider_create(lv_scr_act());

    lv_slider_set_range(r.slider, 0, 255);
    lv_slider_set_range(g.slider, 0, 255);
    lv_slider_set_range(b.slider, 0, 255);

    lv_obj_align(r.slider, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_align_to(g.slider, r.slider, LV_ALIGN_TOP_MID, 0, 40);
    lv_obj_align_to(b.slider, g.slider, LV_ALIGN_TOP_MID, 0, 40);

    rect = lv_obj_create(lv_scr_act());
    lv_obj_set_size(rect, 300, 80);
    lv_obj_align_to(rect, b.slider, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_set_style_border_color(rect, lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_border_width(rect, 3, LV_PART_MAIN);
    lv_obj_set_style_bg_color(rect, lv_color_black(), LV_PART_MAIN);

    //RGB
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_make(0x00, 0x00, 0x00), LV_PART_MAIN);
    lv_obj_set_style_bg_color(r.slider, lv_color_make(0x00, 0xff, 0xff),LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(r.slider, lv_color_make(0x00, 0xff, 0xff), LV_PART_KNOB);
    lv_obj_set_style_bg_color(g.slider, lv_color_make(0xff, 0x00, 0xff), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(g.slider, lv_color_make(0xff, 0x00, 0xff), LV_PART_KNOB);
    lv_obj_set_style_bg_color(b.slider, lv_color_make(0xff, 0xff, 0x00), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(b.slider, lv_color_make(0xff, 0xff, 0x00), LV_PART_KNOB);

    lv_obj_t* heading = lv_label_create(lv_scr_act());
    lv_label_set_text(heading, "RGB Mixer");
    lv_obj_set_style_text_color(heading, lv_color_make(0xff, 0xff, 0xff), LV_PART_MAIN);
    lv_obj_align(heading, LV_ALIGN_TOP_MID, 0, 10);

    r.label = lv_label_create(lv_scr_act());
    lv_label_set_text(r.label, "0");
    lv_obj_set_style_text_color(r.label, lv_color_make(0xff, 0xff, 0xff), LV_PART_MAIN);
    lv_obj_align_to(r.label, r.slider, LV_ALIGN_TOP_MID, 0, 0);

    g.label = lv_label_create(lv_scr_act());
    lv_label_set_text(g.label, "0");
    lv_obj_set_style_text_color(g.label, lv_color_make(0xff, 0xff, 0xff), LV_PART_MAIN);
    lv_obj_align_to(g.label, g.slider, LV_ALIGN_TOP_MID, 0, 0);

    b.label = lv_label_create(lv_scr_act());
    lv_label_set_text(b.label, "0");
    lv_obj_set_style_text_color(b.label, lv_color_make(0xff, 0xff, 0xff), LV_PART_MAIN);
    lv_obj_align_to(b.label, b.slider, LV_ALIGN_TOP_MID, 0, 0);
    lvgl_port_unlock();
}

void IRAM_ATTR rgb_button_isr_handler(void *arg){
    static uint16_t last_time = 0;
    rgb_mixer_t* rgb_mixer = (rgb_mixer_t*)arg;
    if(esp_timer_get_time() - last_time > 200){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(button_queue, &rgb_mixer->slider_type, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        last_time = esp_timer_get_time();
    }
}

void button_task(void *arg){
    uint8_t slider_type;
    while(1){
        if(xQueueReceive(button_queue, &slider_type, portMAX_DELAY)){
            xSemaphoreTake(rgb_mutex, portMAX_DELAY);
            if(slider_type == SLIDER_R) {
                if(r_value == 255) r_value = 0;
                r_value += 5;
                printf("Red\n");
            }
            else if(slider_type == SLIDER_G) {
                if(g_value == 255) g_value = 0;
                g_value += 5;
                printf("Green\n");
            }
            else if(slider_type == SLIDER_B) {
                if(b_value == 255) b_value = 0;
                b_value += 5;
                printf("Blue\n");
            }
            xSemaphoreGive(rgb_mutex);
        }
    }
}

void set_rgb_value(){
    while(1){
        if(lvgl_port_lock(portMAX_DELAY)){
            xSemaphoreTake(rgb_mutex, portMAX_DELAY);
            lv_slider_set_value(r.slider, r_value, LV_ANIM_ON);
            lv_slider_set_value(g.slider, g_value, LV_ANIM_ON);
            lv_slider_set_value(b.slider, b_value, LV_ANIM_ON);
            xSemaphoreGive(rgb_mutex);
            lvgl_port_unlock();
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void set_rect_value(){
    while(1){
        if(lvgl_port_lock(portMAX_DELAY)){
            xSemaphoreTake(rgb_mutex, portMAX_DELAY);
            lv_obj_set_style_bg_color(rect, lv_color_make(~r_value, ~g_value, ~b_value), LV_PART_MAIN);
            xSemaphoreGive(rgb_mutex);
            lvgl_port_unlock();
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void set_label_value(){
    char rv[4], gv[4], bv[4];
    while(1){
        if(lvgl_port_lock(portMAX_DELAY)){
            xSemaphoreTake(rgb_mutex, portMAX_DELAY);
            sprintf(rv, "%d", r_value);
            sprintf(gv, "%d", g_value);
            sprintf(bv, "%d", b_value);
            lv_label_set_text(r.label, rv);
            lv_label_set_text(g.label, gv);
            lv_label_set_text(b.label, bv);
            xSemaphoreGive(rgb_mutex);
            lvgl_port_unlock();
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}

void app_main(){
    rgb_mutex = xSemaphoreCreateMutex();
    button_queue = xQueueCreate(10, sizeof(uint8_t));

    init_st7789();
    init_lvgl();
    rgb_mixer_create_ui();

    gpio_config_t io_config = {};
    io_config.intr_type = GPIO_INTR_NEGEDGE;
    io_config.mode = GPIO_MODE_INPUT;
    io_config.pin_bit_mask = (1ULL << GPIO_NUM_32) | (1ULL << GPIO_NUM_33) | (1ULL << GPIO_NUM_25);
    io_config.pull_up_en = 1;
    io_config.pull_down_en = 0;
    
    gpio_config(&io_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_25, rgb_button_isr_handler, &r);
    gpio_isr_handler_add(GPIO_NUM_32, rgb_button_isr_handler, &g);
    gpio_isr_handler_add(GPIO_NUM_33, rgb_button_isr_handler, &b);

    xTaskCreate(button_task, "Button Task", 2048, NULL, 5, NULL);
    xTaskCreate(set_rgb_value, "RGB Value", 2048, NULL, 5, NULL);
    xTaskCreate(set_rect_value, "Rect Value", 2048, NULL, 5, NULL);
    xTaskCreate(set_label_value, "Label Value", 2048, NULL, 5, NULL);
}