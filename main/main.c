#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"

#define PIN_LCD_BL              2
#define PIN_TP_INT              5
#define PIN_TP_SDA              6
#define PIN_TP_SCL              7
#define PIN_LCD_DC              8
#define PIN_LCD_CS              9
#define PIN_LCD_CLK             10
#define PIN_LCD_MOSI            11
#define PIN_LCD_MISO            12
#define PIN_TP_RST              13
#define PIN_LCD_RST             14

#define LCD_PIXEL_CLOCK_HZ      (80 * 1000 * 1000)
#define LCD_HOST                SPI2_HOST
#define LCD_H_RES               240
#define LCD_V_RES               240
#define LCD_CMD_BITS            8
#define LCD_PARAM_BITS          8

#define LVGL_TICK_PERIOD_MS     2
#define LVGL_TASK_MAX_DELAY_MS  500
#define LVGL_TASK_MIN_DELAY_MS  1
#define LVGL_TASK_STACK_SIZE    (4 * 1024)
#define LVGL_TASK_PRIORITY      2

static const char *TAG = "MAIN";

static SemaphoreHandle_t lvgl_mutex = NULL;

static bool esp_lcd_panel_io_spi_config__on_color_trans_done(esp_lcd_panel_io_handle_t esp_lcd_panel_io_handle, esp_lcd_panel_io_event_data_t *esp_lcd_panel_io_event_data, void *user_ctx)
{
    lv_disp_drv_t *lv_disp_drv = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(lv_disp_drv);
    return false;
}

static void lv_disp_drv__flush_cb(lv_disp_drv_t *lv_disp_drv, const lv_area_t *lv_area, lv_color_t *lv_color)
{
    esp_lcd_panel_handle_t esp_lcd_panel_handle = (esp_lcd_panel_handle_t) lv_disp_drv->user_data;
    int x_start = lv_area->x1;
    int y_start = lv_area->y1;
    int x_end = lv_area->x2 + 1;
    int y_end = lv_area->y2 + 1;
    esp_lcd_panel_draw_bitmap(esp_lcd_panel_handle, x_start, y_start, x_end, y_end, lv_color);
}

static void lvgl_tick_timer_args__callback(void *arg)
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL Task");
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
            task_delay_ms = lv_timer_handler();
            xSemaphoreGiveRecursive(lvgl_mutex);
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

extern void lvgl_ui(lv_disp_t *lv_disp);

void app_main(void)
{
    static lv_disp_draw_buf_t lv_disp_draw_buf;
    static lv_disp_drv_t lv_disp_drv;

    ESP_LOGI(TAG, "Initializing Backlight");
    gpio_config_t gpio_config_bl = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << PIN_LCD_BL
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_config_bl));
    ESP_LOGI(TAG, "Initialized Backlight");

    ESP_LOGI(TAG, "Initializing SPI Bus");
    spi_bus_config_t spi_bus_config = {
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = PIN_LCD_MISO,
        .sclk_io_num = PIN_LCD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &spi_bus_config, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "Initialized SPI Bus");

    ESP_LOGI(TAG, "Initializing Panel IO");
    esp_lcd_panel_io_spi_config_t esp_lcd_panel_io_spi_config = {
        .cs_gpio_num = PIN_LCD_CS,
        .dc_gpio_num = PIN_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .on_color_trans_done = esp_lcd_panel_io_spi_config__on_color_trans_done,
        .user_ctx = &lv_disp_drv,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
    };
    esp_lcd_panel_io_handle_t esp_lcd_panel_io_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &esp_lcd_panel_io_spi_config, &esp_lcd_panel_io_handle));
    ESP_LOGI(TAG, "Initialized Panel IO");

    ESP_LOGI(TAG, "Initializing GC9A01 Panel Driver");
    esp_lcd_panel_dev_config_t esp_lcd_panel_dev_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    esp_lcd_panel_handle_t esp_lcd_panel_handle = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(esp_lcd_panel_io_handle, &esp_lcd_panel_dev_config, &esp_lcd_panel_handle));
    ESP_LOGI(TAG, "Initialized GC9A01 Panel Driver");

    ESP_LOGI(TAG, "Initializing LCD");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(esp_lcd_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(esp_lcd_panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(esp_lcd_panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(esp_lcd_panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(esp_lcd_panel_handle, true));
    ESP_LOGI(TAG, "Initialized LCD");

    ESP_LOGI(TAG, "Enabling Backlight");
    gpio_set_level(PIN_LCD_BL, 1);
    ESP_LOGI(TAG, "Enabled Backlight");

    ESP_LOGI(TAG, "Initializing LVGL");
    lv_init();
    ESP_LOGI(TAG, "Initialized LVGL");

    ESP_LOGI(TAG, "Initializing LVGL Draw Buffer");
    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    lv_disp_draw_buf_init(&lv_disp_draw_buf, buf1, buf2, LCD_H_RES * 20);
    ESP_LOGI(TAG, "Initialized LVGL Draw Buffer");

    ESP_LOGI(TAG, "Registering Display Driver to LVGL");
    lv_disp_drv_init(&lv_disp_drv);
    lv_disp_drv.hor_res = LCD_H_RES;
    lv_disp_drv.ver_res = LCD_V_RES;
    lv_disp_drv.draw_buf = &lv_disp_draw_buf;
    lv_disp_drv.flush_cb = lv_disp_drv__flush_cb;
    lv_disp_drv.user_data = esp_lcd_panel_handle;
    lv_disp_t *lv_disp = lv_disp_drv_register(&lv_disp_drv);
    ESP_LOGI(TAG, "Registered Display Driver to LVGL");

    ESP_LOGI(TAG, "Initializing LVGL Tick Timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick_timer_args__callback,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));
    ESP_LOGI(TAG, "Initialized LVGL Tick Timer");

    lvgl_mutex = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mutex);
    ESP_LOGI(TAG, "Creating LVGL Task");
    xTaskCreate(lvgl_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL);
    ESP_LOGI(TAG, "Created LVGL Task");

    ESP_LOGI(TAG, "Display LVGL UI");
    if (xSemaphoreTakeRecursive(lvgl_mutex, portMAX_DELAY)) {
        lvgl_ui(lv_disp);
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
}
