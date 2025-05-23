#include "lvgl.h"
#include "lvglDrivers.h"
#include "lv_conf.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "ax12.h"
#include "usart.h"

#define DISP_HOR_RES 480
#define DISP_VER_RES 272

/*********************
 * LVGL TASK
 *********************/
static void lvglTask(void *pvParameters)
{
    while (1)
    {
        uint32_t time_till_next = lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(time_till_next));
    }
}

/*********************
 * LVGL FLUSH CALLBACK
 *********************/
static void my_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    uint32_t *buf = (uint32_t *)px_map;
    int32_t x, y;
    for (y = area->y1; y <= area->y2; y++)
    {
        for (x = area->x1; x <= area->x2; x++)
        {
            BSP_LCD_DrawPixel(x, y, *buf);
            buf++;
        }
    }

    lv_display_flush_ready(display);
}

/*********************
 * TOUCH INPUT CALLBACK
 *********************/
static void my_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    TS_StateTypeDef TS_State;
    BSP_TS_GetState(&TS_State);

    if (TS_State.touchDetected != 0)
    {
        data->point.x = TS_State.touchX[0];
        data->point.y = TS_State.touchY[0];
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/*********************
 * LOG PRINT CALLBACK
 *********************/
void my_log_cb(lv_log_level_t level, const char *buf)
{
    printf("[LVGL] %s\n", buf);
}

/*********************
 * EVENT: AX-12 BUTTON
 *********************/
static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED)
    {
        ax12_ping(1);
        HAL_Delay(50);

        uint16_t pos = ax12_read_position(1);
        HAL_Delay(50);

        if (pos != 0xFFFF)
        {
            uint16_t newPos = pos + 50;
            if (newPos > 1023) newPos = 1023;
            ax12_move_to_position(1, newPos);
        }
    }
}

/*********************
 * UI SETUP
 *********************/
void mySetup()
{
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Move AX-12");
    lv_obj_center(label);

    lv_obj_t * btn2 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_height(btn2, LV_SIZE_CONTENT);

    label = lv_label_create(btn2);
    lv_label_set_text(label, "Toggle");
    lv_obj_center(label);
}

/*********************
 * OPTIONAL BACKGROUND TASK
 *********************/
void myTask(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        // Exemple : lire et afficher position AX-12 ici
        // uint16_t pos = ax12_read_position(1);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
    }
}

/*********************
 * SYSTEM SETUP
 *********************/
void setup()
{
    printf("Start\n");

    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
    BSP_TS_Init(480, 272);

    UART1_Init();  // Init UART pour AX-12

    lv_init();
    lv_log_register_print_cb(my_log_cb);

    // Affichage
    lv_display_t *display = lv_display_create(DISP_HOR_RES, DISP_VER_RES);
    static uint32_t buf[DISP_HOR_RES * DISP_VER_RES / 10];  // 10 % buffer
    lv_display_set_flush_cb(display, my_flush_cb);
    lv_display_set_buffers(display, buf, NULL, sizeof(buf), LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Tactile
    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, my_read_cb);

    // Tick handler FreeRTOS
    lv_tick_set_cb(xTaskGetTickCount);

    // Interface utilisateur
    mySetup();

    // Tâches
    xTaskCreate(lvglTask, NULL, 16384, NULL, osPriorityNormal, NULL);
    xTaskCreate(myTask, NULL, 16384, NULL, osPriorityNormal, NULL);

    vTaskStartScheduler();

    // Si jamais on sort du scheduler (RAM manquante ?)
    printf("Insufficient RAM!\n");
    while (1);
}

void loop() {
    // Nécessaire pour linker avec Arduino core
}
