
#include "math.h"
#include "soc/rtc.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "./src/BIANCHI.h"
#define GPIO_WAKEUP_PIN_BITMASK (1ULL << 9)
#define us_s 1000000
TaskHandle_t taskHandleUART = NULL, taskHandleI2C = NULL, taskHandleMenu = NULL;
BIANCHI bianchi;
volatile int mode_menu = 0;
volatile bool flag_enter_menu = false, flag_exit_menu = false;
volatile bool need_batt = true, need_reset = false, need_rTsP = false;
volatile bool flag_go_sleep = false, flag_gps_nothing = true;
volatile bool mode_sunshine = false, need_dis = false, flag_info = false;
bool task_made = false;
bool gps_update = false, sub_update_done = true;
void to_menu();
void TaskUART(void *pvParameters);
void Taski2c(void *pvParameters);
long int push_time = 0;
long int wake_up_time = 1000000 * 10, sleep_level = 1000 * 3, time_menu_dejitters = 100; // in millie
long int t_loop = 0, t_sub_loop = 0, t_end = 0, t_menu_last_push = -100;
uint8_t loop_count = 0, loop_count_sunshine = 0, sunshine_int = 4;
volatile IO_REG_TYPE *pin1_register;
volatile IO_REG_TYPE *pin2_register;
volatile IO_REG_TYPE *pinpush_register;
IO_REG_TYPE pin1_bitmask;
IO_REG_TYPE pin2_bitmask;
IO_REG_TYPE pinpush_bitmask;

uint8_t state;
struct tm time_no_gps;
static ICACHE_RAM_ATTR void isr5(void) { update(); }
static ICACHE_RAM_ATTR void isr22(void) { update(); }
void setup()
{

    // pinMode(22,OUTPUT);
    // digitalWrite(22,1);
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    // Serial.begin(115200);
    if ((wakeup_reason != ESP_SLEEP_WAKEUP_TIMER) && (wakeup_reason != ESP_SLEEP_WAKEUP_EXT1))
    {
        struct tm gpsTime;
        gpsTime.tm_year = 2024;
        gpsTime.tm_mon = 7;   // Month, where 0 = jan
        gpsTime.tm_mday = 1;  // Day of the month
        gpsTime.tm_hour = 12; // Hours since midnight - [0,23]
        gpsTime.tm_min = 0;   // Minutes after the hour - [0,59]
        gpsTime.tm_sec = 0;   // Seconds after the minute - [0,59]
        time_t t = mktime(&gpsTime);
        struct timeval now = {.tv_sec = t};
        settimeofday(&now, NULL);
        bianchi.read_poweron_time();
    }
    // esp_sleep_enable_timer_wakeup(10 * us_s);

    pin1_register = PIN_TO_BASEREG(22);
    pin1_bitmask = PIN_TO_BITMASK(22);
    pin2_register = PIN_TO_BASEREG(5);
    pin2_bitmask = PIN_TO_BITMASK(5);
    pinpush_register = PIN_TO_BASEREG(9);
    pinpush_bitmask = PIN_TO_BITMASK(9);
    delay(10);
    uint8_t s = 0;
    if (DIRECT_PIN_READ(pin1_register, pin1_bitmask))
        s |= 1;
    if (DIRECT_PIN_READ(pin2_register, pin2_bitmask))
        s |= 2;
    state = s;
    while (!bianchi.inst())
    {
    }
    if (bianchi.flag_lowpower_sleep)
    {
        // Serial.printf("flag:%d\n", bianchi.flag_i2c_reading);
        return;
    }
    func_pisition();
    pinMode(5, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);
    attachInterrupt(5, isr5, CHANGE);
    attachInterrupt(22, isr22, CHANGE);
    // attachInterrupt(9, to_menu, CHANGE);
    // setCpuFrequencyMhz(48);
    // delay(200);
    bianchi.read_GPS();
    xTaskCreate(
        Taski2c, "Task I2C",
        2048,
        NULL,
        2,
        &taskHandleI2C);
    xTaskCreate(
        TaskUART, "Task UART",
        4096,
        NULL,
        1,
        &taskHandleUART);
    xTaskCreate(
        Taskmenu, "Task Menu",
        4096,
        NULL,
        2,
        &taskHandleMenu);
    task_made = true;
    // delay(500);
}

void loop()
{
    bool had_dis = need_dis;
    // struct tm timeinfo;
    // getLocalTime(&timeinfo);
    // Serial.printf("%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    // Serial.printf("\t%04d %02d %02d\n", timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday);
    // Serial.printf("flag:%d\n", bianchi.flag_i2c_reading);
    if (flag_go_sleep || bianchi.flag_stop_sleep || bianchi.flag_nogps_sleep || (!bianchi._chag && bianchi.flag_lowpower_sleep))
    {
        // Serial.print("in loop ");
        // Serial.printf("flag:%d\n", bianchi.flag_i2c_reading);
        shut_all();
        // Serial.print(" after shut");

        if (bianchi.flag_stop_sleep)
        {
            esp_sleep_enable_ext1_wakeup((1ULL << 14) | (1ULL << 9), ESP_EXT1_WAKEUP_ANY_LOW);
        }
        else
        {
            esp_sleep_enable_ext1_wakeup(1ULL << 9, ESP_EXT1_WAKEUP_ANY_LOW);
        }
        if (bianchi.flag_lowpower_sleep)
        {
            bianchi.display_map(suzuha_lowpower);
        }
        else
        {
            bianchi.display_map(suzuha2);
        }
        bianchi.shut_power();
        esp_deep_sleep_start();
    }

    while (!bianchi.flag_gps_update)
    {
        // int a = 2;
        delay(10);
    }
    bianchi.flag_gps_update = false;
    need_rTsP = true;
    long int sub_time = millis();
    bianchi.display_dis(need_dis);
    bool invert;
    invert = !mode_sunshine || loop_count_sunshine;
    loop_count_sunshine = loop_count_sunshine > sunshine_int ? 0 : loop_count_sunshine + 1;
    bianchi.display_all(invert, mode_menu);

    for (int sub_dis = 0; sub_dis < 1; sub_dis++)
    {
        while ((millis() - sub_time) < 500)
        {
        }
        sub_time = millis();
        bianchi.display_dis(need_dis);
        bianchi.display_all(1, mode_menu);
    }
    need_batt = true;
    if (had_dis)
    {
        need_dis = false;
    }
    // delay(2);
}

void Taskmenu(void *pvParameters)
{
    bool last_state = true;
    while (1)
    {
        bool state = digitalRead(9);
        if (last_state != state)
        {
            delay(50);
            if (state == false) // push
            {
                push_time = millis();

                if (mode_menu == 2) // menu info reset
                {
                    if (bianchi.position_menu > 15)
                    {
                        bianchi.first_IMU_read = 0;
                    }
                    if (bianchi.position_menu > 5 && bianchi.position_menu <= 15)
                    {
                        bianchi.info = true;
                    }
                    if (bianchi.position_menu <= -5)
                    {
                        bianchi.reset_single_ride();
                    }
                    bianchi.position_menu = 0;
                }
            }
            else // relese
            {
                if (mode_menu == 0 && bianchi.info)
                {
                    bianchi.info = false;
                    bianchi.position_info = 0;
                }
                else
                {
                    mode_menu = mode_menu == 2 ? 0 : mode_menu + 1; // final
                }

                flag_go_sleep = (millis() - push_time) > sleep_level ? true : false;
            }
            last_state = state;
        }
        delay(5);
    }
}

void Taski2c(void *pvParameters)
{
    while (1)
    {
        if (flag_go_sleep || bianchi.flag_stop_sleep || bianchi.flag_nogps_sleep || bianchi.flag_i2c_reading)
        {
            delay(20);
            continue;
        }
        bianchi.flag_i2c_reading = true;
        bianchi.read_IMU();
        if (need_rTsP)
        {
            //useless chip
            need_rTsP = false;
        }
        if (need_batt)
        {
            bianchi.read_T_start_P();
            delay(20);
            bianchi.read_batt();
            bianchi.readCurrent();
            bianchi.read_P_start_T();
            need_batt = false;
        }
        if (mode_menu && bianchi.position_old != bianchi.position)
        {
            func_pisition();
        }
        if (!mode_menu)
        {
            if (bianchi.speed < 1 && bianchi.position_old > 1 && !bianchi.info)
            {
                for (int ii = bianchi.position_old; ii > 1; ii--)
                {
                    bianchi.setlumi(ii);
                    delay(5);
                }
                bianchi.setlumi(0, 0);
                bianchi.setlumi(0, 2);
                bianchi.setlumi(0, 3);
                bianchi.setlumi(0, 5);
                bianchi.setlumi(0, 6);
                bianchi.setlumi(0, 8);
                bianchi.position_old = 1;
            }
            if (bianchi.speed >= 1 && bianchi.position_old == 1 && bianchi.position > 1)
            {
                for (int ii = 1; ii < bianchi.position; ii++)
                {
                    bianchi.setlumi(ii);
                    delay(5);
                }
                bianchi.position_old = bianchi.position;
            }
        }
        bianchi.flag_i2c_reading = false;
        delay(5);
    }
}

inline void func_pisition()
{
    if (bianchi.position_old == 1 && bianchi.position > 1)
    {
        for (int ii = 1; ii < bianchi.position; ii++)
        {
            bianchi.setlumi(ii);
            delay(8);
        }
        bianchi.position_old = bianchi.position;
        return;
    }
    if (bianchi.position < 256 && bianchi.position >= 0)
    {
        bianchi.setlumi(bianchi.position);
        need_reset = false;
        flag_info = false;
    }
    if (bianchi.position < 0 && bianchi.position >= -10)
    {
        bianchi.setlumi(0);
        // if (mode_sunshine)
        // {
        //     need_dis = true;
        // }
        mode_sunshine = false;
        need_reset = false;
        flag_info = false;
    }
    if (bianchi.position < -10 && bianchi.position >= -15)
    {
        mode_sunshine = true;
        need_reset = false;
        sunshine_int = 3;
        flag_info = false;
    }

    if (bianchi.position < -15 && bianchi.position >= -20)
    {
        mode_sunshine = true;
        need_reset = false;
        sunshine_int = 0;
        flag_info = false;
    }
    // if (bianchi.position < -20 && bianchi.position >= -25) // info
    // {
    //     mode_sunshine = false;
    //     flag_info = true;
    //     need_reset = false;
    // }
    // if (bianchi.position < -25 && bianchi.position >= -30)
    // {
    //     mode_sunshine = false;
    //     need_reset = true;
    //     flag_info = false;
    // }
    bianchi.position_old = bianchi.position;
    Serial.println(bianchi.position);
}

void TaskUART(void *pvParameters)
{
    while (1)
    {
        bianchi.read_GPS();
        delay(20);
        // Serial.println("sub print");
    }
}

void update()
{
    if (!mode_menu && !bianchi.info)
    {
        return;
    }
    uint8_t p1val = DIRECT_PIN_READ(pin1_register, pin1_bitmask);
    uint8_t p2val = DIRECT_PIN_READ(pin2_register, pin2_bitmask);
    uint8_t ustate = state & 3;
    if (p1val)
        ustate |= 4;
    if (p2val)
        ustate |= 8;
    state = (ustate >> 2);
    if (mode_menu == 1)
    {
        switch (ustate)
        {
        case 1:
        case 7:
        case 8:
        case 14:
            bianchi.position += 1;
            break;
        case 2:
        case 4:
        case 11:
        case 13:
            bianchi.position -= 1;
            break;
        case 3:
        case 12:
            bianchi.position += 2;
            break;
        case 6:
        case 9:
            bianchi.position -= 2;
            break;
        }
        if ((bianchi.position - bianchi.position_old) > 8)
        {
            bianchi.position = bianchi.position_old;
        }
        bianchi.position = bianchi.position > 255 ? 255 : bianchi.position;
        bianchi.position = bianchi.position < -20 ? -20 : bianchi.position;
        return;
    }

    if (mode_menu == 2)
    {
        switch (ustate)
        {
        case 1:
        case 7:
        case 8:
        case 14:
            bianchi.position_menu += 1;
            break;
        case 2:
        case 4:
        case 11:
        case 13:
            bianchi.position_menu -= 1;
            break;
        case 3:
        case 12:
            bianchi.position_menu += 2;
            break;
        case 6:
        case 9:
            bianchi.position_menu -= 2;
            break;
        }
        bianchi.position_menu = bianchi.position_menu > 25 ? 25 : bianchi.position_menu;
        bianchi.position_menu = bianchi.position_menu < -15 ? -15 : bianchi.position_menu;
        return;
    }

    if (bianchi.info)
    {
        switch (ustate)
        {
        case 1:
        case 7:
        case 8:
        case 14:
            bianchi.position_info += 1;
            break;
        case 2:
        case 4:
        case 11:
        case 13:
            bianchi.position_info -= 1;
            break;
        case 3:
        case 12:
            bianchi.position_info += 2;
            break;
        case 6:
        case 9:
            bianchi.position_info -= 2;
            break;
        }
        bianchi.position_info = bianchi.position_info > limit_speed_dis / 10 ? limit_speed_dis / 10 : bianchi.position_info;
        bianchi.position_info = bianchi.position_info < 0 ? 0 : bianchi.position_info;
        return;
    }
}

void shut_all()
{
    detachInterrupt(22);
    detachInterrupt(5);
    detachInterrupt(9);
    // Serial.print(" before i2c wait ");
    // Serial.printf("flag:%d\n", bianchi.flag_i2c_reading);
    while (bianchi.flag_i2c_reading)
    {
        delay(1);
    }
    bianchi.flag_i2c_reading = true;
    if (task_made)
    {
        // Serial.print(" before delete ");
        vTaskDelete(taskHandleI2C);
        vTaskDelete(taskHandleUART);
        vTaskDelete(taskHandleMenu);
        taskHandleI2C = NULL;
        taskHandleUART = NULL;
        taskHandleMenu = NULL;
        task_made = false;
    }
    // Serial.print(" after shut");
    bianchi.setlumi(0);
}