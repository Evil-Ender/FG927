#include <Arduino.h>
#include "math.h"
// IO
#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include "Encoder.h"
// i2c device
#include "LP50XX.h"
#include <INA219.h>
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "ICM_20948.h"
#include ".\src\MS5611.h"
#define T_OSR 0x58
#define P_OSR 0x48
// Serial device
#include <TinyGPSPlus.h>
#include <esp_timer.h>
#include <time.h>
// u8g2
// #include "C:\Users\Evil Ender\Documents\Arduino\libraries\u8g2_wqy-master\src\u8g2_wqy.h"
#include "../src/U8g2_for_Adafruit_GFX_exFont/U8g2_for_Adafruit_GFX.h"
// #include <U8g2lib.h>
// pin
#define _SCL 10
#define _SDA 8
#define _SCK 11
#define _MISO 12
#define _MOSI 13
#define _EPD_CS 1
#define _ROM_CS 0
#define _EPD_DC 2
#define _EPD_RST 3
#define _EPD_BUSY 4
#define _INT_IMU 14
#define _INT_MENU 9
#define _PIN_UP 8
#define _PIN_DOWN 10

// add
#define _ADD_INA219 0x40
#define _ADD_ICM20948 0x68
#define _ADD_AXP2101 0x34
#define _ADD_MS5611 0x77
#define _ADD_LP5009 0x14

// var
#define R_SHUNT 0.030
#define V_SHUNT_MAX 0.075
#define V_BUS_MAX 16
#define I_MAX_EXPECTED 20

// print
// #define D_BUG
#ifdef D_BUG
#define D_mail(...) Serial.print(__VA_ARGS__)
#define D_mailln(...) Serial.println(__VA_ARGS__)
#define D_mailf(...) Serial.printf(__VA_ARGS__)
#else
#define D_mail(...)
#define D_mailln(...)
#define D_mailf(...)
#endif

// map
#define suzuha 0
#define suzuha2 1
#define suzuha_lowpower 2

#define limit_stop 480
#define limit_nogps_stop 840
#define limit_speed_dis 720
#define yaw_size 10

static void icmISR();
class BIANCHI
{
protected:
    int my_white = 0xFFFF, my_black = 0x0000;
    float shuntvoltage = 0;
    float busvoltage = 0;
    volatile float current_A = 0, shut_V = 0;
    float batvoltage = 0;
    float power = 0;
    float Ah = 0;
    unsigned long lastread = 0; // used to calculate Ah
    unsigned long tick;         // current read time - last read
    // volatile bool isrFired = false;
    volatile bool sensorSleep = false;
    volatile bool canToggle = false;
    int16_t tbx, tby;
    uint16_t tbw, tbh;
    char _gps_lng[20], _gps_lat[20], _gps_time[20], _gps_speed[20];

    // shippai code          axp    INA    LP     ICM    MS
    bool shippai_code[5] = {false, false, false, false, false};
    bool _in_menu = false;
    uint8_t _add_all[5] = {_ADD_AXP2101, _ADD_INA219, _ADD_LP5009, _ADD_ICM20948, _ADD_MS5611};
    String device_name[5] = {"AXP2101", "INA219", "LP5009", "ICM20948", "MS5611"};
    //
    void calcu_stop_speed(int speed_this);
    float south_ang_diss = 0;
    int yaw_loop[yaw_size],yaw_speed_loop[yaw_size],yaw_diss_loop[yaw_size],yaw_loop_pointer=0;

public:
    BIANCHI();
    bool inst();
    bool shippai_shita();
    void shut_power();
    void conn_test();
    void readCurrent();
    void setlumi(uint8_t lumi);
    void setlumi(uint8_t limu, uint8_t led_num);
    bool display_power(int16_t x, int16_t y, bool menu);
    bool display_gps(bool ud);
    bool display_IMU();
    bool display_map(int map);
    bool display_speed(int16_t x, int16_t y);
    bool display_batt(int16_t x, int16_t y);
    bool display_time(int16_t x, int16_t y, int menu);
    bool display_menu(int16_t x, int16_t y, int menu);
    bool display_TuP(int16_t x, int16_t y);
    bool display_avg_data(int16_t x, int16_t y, bool menu);
    void display_9648(int x, int y, int num);
    void display_4824(int x, int y, int num);
    void display_fill();
    void display_info();
    void display_dis(bool need_dis);
    void display_all(int config, int menu);
    bool gps_update();
    bool init_IMU();
    bool init_DMP();
    void blink();
    bool read_IMU();
    void read_GPS();
    void reset_IMU_fifo();
    void reset_single_ride();
    void read_batt();
    void convert_YPR_and_compAngle();
    void read_T_start_P();
    void read_P_start_T();
    double convert_gps_dis(double lat_last, double lng_last, double lat, double lng);
    bool single_conn_test(uint8_t add);
    // void to_menu();
    volatile int _batt = 0, position = -5, position_old = -5, sat = 0, position_info = 0, position_menu = 0, position_mode = 0;
    volatile bool _chag = false;
    int first_IMU_read = 0;
    bool flag_en_i2c = false, flag_gps_time = false, flag_i2c_reading = false, info = false;
    static uint8_t _lumi[9];
    double qx, qy, qz, qw;
    double pres = 0, tem = 0;
    volatile float Yaw, Pitch, Roll, gx, gy, gz, Pitch_bike = 0, south_angle = 0;
    volatile double speed = 0, speed_last = 0, lat, lng, lat_last, lng_last, lat_mid, lng_mid;
    float comp_x, comp_y, comp_z;
    float comp_x_c, comp_y_c, comp_z_c;
    float kal = 0.1;
    int diss_hours = -1, diss_minutes = -1, diss_seconds = -1;
    // val about time
    volatile bool flag_gps_update = true, flag_time_power_on = false, flag_speed_update_once = false;
    int sec_flag = 0, sec_last_gps = 0, sec_last_rtc = 0;
    long int gps_update_last_time = 0;
    struct tm time_main;

    bool flag_update_speed_once_thistime = false, flag_update_time_once_thistime = false;

    int pointer_stop = 0, count_stop = 0, count_nogps_stop = 0;
    uint8_t speed_stop[limit_stop];
    int sub_speed_dis = 0;
    uint8_t speed_dis_count = 0;
    volatile bool flag_stop_sleep = false, flag_nogps_sleep = false, flag_lowpower_sleep = false;
    void read_poweron_time();
};
