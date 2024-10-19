#include "BIANCHI.h"

// device
XPowersPMU myPMU;
INA219 ina219;
SPIClass hspi(FSPI);
LP50XX lp5009;
TinyGPSPlus gps;
ICM_20948_I2C ICM;
MS5611 ms5611;
RTC_DATA_ATTR struct tm time_begin, time_dis[24];
RTC_DATA_ATTR bool flag_gps_update_firsttime = false, flag_lowpower_rtc = false, flag_speed_dis_clear = false;
RTC_DATA_ATTR double diff_gps_poweron = 0, single_ride_dis = 0, single_ride_average_speed = 0;
RTC_DATA_ATTR int single_ride_time = 0, working_time = 0;
RTC_DATA_ATTR double total_ride_dis = 0;
RTC_DATA_ATTR int rtc_position = -5;
RTC_DATA_ATTR volatile uint8_t speed_dis[limit_speed_dis], speed_dis_max[limit_speed_dis], speed_max = 0;
RTC_DATA_ATTR int pointer_speed_dis_inMinute = 0;
RTC_DATA_ATTR float carlo = 0;
volatile bool isrFi1red = false;
double single_ride_dis_last = 0;
uint8_t BIANCHI::_lumi[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t speed_max_minute = 0;
U8G2_FOR_ADAFRUIT_GFX u8g2Fonts;
// EPD
#define ENABLE_GxEPD2_GFX 1
#include "map_suzu.h"
#include "map_suzu2.h"
#include "num96_48.h"
#include "num48_24.h"
#include "map.h"
#include "batt.h"
#include "ionic.h"
#include "GxEPD2_4G_BW.h"
#include "GxEPD2_4G_4G.h"
// #include "GxEPD2_4G_display_selection_new_style.h"
#include <Fonts/FreeMonoBold9pt7b.h>
// #include "GxEPD2_display_selection_new_style.h"
GxEPD2_4G_BW<GxEPD2_154_GDEY0154D67, (GxEPD2_154_GDEY0154D67::HEIGHT <= 65536ul / (GxEPD2_154_GDEY0154D67::WIDTH / 8) ? GxEPD2_154_GDEY0154D67::HEIGHT : 65536ul / (GxEPD2_154_GDEY0154D67::WIDTH / 8))> display(GxEPD2_154_GDEY0154D67(1, 2, 3, 4));
GxEPD2_4G_4G<GxEPD2_154_GDEY0154D67, (GxEPD2_154_GDEY0154D67::HEIGHT <= 65536ul / (GxEPD2_154_GDEY0154D67::WIDTH / 8) ? GxEPD2_154_GDEY0154D67::HEIGHT : 65536ul / (GxEPD2_154_GDEY0154D67::WIDTH / 8))> display_4G(GxEPD2_154_GDEY0154D67(1, 2, 3, 4));
BIANCHI::BIANCHI()
{
}

bool BIANCHI::inst()
{
    position = rtc_position;
    position_old = position;
    u8g2Fonts.setFont(u8g2_font_wqy16_t_gb2312);
    u8g2Fonts.setFontMode(1);                  // use u8g2 transparent mode (this is default)
    u8g2Fonts.setFontDirection(0);             // left to right (this is default)
    u8g2Fonts.setForegroundColor(GxEPD_WHITE); // apply Adafruit GFX color
    u8g2Fonts.setBackgroundColor(GxEPD_BLACK); // apply Adafruit GFX color
    hspi.begin(_SCK, _MISO, _MOSI, _EPD_CS);
    Wire.begin(_SDA, _SCL, 400000);
    Serial0.begin(9600, SERIAL_8N1, 23, 24);
    myPMU.begin(Wire, AXP2101_SLAVE_ADDRESS, 8, 10);
    Serial.begin(9600);
    read_batt();
    if (flag_lowpower_rtc && !_chag && _batt <= 5)
    {
        flag_lowpower_sleep = true;
        flag_i2c_reading = false;
        myPMU.setChargingLedMode(XPOWERS_CHG_LED_BLINK_4HZ);
        delay(2000);
        myPMU.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
        return true;
    }
    myPMU.disableALDO2();
    D_mailln("OHA!");
    display_4G.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    display.epd2.selectSPI(hspi, SPISettings(4000000, MSBFIRST, SPI_MODE0));
    display.init(115200, true, 2, false);
    display_4G.init(115200, true, 2, false);
    display_map(0);
    display.setRotation(0);
    display.setFont(&FreeMonoBold9pt7b);
    display.setPartialWindow(0, 0, 200, 200); // 设置部分窗口
    display.setTextColor(my_black);
    u8g2Fonts.begin(display);
    display.firstPage();
    conn_test();
    if (shippai_code[0])
    {
        D_mailln("AXP2101 transmission shippai shita!");
        return false;
    }
    // myPMU.reset();
    delay(50);
    myPMU.setALDO3Voltage(3300);
    myPMU.enableALDO3();
    myPMU.setALDO1Voltage(3300);
    myPMU.enableALDO1();
    myPMU.disableDC2();
    myPMU.disableDC3();
    myPMU.disableDC4();
    myPMU.setALDO2Voltage(1800);
    myPMU.enableALDO2();
    myPMU.setDC5Voltage(3300);
    myPMU.enableDC5();
    myPMU.setALDO4Voltage(3000);
    myPMU.enableALDO4();
    myPMU.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
    myPMU.setChargingLedMode(XPOWERS_CHG_LED_CTRL_CHG);
    myPMU.setPowerKeyPressOffTime(XPOWERS_POWEROFF_4S);
    myPMU.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_300MA);
    delay(200);
    if (shippai_shita())
    {
        return false;
    }

    lp5009.Begin(_ADD_LP5009);
    lp5009.SetScaling(LOG_SCALE_ON);
    setlumi(64, 0);
    setlumi(64, 1);
    setlumi(64, 2);
    delay(100);
    // inst INA
    ina219.begin();
    ina219.configure(INA219::RANGE_16V, INA219::GAIN_2_80MV, INA219::ADC_16SAMP, INA219::ADC_16SAMP, INA219::CONT_SH_BUS);
    ina219.calibrate(R_SHUNT, V_SHUNT_MAX, V_BUS_MAX, I_MAX_EXPECTED);
    lastread = millis();
    ms5611.setI2Caddr(_ADD_MS5611);
    ms5611.set_wire();
    ms5611.ReadProm();
    ms5611.start_mesure(T_OSR);
    delay(100);
    while (!init_IMU())
    {
    }
    setlumi(64, 3);
    setlumi(64, 4);
    setlumi(64, 5);
    while (!init_DMP())
    {
    }
    setlumi(64, 6);
    setlumi(64, 7);
    setlumi(64, 8);
    delay(500);
    blink();
    count_stop = 0;
    for (int ii = 0; ii < limit_stop; ii++)
    {
        speed_stop[ii] = 0;
    }
    if (!flag_speed_dis_clear)
    {
        for (int ii = 0; ii < limit_speed_dis; ii++)
        {
            speed_dis[ii] = 0;
            D_mailln(speed_dis[ii]);
        }
        for (int ii = 0; ii < limit_speed_dis; ii++)
        {
            speed_dis_max[ii] = 0;
            D_mailln(speed_dis_max[ii]);
        }
        tm time_clear;
        time_clear.tm_hour = 0;
        time_clear.tm_min = 0;
        time_clear.tm_sec = 0;
        for (int ii = 0; ii < limit_speed_dis / 60 * 2; ii++)
        {
            time_dis[ii] = time_clear;
        }
        flag_speed_dis_clear = true;
    }
    flag_stop_sleep = false;
    flag_nogps_sleep = false;
    return true;
}
bool BIANCHI::init_IMU()
{
    pinMode(_INT_IMU, INPUT_PULLUP); // Using a pullup b/c ICM-20948 Breakout board has an onboard pullup as well and we don't want them to compete
    // attachInterrupt(digitalPinToInterrupt(_INT_IMU), icmISR, FALLING); // Set up a falling interrupt
    bool initialized = false;
    while (!initialized)
    {
        ICM.begin(Wire, 0);
        // D_mail(F("Initialization of the sensor returned: "));
        // D_mailln(ICM.statusString());
        if (ICM.status != ICM_20948_Stat_Ok)
        {
            D_mailln("initialized ICM20948 shippai shita!");
            delay(100);
            return false;
        }
        else
        {
            initialized = true;
        }
    }
    ICM.swReset();
    delay(100);
    D_mailln("ICM20948 connected!");
    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

    myFSS.a = gpm8; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                    // gpm2
                    // gpm4
                    // gpm8
                    // gpm16

    myFSS.g = dps1000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                       // dps250
                       // dps500
                       // dps1000
                       // dps2000

    ICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
    return true;
}
bool BIANCHI::init_DMP()
{
    bool success = true;
    success &= (ICM.initializeDMP() == ICM_20948_Stat_Ok);
    success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
    success &= (ICM.enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);
    success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 4) == ICM_20948_Stat_Ok);        // Set to 10Hz
    success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 4) == ICM_20948_Stat_Ok);        // Set to 10Hz
    success &= (ICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 4) == ICM_20948_Stat_Ok); // Set to 10Hz
    success &= (ICM.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (ICM.enableDMP() == ICM_20948_Stat_Ok);
    success &= (ICM.resetDMP() == ICM_20948_Stat_Ok);
    success &= (ICM.resetFIFO() == ICM_20948_Stat_Ok);
    if (success)
    {
        D_mailln(F("DMP enabled!"));
        return true;
    }
    else
    {
        D_mailln(F("Enable DMP failed!"));
        D_mailln(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        return false;
    }
}
void BIANCHI::shut_power()
{
    myPMU.disableALDO1();
    if (flag_stop_sleep)
    {
        ICM.cfgIntActiveLow(true);  // Active low to be compatible with the breakout board's pullup resistor
        ICM.cfgIntOpenDrain(false); // Push-pull, though open-drain would also work thanks to the pull-up resistors on the breakout
        ICM.cfgIntLatch(true);      // Latch the interrupt until cleared

        ICM.WOMLogic(1, 1);
        ICM.WOMThreshold(75);
        ICM.intEnableWOM(true);
    }
    else
    {
        myPMU.disableALDO2();
        myPMU.disableALDO3();
        // myPMU.disableALDO4();
    }
    if (flag_stop_sleep)
    {
        rtc_position = position;
    }
    else
    {
        rtc_position = -5;
    }
    pinMode(14, INPUT_PULLUP);
    while (!digitalRead(14))
    {
        ICM.clearInterrupts();
        delay(100);
    }
    myPMU.disableDC5();
}
bool BIANCHI::shippai_shita()
{
    conn_test();
    bool shippai = false;
    for (int ii = 0; ii < 5; ii++)
    {
        if (shippai_code[ii])
        {
            D_mail(device_name[ii]);
            D_mailln(" transmission shippai shita!");
            shippai = true;
        }
    }
    if (!shippai)
    {
        D_mailln("Okey dokey!");
    }
    return shippai;
}
void BIANCHI::conn_test()
{
    flag_i2c_reading = true;
    for (int ii = 0; ii < 5; ii++)
    {
        shippai_code[ii] = false;
    }
    for (int ii = 0; ii < 5; ii++)
    {
        Wire.beginTransmission(_add_all[ii]);
        if (Wire.endTransmission())
        {
            shippai_code[ii] = true;
        }
    }
    flag_i2c_reading = false;
    return;
}
bool BIANCHI::single_conn_test(uint8_t add)
{
    flag_i2c_reading = true;
    Wire.beginTransmission(add);
    if (Wire.endTransmission())
    {
        flag_i2c_reading = false;
        return false;
    }
    flag_i2c_reading = false;
    return true;
}
void BIANCHI::readCurrent() // something isn't right
{
    uint32_t count = 0;
    unsigned long newtime;
    busvoltage = ina219.busVoltage();
    shuntvoltage = ina219.shuntVoltage() * 1000;
    current_A = ina219.shuntCurrent();
    batvoltage = busvoltage + (shuntvoltage / 1000);
    power = ina219.busPower();
    newtime = millis();
    tick = newtime - lastread;
    Ah += (current_A * tick) / 3600000.0;
    lastread = newtime;
}
void BIANCHI::read_T_start_P()
{
    ms5611.get_T_5611();
    ms5611.start_mesure(P_OSR);
}
void BIANCHI::read_P_start_T()
{
    ms5611.get_P_5611();
    pres = ms5611.calcu_P_5611();
    tem = ms5611.GetTemp();
    D_mailf("T:%f,P:%f\n", tem, pres);
    ms5611.start_mesure(T_OSR);
}
void BIANCHI::read_GPS()
{
    while (Serial0.available() > 0)
    {
        int sth = Serial0.read();
        gps.encode(sth);
        // Serial.printf("%c", sth);
    }
    double speed_tmp = sat == 0 ? 0 : gps.speed.kmph();
    speed = speed_tmp; // speed_tmp < 1.5 ? 0 : speed_tmp;
    // speed = rand() % 40;
    // flag_update_speed_once_thistime = true; ////////
    sec_flag = gps.time.second();
    if (sec_flag != 0)
    {
        flag_update_time_once_thistime = true;
    }
    if (sec_flag != sec_last_gps && (sec_flag != 0 || flag_update_time_once_thistime) && flag_gps_update == false)
    {
        sec_last_gps = sec_flag;
        struct tm gpsTime;
        gpsTime.tm_year = gps.date.year();
        gpsTime.tm_mon = gps.date.month();  // Month, where 0 = jan
        gpsTime.tm_mday = gps.date.day();   // Day of the month
        gpsTime.tm_hour = gps.time.hour();  // Hours since midnight - [0,23]
        gpsTime.tm_min = gps.time.minute(); // Minutes after the hour - [0,59]
        gpsTime.tm_sec = sec_flag;          // Seconds after the minute - [0,59]
        if (flag_gps_update_firsttime == false && gpsTime.tm_year > 2020)
        {
            struct tm time_tmp;
            getLocalTime(&time_tmp);
            time_t timetmp_t = mktime(&time_tmp), time_begin_t = mktime(&time_begin);
            diff_gps_poweron = difftime(timetmp_t, time_begin_t);
            // Serial.printf("time_begin:%d:%d:%d:%d:%d:%d.time_tmp:%d:%d:%d:%d:%d:%d.diff time:%f\n", /*asdf*/
            //               time_begin.tm_year, time_begin.tm_mon, time_begin.tm_mday, time_begin.tm_hour, time_begin.tm_min, time_begin.tm_sec,
            //               time_tmp.tm_year, time_tmp.tm_mon, time_tmp.tm_mday, time_tmp.tm_hour, time_tmp.tm_min, time_tmp.tm_sec,
            //               diff_gps_poweron);
            time_begin = gpsTime;
            flag_gps_update_firsttime = true;
        }
        if (flag_gps_update_firsttime)
        {
            if (speed != 0)
            {
                flag_speed_update_once = true;
                single_ride_average_speed += (speed + speed_last) / 2;
                single_ride_time++;

                lat = gps.location.lat();
                lng = gps.location.lng();
                double dis = convert_gps_dis(lat_last, lng_last, lat, lng);

                // D_mailf("lat:%f,lng:%f,aqu:%f,dis:%f\n", lat, lng, gps.hdop.hdop(), dis);
                if (dis > 10 && dis < 100)
                {
                    lat_last = lat;
                    lng_last = lng;
                    single_ride_dis += dis;
                    total_ride_dis += dis;
                }
                else if (dis >= 100)
                {
                    lat_last = lat;
                    lng_last = lng;
                }
            }

            // dis
            sub_speed_dis += (speed + speed_last) / 2;
            speed_max = speed > speed_max ? speed : speed_max;
            speed_max_minute = speed > speed_max_minute ? speed : speed_max_minute;
            speed_dis_count++;
            D_mailf("max:%d,sub:%d,count:%d\n", speed_max, sub_speed_dis, speed_dis_count);
            if (speed_dis_count >= 60)
            {
                speed_dis_count = 0;
                uint8_t speed_one_minute = sub_speed_dis / 60;
                sub_speed_dis = 0;
                // after mean and reset pointer, time to % u /
                uint8_t pointer_hour = pointer_speed_dis_inMinute / 60;
                uint8_t pointer_hour_flag = pointer_speed_dis_inMinute % 60;
                // after % u /, time to write data
                speed_dis[pointer_speed_dis_inMinute] = speed_one_minute;
                speed_dis_max[pointer_speed_dis_inMinute] = speed_max_minute;
                speed_max_minute = 0;
                // now hour
                if (!pointer_hour_flag)
                {
                    time_dis[pointer_hour] = time_main;
                }
                if (pointer_speed_dis_inMinute % 60 == 0)
                {
                    int dis_hour = pointer_speed_dis_inMinute / 60;
                    time_dis[dis_hour] = time_main;
                }
                // col
                float col_one_minute = 0.021 * speed_one_minute * speed_one_minute + 0.25;
                carlo += col_one_minute / 60 * 70;
                // single_ride_dis_last = single_ride_dis;
                pointer_speed_dis_inMinute++;
                pointer_speed_dis_inMinute = pointer_speed_dis_inMinute == limit_speed_dis ? 0 : pointer_speed_dis_inMinute;
            }
            speed_last = speed;
            //
            working_time++;
            flag_gps_update = true;
            gps_update_last_time = millis();
            time_t t = mktime(&gpsTime);
            struct timeval now = {.tv_sec = t};
            settimeofday(&now, NULL);
            time_main = gpsTime;
        }
    }
    if (((millis() - gps_update_last_time) > 2000) || !flag_gps_update_firsttime)
    {
        struct tm time_rtc;
        getLocalTime(&time_rtc);
        sec_flag = time_rtc.tm_sec;
        // Serial.printf("%d,%d,",sec_flag,sec_last_rtc);
        if ((sec_flag != sec_last_rtc) && (flag_gps_update == false))
        {
            // Serial.println("true");
            sec_last_rtc = sec_flag;
            working_time++;
            flag_gps_update = true;
            time_main = time_rtc;
        }
    }
}
void BIANCHI::read_poweron_time()
{
    getLocalTime(&time_begin);
    diff_gps_poweron = 0;
}
bool BIANCHI::read_IMU()
{
    icm_20948_DMP_data_t data;
    ICM.readDMPdataFromFIFO(&data);
    if ((ICM.status == ICM_20948_Stat_Ok) || (ICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
        if ((data.header & DMP_header_bitmap_Quat9) > 0) // Check for Gyro
        {
            qx = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            qy = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
            qz = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
            qw = sqrt(1.0 - ((qx * qx) + (qy * qy) + (qz * qz)));
        }

        if ((data.header & DMP_header_bitmap_Compass) > 0) // Check for Compass
        {
            comp_x = (float)data.Compass.Data.X; // Extract the compass data
            comp_y = (float)data.Compass.Data.Y;
            comp_z = (float)data.Compass.Data.Z;
        }

        if ((data.header & DMP_header_bitmap_Compass_Calibr) > 0) // Check for Compass
        {
            comp_x_c = (float)data.Compass_Calibr.Data.X; // Extract the compass data
            comp_y_c = (float)data.Compass_Calibr.Data.Y;
            comp_z_c = (float)data.Compass_Calibr.Data.Z;
        }
        convert_YPR_and_compAngle();
        return true;
    }
    else
    {
        return false;
    }
}
void BIANCHI::read_batt()
{
    _batt = myPMU.getBatteryPercent();
    _chag = myPMU.isCharging();
    if (_chag)
    {
        read_poweron_time();
        working_time = 0;
    }
    // D_mailf("batt: %d\n", _batt);
}
void BIANCHI::reset_IMU_fifo()
{
    ICM.resetFIFO();
}
void BIANCHI::reset_single_ride()
{
    single_ride_average_speed = 0;
    single_ride_dis = 0;
    single_ride_time = 0;
    for (int ii = 0; ii < limit_speed_dis; ii++)
    {
        speed_dis[ii] = 0;
    }
    pointer_speed_dis_inMinute = 0;
    sub_speed_dis = 0;
    speed_dis_count = 0;
    tm time_clear;
    time_clear.tm_hour = 0;
    time_clear.tm_min = 0;
    time_clear.tm_sec = 0;
    for (int ii = 0; ii < limit_speed_dis / 60 * 2; ii++)
    {
        time_dis[ii] = time_clear;
    }
    for (int ii = 0; ii < limit_speed_dis; ii++)
    {
        speed_dis_max[ii] = 0;
        D_mailln(speed_dis_max[ii]);
    }
    speed_max = 0;
    speed_max_minute = 0;
    carlo = 0;
}
void BIANCHI::convert_YPR_and_compAngle()
{
    gx = 2 * (qx * qz - qw * qy);
    gy = 2 * (qw * qx + qy * qz);
    gz = qw * qw - qx * qx - qy * qy + qz * qz;
    Yaw = atan2(2 * qx * qy - 2 * qw * qz, 2 * qw * qw + 2 * qx * qx - 1) * 180 / PI;
    Pitch = atan2(gx, sqrt(gy * gy + gz * gz)) * 180 / PI;
    Roll = atan2(gy, gz) * 180 / PI;
    float south_Yaw = Yaw;
    south_angle = south_Yaw > 180 ? south_Yaw - 360 : south_Yaw;

    // float _comp_x = comp_x_c / 100000;
    // float _comp_y = comp_y_c / 100000;
    // float _comp_z = comp_z_c / 100000;
    // float moo = _comp_x * _comp_x + _comp_z * _comp_z;
    // float mo = _comp_x * _comp_x + _comp_y * _comp_y + _comp_z * _comp_z;
    // float anN = acos(moo / mo);
    // anN = anN * 180 / PI;
    // moo = _comp_y * _comp_y + _comp_z * _comp_z;
    // mo = _comp_x * _comp_x + _comp_y * _comp_y + _comp_z * _comp_z;
    // float anE = acos(moo / mo);
    // anE = anE * 180 / PI;
    // anE = 90 - anE;
    // // float an = (anE + anN) / 2;
    // float an = anN > 45 ? anE : anN;
    // if ((_comp_x < 0 & (Pitch < 45)) || (_comp_z > 0 & Pitch > 45))
    // {
    //     an = 180 - an;
    // }
    // if (comp_y > 0)
    // {
    //     an = -an;
    // }
    // else
    // {
    //     an = an;
    // }
    if (first_IMU_read < 10)
    {
        if (!isnan(Pitch))
        {
            first_IMU_read++;
            // south_angle = an;
            Pitch_bike = Pitch;
            // D_mail("pb:");
            // D_mailln(Pitch_bike);
        }
    }
    // else
    // {
    //     if (isnan(south_angle))
    //     {
    //         south_angle = an;
    //     }
    //     else
    //     {
    //         south_angle = (1 - kal) * south_angle + kal * an;
    //     }
    // }
}
double BIANCHI::convert_gps_dis(double lat_last, double lng_last, double lat, double lng)
{
    double phy1 = lat_last * PI / 180;
    double phy2 = lat * PI / 180;
    double lambda1 = lng_last * PI / 180;
    double lambda2 = lng * PI / 180;
    double delta_phy = phy2 - phy1;
    double delta_lambda = lambda2 - lambda1;
    double some_a = sin(delta_phy / 2) * sin(delta_phy / 2) + cos(phy1) * cos(phy2) * sin(delta_lambda / 2) * sin(delta_lambda / 2);
    double some_c = 2 * asin(sqrt(some_a));
    double dis = 6371 * 1000 * some_c;
    return dis;
}
void BIANCHI::setlumi(uint8_t lumi)
{
    for (int ii = 0; ii < 9; ii++)
    {
        lp5009.SetOutputColor(ii, lumi);
        _lumi[ii] = lumi;
    }
    return;
}
void BIANCHI::setlumi(uint8_t lumi, uint8_t led_num)
{
    lp5009.SetOutputColor(led_num, lumi);
    _lumi[led_num] = lumi;
    return;
}
bool BIANCHI::display_power(int16_t x, int16_t y, bool menu)
{
    char cpower[10];
    if (menu)
    {
        display.getTextBounds(cpower, 0, 0, &tbx, &tby, &tbw, &tbh);
        // center the bounding box by transposition of the origin:
        if (current_A < 0)
        {
            sprintf(cpower, "%.0fmA", current_A * 1000);
        }
        else
        {
            sprintf(cpower, "%.0fmW", current_A * 1000 * busvoltage);
        }
        display.setCursor(x, y);
        display.print(cpower);
    }
    else
    {
        sat = gps.satellites.value();
        display.drawBitmap(x + 3, y - 14, ionicx16_0, 16, 16, my_black);
        display.setCursor(x + 27, y);
        display.print(sat);
    }
    return true;
    // display.nextPage();
}
bool BIANCHI::display_gps(bool ud)
{
    // bool _need_display = false;
    int x, y;
    // // Serial.print(Serial0.read());
    // // 检查并打印经纬度
    // if (gps.location.isUpdated())
    // {
    //     D_mail("Latitude: ");
    //     D_mailln(gps.location.lat(), 6);
    //     D_mail("Longitude: ");
    //     D_mailln(gps.location.lng(), 6);
    //     D_mailf(_gps_lat, "lat.%.6f", gps.location.lat());
    //     D_mailf(_gps_lng, "lng.%.6f", gps.location.lng());
    //     _need_display = true;
    // }

    // // 检查并打印时间
    // if (gps.time.isUpdated())
    // {
    //     // D_mail("Time: ");
    //     // D_mail(gps.time.hour());
    //     // D_mail(":");
    //     // D_mail(gps.time.minute());
    //     // D_mail(":");
    //     // D_mailln(gps.time.second());
    //     int _hour = gps.time.hour();
    //     _hour = (_hour + 8 > 24) ? _hour - 16 : _hour + 8;
    //     D_mailf("UTC+8 %d:%d:%d", _hour, gps.time.minute(), gps.time.second());
    //     _need_display = true;
    // }

    // if (gps.speed.isUpdated())
    // {
    //     float _speed = gps.speed.kmph();
    //     D_mailf(_gps_speed, "spd.%.1fkm/h", _speed);
    // }

    display.getTextBounds(_gps_time, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((display.width() - tbw) / 2) - tbx;
    y = ((display.height() - tbh) / 2) - tby;
    // display.setFullWindow();
    display.setCursor(x, y - 20); // 设置开始绘制的坐标
    display.print(_gps_time);
    // display.setCursor(x, y - 30); // 设置开始绘制的坐标
    // display.print("UTC+8");
    display.getTextBounds(_gps_lat, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((display.width() - tbw) / 2) - tbx;
    y = ((display.height() - tbh) / 2) - tby;
    display.setCursor(x, y);
    display.print(_gps_lat);
    display.setCursor(x, y + 20);
    display.print(_gps_lng);
    display.getTextBounds(_gps_speed, 0, 0, &tbx, &tby, &tbw, &tbh);
    x = ((display.width() - tbw) / 2) - tbx;
    y = ((display.height() - tbh) / 2) - tby;
    display.setCursor(x, y + 40);
    display.print(_gps_speed);
    // display.nextPage();
    return true;
}
void BIANCHI::display_fill()
{
    display.fillRect(0, 0, 200, 200, my_white);
}
bool BIANCHI::display_IMU()
{
    double south_angle_gps = gps.course.deg();
    south_angle_gps = south_angle_gps > 180 ? south_angle_gps - 360 : south_angle_gps;

    float south_angle_display = south_angle;
    yaw_loop_pointer = yaw_loop_pointer == yaw_size ? 0 : yaw_loop_pointer;
    yaw_loop[yaw_loop_pointer] = (int)south_angle_display;
    yaw_speed_loop[yaw_loop_pointer] = (int)speed;
    yaw_diss_loop[yaw_loop_pointer] = (int)south_angle_display - (int)south_angle_gps;
    yaw_loop_pointer++;
    int min_yaw = 1000, max_yaw = 0, min_speed = 1000;
    for (int ii = 0; ii < yaw_size; ii++)
    {
        min_yaw = yaw_loop[ii] < min_yaw ? yaw_loop[ii] : min_yaw;
        max_yaw = yaw_loop[ii] > max_yaw ? yaw_loop[ii] : max_yaw;
        min_speed = yaw_speed_loop[ii] < min_speed ? yaw_speed_loop[ii] : min_speed;
    }
    int yaw_thr = (max_yaw - min_yaw) <= 180 ? (max_yaw - min_yaw) : (360 - (max_yaw - min_yaw));
    bool corr = false;
    if ((min_speed > 10) && (yaw_thr < 22))
    {
        int pointer_tmp = (yaw_loop_pointer + yaw_size / 2) >= yaw_size ? (yaw_loop_pointer - yaw_size / 2) : (yaw_loop_pointer + yaw_size / 2);
        south_ang_diss = yaw_diss_loop[pointer_tmp];
        corr = true;
    }
    south_angle_display = south_angle_display - south_ang_diss;

    // for (int ii = 0; ii < yaw_size; ii++)
    // {
    //     Serial.print(yaw_loop[ii]);
    //     Serial.print(" ");
    // }
    // Serial.println(yaw_thr);
    // for (int ii = 0; ii < yaw_size; ii++)
    // {
    //     Serial.print(yaw_speed_loop[ii]);
    //     Serial.print(" ");
    // }
    // Serial.println(min_speed);
    // Serial.println();

    display.fillRect(0, 170, 200, 30, my_white);
    display.fillRect(160, 0, 40, 200, my_white);
    int hig = 22, wig = 4, x, y;
    int ang_pix, south_angle_right_pix = round(south_angle_display) - 25, _pitch = round(Pitch - Pitch_bike);
    // display.setPartialWindow(0, 200 - hig, 200, hig);
    // display.fillRect(0, 170, 200, 30, my_white);
    for (int ii = 0; ii < 50; ii++) // 1 degree = 4 pixel
    {
        ang_pix = south_angle_right_pix + ii;
        ang_pix = ang_pix < -180 ? ang_pix + 360 : ang_pix;
        ang_pix = ang_pix > 180 ? ang_pix - 360 : ang_pix;
        if ((ang_pix) % 5 == 0 && (ang_pix) % 45 != 0)
        {
            display.fillRect(ii * 4, 193, wig, 7, my_black);
        }
        if ((ang_pix) % 45 == 0)
        {
            display.fillRect(ii * 4, 186, wig, 14, my_black);
        }
        switch (ang_pix)
        {
        case 0:
            display.getTextBounds("N", 0, 0, &tbx, &tby, &tbw, &tbh);
            x = (ii * 4 + wig / 2 - tbw / 2) - tbx;
            y = (200 - hig - tbh / 2) - tby;
            display.setCursor(x, y);
            display.print("N");
            break;
        case -45:
            display.getTextBounds("NW", 0, 0, &tbx, &tby, &tbw, &tbh);
            x = (ii * 4 + wig / 2 - tbw / 2) - tbx;
            y = (200 - hig - tbh / 2) - tby;
            display.setCursor(x, y);
            display.print("NW");
            break;
        case -90:
            display.getTextBounds("W", 0, 0, &tbx, &tby, &tbw, &tbh);
            x = (ii * 4 + wig / 2 - tbw / 2) - tbx;
            y = (200 - hig - tbh / 2) - tby;
            display.setCursor(x, y);
            display.print("W");
            break;
        case -135:
            display.getTextBounds("SW", 0, 0, &tbx, &tby, &tbw, &tbh);
            x = (ii * 4 + wig / 2 - tbw / 2) - tbx;
            y = (200 - hig - tbh / 2) - tby;
            display.setCursor(x, y);
            display.print("SW");
            break;
        case -180:
            display.getTextBounds("S", 0, 0, &tbx, &tby, &tbw, &tbh);
            display.fillRect(ii * 4 + wig / 4, 188, wig / 2, 12, my_white);
            x = (ii * 4 + wig / 2 - tbw / 2) - tbx;
            y = (200 - hig - tbh / 2) - tby;
            display.setCursor(x, y);
            display.print("S");
            break;
        case 180:
            display.getTextBounds("S", 0, 0, &tbx, &tby, &tbw, &tbh);
            display.fillRect(ii * 4 + wig / 4, 188, wig / 2, 12, my_white);
            x = (ii * 4 + wig / 2 - tbw / 2) - tbx;
            y = (200 - hig - tbh / 2) - tby;
            display.setCursor(x, y);
            display.print("S");
            break;
        case 135:
            display.getTextBounds("SE", 0, 0, &tbx, &tby, &tbw, &tbh);
            x = (ii * 4 + wig / 2 - tbw / 2) - tbx;
            y = (200 - hig - tbh / 2) - tby;
            display.setCursor(x, y);
            display.print("SE");
            break;
        case 90:
            display.getTextBounds("E", 0, 0, &tbx, &tby, &tbw, &tbh);
            x = (ii * 4 + wig / 2 - tbw / 2) - tbx;
            y = (200 - hig - tbh / 2) - tby;
            display.setCursor(x, y);
            display.print("E");
            break;
        case 45:
            display.getTextBounds("NE", 0, 0, &tbx, &tby, &tbw, &tbh);
            x = (ii * 4 + wig / 2 - tbw / 2) - tbx;
            y = (200 - hig - tbh / 2) - tby;
            display.setCursor(x, y);
            display.print("NE");
            break;
        }
    }
    if (corr)
    {
        display.setCursor(10, y);
        display.print("C");
    }
    for (int ii = -20; ii < 21; ii = ii + 5)
    {
        display.fillRect(192, 85 + ii * 4 - wig / 2, 7, wig, my_black);
    }
    display.fillRect(186, 85 - wig / 2, 14, wig, my_black);
    display.fillRect(172, 85 - _pitch * 4 - wig / 2, 10, wig, my_black);
    display.getTextBounds("20", 0, 0, &tbx, &tby, &tbw, &tbh);
    x = (185 - tbw) - tbx;
    y = (5 - tbh / 2) - tby;
    display.setCursor(x, y);
    display.print("20");
    display.getTextBounds("-20", 0, 0, &tbx, &tby, &tbw, &tbh);
    x = (185 - tbw) - tbx;
    y = (165 - tbh / 2) - tby;
    display.setCursor(x, y);
    display.print("-20");
    return true;
}
bool BIANCHI::display_map(int map)
{
    switch (map)
    {
    case 0:
        display_4G.setFullWindow();
        display_4G.drawGreyPixmap(bitmap_suzu, 4, 0, 0, 200, 200);
        display_4G.display();
        break;
    case 1:
        display_4G.init(115200, true, 2, false);
        display_4G.setFullWindow();
        display_4G.fillScreen(my_white);
        display_4G.display();
        display_4G.drawGreyPixmap(bitmap_suzu2, 4, 0, 0, 200, 200);
        display_4G.display();
        break;
    case 2:
        // myPMU.setChargingLedMode()
        if (!flag_lowpower_rtc)
        {
            display_4G.init(115200, true, 2, false);
            display_4G.setFullWindow();
            display_4G.fillScreen(my_white);
            display_4G.display();
            display_4G.drawGreyPixmap(bitmap_suzu_lowpower, 4, 0, 0, 200, 200);
            display_time((int16_t)64, (int16_t)16, 1);
            display_4G.display();
            flag_lowpower_rtc = true;
        }
        break;
    case 100:
        display_4G.init(115200, true, 2, false);
        display_4G.setFullWindow();
        display_4G.fillScreen(my_white);
        display_4G.display();
        display_4G.drawGreyPixmap(bit_map, 4, 0, 0, 200, 200);
        display_4G.display();
        break;
    }

    return true;
}
bool BIANCHI::display_batt(int16_t x, int16_t y)
{
    char pre[10];
    if (_batt <= 100 && _batt >= 0)
    {
        display.fillRect(x, y, 24, 13, my_white);
        display.fillRect(x + 28, y - 1, 8, 15, my_white);
    }
    if (_batt > 75 && _batt <= 100)
    {
        display.drawBitmap(x, y, batt4, 24, 13, my_black);
    }
    if (_batt > 50 && _batt <= 75)
    {
        display.drawBitmap(x, y, batt3, 24, 13, my_black);
    }
    if (_batt > 25 && _batt <= 50)
    {
        display.drawBitmap(x, y, batt2, 24, 13, my_black);
    }
    if (_batt > 10 && _batt <= 25)
    {
        display.drawBitmap(x, y, batt1, 24, 13, my_black);
    }
    if (_batt >= 0 && _batt <= 10)
    {
        display.drawBitmap(x, y, batt0, 24, 13, my_black);
    }
    if (_batt >= 0 && _batt <= 5)
    {
        flag_lowpower_sleep = true;
    }
    else
    {
        flag_lowpower_sleep = false;
        flag_lowpower_rtc = false;
    }
    if (_chag)
    {
        display.drawBitmap(x + 28, y - 1, batt5, 8, 15, my_black);
    }
    else
    {
        // sprintf(pre,"%d",_batt);
        // display.setCursor(x+25,y+11);
        // display.print(pre);
    }
    return true;
}
bool BIANCHI::display_time(int16_t x, int16_t y, int menu)
{
    int hour = -1, minute = -1, second = -1;
    char time[10];
    time_t timemain_t = mktime(&time_main), time_begin_t = mktime(&time_begin);
    double difference = difftime(timemain_t, time_begin_t) + diff_gps_poweron;
    // D_mailf("in dis:time_begin:%d:%d:%d:%d:%d:%d.time_main:%d:%d:%d:%d:%d:%d.diff time:%f,diff poweron time:%f\n", /*asdf*/
    //               time_begin.tm_year, time_begin.tm_mon, time_begin.tm_mday, time_begin.tm_hour, time_begin.tm_min, time_begin.tm_sec,
    //               time_main.tm_year, time_main.tm_mon, time_main.tm_mday, time_main.tm_hour, time_main.tm_min, time_main.tm_sec,
    //               difference, diff_gps_poweron);
    if (flag_gps_update_firsttime)
    {
        hour = time_main.tm_hour;
        minute = time_main.tm_min;
        second = time_main.tm_sec;
        hour = hour + 8 >= 24 ? hour - 16 : hour + 8;
    }

    diss_hours = (int)difference / 3600;
    diss_hours = diss_hours < 0 ? -1 : diss_hours;
    if (diss_hours > 99)
    {
        diss_hours = 99;
    }
    diss_minutes = ((int)difference % 3600) / 60;
    diss_seconds = (int)difference % 60;

    u8g2Fonts.setFont(u8g2_font_profont22_mf);
    u8g2Fonts.setForegroundColor(my_black); // apply Adafruit GFX color
    u8g2Fonts.setBackgroundColor(my_white); // apply Adafruit GFX color
    sprintf(time, "%2d:%2d:%2d", hour, minute, second);
    display.getTextBounds(time, x, y, &tbx, &tby, &tbw, &tbh);
    display.fillRect(x, y, tbw, tbh, my_white);
    u8g2Fonts.setCursor(x, y);
    u8g2Fonts.print(time);

    if (menu)
    {
        int single_ride_hour = working_time / 3600;
        int single_ride_minute = (working_time % 3600) / 60;
        int single_ride_second = working_time % 60;
        sprintf(time, "%2d:%2d:%2d", single_ride_hour, single_ride_minute, single_ride_second);
    }
    else
    {
        int single_ride_hour = single_ride_time / 3600;
        int single_ride_minute = (single_ride_time % 3600) / 60;
        int single_ride_second = single_ride_time % 60;
        sprintf(time, "%2d:%2d:%2d", single_ride_hour, single_ride_minute, single_ride_second);
    }

    display.getTextBounds(time, x, y, &tbx, &tby, &tbw, &tbh);
    // display.fillRect(x, y + 20, tbw, tbh, my_white);
    u8g2Fonts.setCursor(x, y + 18);
    u8g2Fonts.print(time);
    // sprintf(time, "%2d:%2d:%2d", diss_hours, diss_minutes, diss_seconds);
    // display.getTextBounds(time, x, y, &tbx, &tby, &tbw, &tbh);
    // display.fillRect(x, y + 13, tbw, tbh, my_white);
    // display.setCursor(x, y + 13);
    // display.print(time);
    return true;
}
bool BIANCHI::display_speed(int16_t x, int16_t y)
{
    // double speed_test = 10.5;
    calcu_stop_speed((int)speed);
    double tmp_speed = speed;
    if (((int)(speed / 10) % 10) != 0)
    {
        display_9648(x, y, (int)(speed / 10) % 10);
    }
    display_9648(x + 48 + 8, y, (int)speed % 10);
    display_4824(x + 48 + 8 + 48 + 8, y + 48, (int)(speed * 10) % 10);
    return true;
}
bool BIANCHI::display_menu(int16_t x, int16_t y, int menu_token)
{
    char menu[15];
    u8g2Fonts.setForegroundColor(my_white); // apply Adafruit GFX color
    u8g2Fonts.setBackgroundColor(my_black); // apply Adafruit GFX color
    if (menu_token == 1)
    {
        u8g2Fonts.setCursor(x, y);
        sprintf(menu, "LIGH");
        int up = u8g2Fonts.getFontAscent(), dw = u8g2Fonts.getFontDescent();

        display.fillRect(x - 4, y - up + dw, 40, 30, my_white);

        display.fillRect(x - 4, y - up + dw, u8g2Fonts.getUTF8Width(menu) + 6, up - dw * 2, my_black);
        u8g2Fonts.print(menu);
        u8g2Fonts.setForegroundColor(my_black); // apply Adafruit GFX color
        u8g2Fonts.setBackgroundColor(my_white); // apply Adafruit GFX color
        if (position >= 0)
        {
            u8g2Fonts.setFont(u8g2_font_open_iconic_weather_2x_t);
            menu[0] = 66;
            menu[1] = 0;
            u8g2Fonts.setCursor(x - 4, y + 24);
            u8g2Fonts.print(menu);
            u8g2Fonts.setFont(u8g2_font_profont22_mf);
            sprintf(menu, ":%d", position);
            u8g2Fonts.setCursor(x + 11, y + 24);
            u8g2Fonts.print(menu);
        }
        if (position < 0 && position >= -10)
        {
        }
        if (position < -10 && position >= -15)
        {
            u8g2Fonts.setFont(u8g2_font_open_iconic_weather_2x_t);
            menu[0] = 69;
            menu[1] = 0;
            u8g2Fonts.setCursor(x - 4, y + 24);
            u8g2Fonts.print(menu);
            u8g2Fonts.setFont(u8g2_font_profont22_mf);
            sprintf(menu, ":4s");
            u8g2Fonts.setCursor(x + 11, y + 24);
            u8g2Fonts.print(menu);
        }
        if (position < -15 && position >= -20)
        {
            u8g2Fonts.setFont(u8g2_font_open_iconic_weather_2x_t);
            menu[0] = 69;
            menu[1] = 0;
            u8g2Fonts.setCursor(x - 4, y + 24);
            u8g2Fonts.print(menu);
            u8g2Fonts.setFont(u8g2_font_profont22_mf);
            sprintf(menu, ":2s");
            u8g2Fonts.setCursor(x + 11, y + 24);
            u8g2Fonts.print(menu);
        }
    }
    if (menu_token == 2)
    {
        u8g2Fonts.setCursor(x, y);
        sprintf(menu, "MENU");
        int up = u8g2Fonts.getFontAscent(), dw = u8g2Fonts.getFontDescent();

        display.fillRect(x - 4, y - up + dw, 40, 30, my_white);

        display.fillRect(x - 4, y - up + dw, u8g2Fonts.getUTF8Width(menu) + 6, up - dw * 2, my_black);
        u8g2Fonts.print(menu);
        u8g2Fonts.setForegroundColor(my_black); // apply Adafruit GFX color
        u8g2Fonts.setBackgroundColor(my_white); // apply Adafruit GFX color
        if (position_menu > 15)
        {
            u8g2Fonts.setFont(u8g2_font_open_iconic_mime_2x_t);
            menu[0] = 79;
            menu[1] = 0;
            u8g2Fonts.setCursor(x - 4, y + 24);
            u8g2Fonts.print(menu);
            u8g2Fonts.setFont(u8g2_font_profont22_mf);
            sprintf(menu, "RIMU");
            u8g2Fonts.setCursor(x, y + 24);
            u8g2Fonts.print(menu);
        }
        if (position_menu > 5 && position_menu <= 15)
        {
            u8g2Fonts.setFont(u8g2_font_open_iconic_mime_2x_t);
            menu[0] = 79;
            menu[1] = 0;
            u8g2Fonts.setCursor(x - 4, y + 24);
            u8g2Fonts.print(menu);
            u8g2Fonts.setFont(u8g2_font_profont22_mf);
            sprintf(menu, "INFO");
            u8g2Fonts.setCursor(x, y + 24);
            u8g2Fonts.print(menu);
        }
        if (position_menu <= -5)
        {
            u8g2Fonts.setFont(u8g2_font_open_iconic_mime_2x_t);
            menu[0] = 79;
            menu[1] = 0;
            u8g2Fonts.setCursor(x - 4, y + 24);
            u8g2Fonts.print(menu);
            u8g2Fonts.setFont(u8g2_font_profont22_mf);
            sprintf(menu, "RST?");
            u8g2Fonts.setCursor(x, y + 24);
            u8g2Fonts.print(menu);
        }
    }
    return true;
}
bool BIANCHI::display_TuP(int16_t x, int16_t y)
{
    char TuP[10];
    float TC = tem / 100, PC = pres;
    float TF = (TC * 9 / 5) + 32;
    // float H = 44300 * (1 - pow(pres / 100782, 1 / 5.256));
    float H = gps.altitude.meters();
    u8g2Fonts.setFont(u8g2_font_profont17_mf);
    u8g2Fonts.setCursor(x - 2, y - 5);
    sprintf(TuP, "T%.1fC", TC - 10);
    u8g2Fonts.print(TuP);
    u8g2Fonts.setCursor(x - 2, y + 10);
    sprintf(TuP, "A%.0fm", H);
    u8g2Fonts.print(TuP);
    u8g2Fonts.setCursor(x - 2, y + 25);
    sprintf(TuP, "P%.0fh", pres / 100);
    u8g2Fonts.print(TuP);
    u8g2Fonts.setFont(u8g2_font_profont22_mf);
    return true;
}
bool BIANCHI::display_avg_data(int16_t x, int16_t y, bool menu)
{
    if (!menu)
    {
        char avg[15];
        sprintf(avg, "Dst:%.2fKM", single_ride_dis / 1000);
        u8g2Fonts.setCursor(x, y + 18);
        u8g2Fonts.print(avg);
        sprintf(avg, "ASpd:%.1fKPH", single_ride_average_speed / single_ride_time);
        u8g2Fonts.setCursor(x, y);
        u8g2Fonts.print(avg);
    }
    else
    {
        char avg[15];
        sprintf(avg, "Total Dst:");
        u8g2Fonts.setCursor(x, y);
        u8g2Fonts.print(avg);
        sprintf(avg, "%.1fKM", total_ride_dis / 1000);
        u8g2Fonts.setCursor(x, y + 18);
        u8g2Fonts.print(avg);
    }
    return true;
}
void BIANCHI::display_9648(int x, int y, int num)
{
    int w = 48, h = 96;
    switch (num)
    {
    case 0:
        display.drawBitmap(x, y, num96_480, w, h, my_black);
        break;
    case 1:
        display.drawBitmap(x, y, num96_481, w, h, my_black);
        break;
    case 2:
        display.drawBitmap(x, y, num96_482, w, h, my_black);
        break;
    case 3:
        display.drawBitmap(x, y, num96_483, w, h, my_black);
        break;
    case 4:
        display.drawBitmap(x, y, num96_484, w, h, my_black);
        break;
    case 5:
        display.drawBitmap(x, y, num96_485, w, h, my_black);
        break;
    case 6:
        display.drawBitmap(x, y, num96_486, w, h, my_black);
        break;
    case 7:
        display.drawBitmap(x, y, num96_487, w, h, my_black);
        break;
    case 8:
        display.drawBitmap(x, y, num96_488, w, h, my_black);
        break;
    case 9:
        display.drawBitmap(x, y, num96_489, w, h, my_black);
        break;
    case 10:
        display.drawBitmap(x, y, num96_4810, w, h, my_black);
        break;
    }
}
void BIANCHI::display_4824(int x, int y, int num)
{
    int w = 24, h = 48;
    switch (num)
    {
    case 0:
        display.drawBitmap(x, y, num48_240, w, h, my_black);
        break;
    case 1:
        display.drawBitmap(x, y, num48_241, w, h, my_black);
        break;
    case 2:
        display.drawBitmap(x, y, num48_242, w, h, my_black);
        break;
    case 3:
        display.drawBitmap(x, y, num48_243, w, h, my_black);
        break;
    case 4:
        display.drawBitmap(x, y, num48_244, w, h, my_black);
        break;
    case 5:
        display.drawBitmap(x, y, num48_245, w, h, my_black);
        break;
    case 6:
        display.drawBitmap(x, y, num48_246, w, h, my_black);
        break;
    case 7:
        display.drawBitmap(x, y, num48_247, w, h, my_black);
        break;
    case 8:
        display.drawBitmap(x, y, num48_248, w, h, my_black);
        break;
    case 9:
        display.drawBitmap(x, y, num48_249, w, h, my_black);
        break;
    }
}
void BIANCHI::display_info()
{
    int speed_max_info = speed_max > 10 ? speed_max : 10;
    display.fillRect(0, 35, 165, 200, my_white);
    u8g2Fonts.setFont(u8g2_font_profont22_mf);
    u8g2Fonts.setCursor(4, 34);
    u8g2Fonts.printf("%dKPH", speed_max_info);
    display.fillRect(10, 38, 180, 2, my_black);
    int position_inloop = position_info * 10;
    for (int ii = 0; ii < 180; ii++)
    {
        int pointer_loop = pointer_speed_dis_inMinute - ii - position_inloop - 1;
        // Serial.print(position_info);
        // Serial.print(" ");
        // Serial.print(pointer_loop);
        // Serial.print(" ");
        pointer_loop = pointer_loop < 0 ? pointer_loop + limit_speed_dis : pointer_loop;
        pointer_loop = pointer_loop < 0 ? pointer_loop + limit_speed_dis : pointer_loop;
        // Serial.println(pointer_loop);
        int fxxk = speed_dis[pointer_loop];
        int hi = fxxk * 80 / speed_max_info;
        fxxk = speed_dis_max[pointer_loop];
        int hi_max = fxxk * 80 / speed_max_info;
        // Serial.printf("fxxk:%d,l:%d", fxxk, speed_max_info);
        if (ii == 0 && position_inloop == 0)
        {
            display.fillRect(187, 30, 6, 10, my_black);
            display.fillRect(189, 32, 2, 6, my_white);
        }
        if (pointer_loop % 60 == 0)
        {
            int dis_hour = pointer_loop / 60;
            tm time_this_pixel = time_dis[dis_hour];
            uint8_t hour = time_this_pixel.tm_hour;
            uint8_t minute = time_this_pixel.tm_min;
            uint8_t second = time_this_pixel.tm_sec;
            bool NtYt = !hour && !minute && !second;
            hour = hour + 8 >= 24 ? hour - 16 : hour + 8;
            u8g2Fonts.setCursor(190 - ii - 21, 140);
            if (ii > 10 && ii < 170)
            {
                u8g2Fonts.setFont(u8g2_font_profont17_mf);
                if (NtYt)
                {
                    u8g2Fonts.print("Nt:Yt");
                }
                else
                {
                    u8g2Fonts.printf("%02d:%02d", hour, minute);
                }
                u8g2Fonts.setFont(u8g2_font_profont22_mf);
            }
            display.fillRect(190 - ii - 1, 120, 3, 6, my_black);
        }
        display.fillRect(190 - ii, 120 - hi, 1, hi, my_black);
        if (hi_max != 0 && hi_max < 79)
        {
            int hi_max_al = (80 - hi - hi_max > 4) ? (80 - hi_max) : (80 - hi_max - 3);
            display.fillRect(190 - ii, 40, 1, hi_max_al, my_black);
        }
    }
    u8g2Fonts.setFont(u8g2_font_profont17_mf);
    u8g2Fonts.setCursor(4, 200);
    u8g2Fonts.printf("Total Dst:%.0fKM", total_ride_dis / 1000);
    u8g2Fonts.setCursor(4, 185);
    u8g2Fonts.printf("Single Dst:%.1fKM", single_ride_dis / 1000);
    u8g2Fonts.setCursor(4, 170);
    u8g2Fonts.printf("Energy:%.0fCAL", carlo);
    u8g2Fonts.setCursor(4, 155);
    u8g2Fonts.printf("Avrg Spd:%.1fKPH", single_ride_average_speed / single_ride_time);
}
void BIANCHI::display_dis(bool need_dis)
{
    if (need_dis)
    {
        display.display();
        delay(333);
        display.firstPage();
    }
    else
    {
        display.nextPage();
    }
}
void BIANCHI::display_all(int config, int menu)
{
    if (config)
    {
        my_black = 0x0000;
        my_white = 0xFFFF;
        display.setTextColor(GxEPD_BLACK);
    }
    else
    {
        my_black = 0xFFFF;
        my_white = 0x0000;
        display.setTextColor(GxEPD_WHITE);
    }
    display_fill();
    display_time((int16_t)64, (int16_t)16, menu);
    display_batt((int16_t)2, (int16_t)2);
    if (info)
    {
        display_info();
    }
    else
    {
        display_speed((int16_t)10, (int16_t)38);
        display_power(2, 32, menu);
        display_avg_data((int16_t)10, (int16_t)152, menu);
        display_IMU();
        if (menu)
        {
            display_menu((int16_t)122, (int16_t)56, menu);
        }
        else
        {
            display_TuP((int16_t)122, (int16_t)56);
        }
    }
}
bool BIANCHI::gps_update()
{
    return gps.time.isUpdated();
}
void BIANCHI::blink()
{
    setlumi(0);
    delay(100);
    setlumi(64);
    delay(100);
    setlumi(0);
}

void BIANCHI::calcu_stop_speed(int speed_this)
{
    if (gps.location.isUpdated())
    {
        flag_update_speed_once_thistime = true;
    }
    float speed_all = 0, speed_mean = 0;
    speed_stop[pointer_stop] = speed_this;
    pointer_stop++;
    pointer_stop = pointer_stop >= limit_stop ? 0 : pointer_stop;

    if (count_stop < limit_stop)
    {
        count_stop++;
    }
    if (!flag_update_speed_once_thistime && (count_stop > (limit_stop / 4 * 3)))
    {
        count_stop = limit_stop / 4 * 3;
        count_nogps_stop++;
    }

    for (int ii = 0; ii < limit_stop; ii++)
    {
        speed_all += speed_stop[ii];
    }
    speed_mean = speed_all / limit_stop;
    D_mailf("flag1:%d,flag2:%d,flag3:%d  ", speed_mean < 0.5, count_stop == limit_stop, ((speed_mean < 0.5) && (count_stop == limit_stop)));
    if ((speed_mean < 0.5) && (count_stop == limit_stop))
    {
        flag_stop_sleep = true;
    }
    if (count_nogps_stop > limit_nogps_stop)
    {
        flag_nogps_sleep = true;
    }

    D_mailf("count:%d,count no gps:%d,mean:%f,flag speed:%d,flag gps:%d\n", count_stop, count_nogps_stop, speed_mean, flag_stop_sleep, flag_update_speed_once_thistime);
}