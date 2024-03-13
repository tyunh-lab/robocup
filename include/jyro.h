#include <Arduino.h>
#include <Wire.h>
#include <JY901.h>

#ifndef ROBOCUP_JYRO
#define ROBOCUP_JYRO

CJY901 jy901;

// セットアップ //
void jyro_setup()
{
    Wire.setSCL(JYLO_SCL);
    Wire.setSDA(JYLO_SDA);
    jy901.StartIIC();
}

// 時間 //
void jyro_get_time()
{
    jy901.GetTime();
    Serial.print("Time:20");
    Serial.print(jy901.stcTime.ucYear);
    Serial.print("-");
    Serial.print(jy901.stcTime.ucMonth);
    Serial.print("-");
    Serial.print(jy901.stcTime.ucDay);
    Serial.print(" ");
    Serial.print(jy901.stcTime.ucHour);
    Serial.print(":");
    Serial.print(jy901.stcTime.ucMinute);
    Serial.print(":");
    Serial.print((float)jy901.stcTime.ucSecond + (float)jy901.stcTime.usMiliSecond / 1000);
}

// 加速度x, y, z//
void jyro_get_acc()
{
    jy901.GetAcc();
    Serial.print("Acc:");
    Serial.print((float)jy901.stcAcc.a[0] / 32768 * 16);
    Serial.print(" ");
    Serial.print((float)jy901.stcAcc.a[1] / 32768 * 16);
    Serial.print(" ");
    Serial.print((float)jy901.stcAcc.a[2] / 32768 * 16);
}

// 慣性x, y, z//
void jyro_get_jyro()
{
    jy901.GetGyro();
    Serial.print("Gyro:");
    Serial.print((float)jy901.stcGyro.w[0] / 32768 * 2000);
    Serial.print(" ");
    Serial.print((float)jy901.stcGyro.w[1] / 32768 * 2000);
    Serial.print(" ");
    Serial.print((float)jy901.stcGyro.w[2] / 32768 * 2000);
}

// 角度x, y, z//
void jyro_get_angle()
{
    jy901.GetAngle();
    Serial.print("Angle:");
    Serial.print((float)jy901.stcAngle.Angle[0] / 32768 * 180);
    Serial.print(" ");
    Serial.print((float)jy901.stcAngle.Angle[1] / 32768 * 180);
    Serial.print(" ");
    Serial.print((float)jy901.stcAngle.Angle[2] / 32768 * 180);
}

float get_angle()
{
    jy901.GetAngle();
    return (float)jy901.stcAngle.Angle[2] / 32768 * 180 * -1;
}

// 地磁気x, y, z//
void jyro_get_mag()
{
    jy901.GetMag();
    Serial.print("Mag:");
    Serial.print(jy901.stcMag.h[0]);
    Serial.print(" ");
    Serial.print(jy901.stcMag.h[1]);
    Serial.print(" ");
    Serial.print(jy901.stcMag.h[2]);
}

// 磁気角度x, y, z//
void jyro_get_press()
{
    jy901.GetPress();
    Serial.print("Pressure:");
    Serial.print(jy901.stcPress.lPressure);
    Serial.print(" ");
    Serial.println((float)jy901.stcPress.lAltitude / 100);
}

// 状態 //
void jyro_get_dstatus()
{
    jy901.GetDStatus();
    Serial.print("DStatus:");
    Serial.print(jy901.stcDStatus.sDStatus[0]);
    Serial.print(" ");
    Serial.print(jy901.stcDStatus.sDStatus[1]);
    Serial.print(" ");
    Serial.print(jy901.stcDStatus.sDStatus[2]);
    Serial.print(" ");
    Serial.println(jy901.stcDStatus.sDStatus[3]);
}

// 経度緯度//
void jyro_get_lonlat()
{
    jy901.GetLonLat();
    Serial.print("Longitude:");
    Serial.print(jy901.stcLonLat.lLon / 10000000);
    Serial.print("Deg");
    Serial.print((double)(jy901.stcLonLat.lLon % 10000000) / 1e5);
    Serial.print("m Lattitude:");
    Serial.print(jy901.stcLonLat.lLat / 10000000);
    Serial.print("Deg");
    Serial.print((double)(jy901.stcLonLat.lLat % 10000000) / 1e5);
    Serial.println("m");
}

// GPS//
void jyro_get_gpsv()
{
    jy901.GetGPSV();
    Serial.print("GPSHeight:");
    Serial.print((float)jy901.stcGPSV.sGPSHeight / 10);
    Serial.print("m GPSYaw:");
    Serial.print((float)jy901.stcGPSV.sGPSYaw / 10);
    Serial.print("Deg GPSV:");
    Serial.print((float)jy901.stcGPSV.lGPSVelocity / 1000);
    Serial.println("km/h");
}

#endif