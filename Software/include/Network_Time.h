#ifndef NETWORK_TIME_H
#define NETWORK_TIME_H

#include <arduino.h>
#include <WiFi.h>
#include <RtcDS3231.h>
#include <Preferences.h>

#define Online_Mode   0
#define Offline_Mode  1
#define MAX_RETRY_NUM 15

#define NTP1  "cn.pool.ntp.org"
#define NTP2  "asia.pool.ntp.org"
#define NTP3  "ntp1.aliyun.com"

class Wifi_Module
{
    private:
        Preferences NVS;
        String SSID, PWD;

    public:
        int Network_Mode;
        int NetowrkSW_Pin;
        uint64_t timestamp;
        
        void Init(void);
        void Gettimestamp(RtcDateTime &);
        void Start_Connection(int &);
        void Disconnect(void);
        void Reconnect(void);
        void Smart_Config(int &);
        void Get_Network_Time(int, int &, struct tm &);
        void Set_Time_Manual(int []);
        void Save_Time(uint64_t);
};

#endif