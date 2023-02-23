#include <Network_Time.h>

void Wifi_Module::Init()
{
    SSID = "";
    PWD = "";
    timestamp = 0;
}

void Wifi_Module::Gettimestamp(RtcDateTime &timeinfo)
{
    NVS.begin("Time", false);
    timestamp = NVS.getULong64("Second", 0);
    NVS.end();

    RtcDateTime last_time(timestamp);
    timeinfo = last_time;
}

void Wifi_Module::Start_Connection(int &Network_Status)
{
    int retry = 0;

    // Read wifi in NVS
    NVS.begin("wifi", false);
    SSID = NVS.getString("ssid", "none");
    PWD  = NVS.getString("password", "none");
    NVS.end();

    if (SSID == "none") Smart_Config(Network_Status);

    WiFi.begin(SSID.c_str(), PWD.c_str());
    while (WiFi.status() != WL_CONNECTED) 
    {
        if (Network_Status == Offline_Mode) return;

        if (retry == MAX_RETRY_NUM) 
        {
            Serial.println("Failed to Connect. Start Smartconfig");
            Smart_Config(Network_Status);
        }
        delay(1000);
        retry++;
        Serial.printf(".");
    }
    Serial.printf("Connected! IP address: ");
    Serial.println(WiFi.localIP());      
}

void Wifi_Module::Smart_Config(int &Netwrok_Status)
{
    int retry = 0;
    int waiting_time = 0;

    WiFi.mode(WIFI_AP_STA);
    WiFi.beginSmartConfig();

    Serial.println("Waiting for SmartConfig.");
    while (!WiFi.smartConfigDone()) 
    {
        if (retry >= MAX_RETRY_NUM) 
        {
            Serial.println("Stop Smartconfig. Turn to offline mode.");
            Netwrok_Status = Offline_Mode;
            return;
        }
        retry++;
        delay(4000);
        Serial.printf(".");
    }

    NVS.begin("wifi", false);
    NVS.putString( "ssid"    , WiFi.SSID());
    NVS.putString( "password", WiFi.psk());
    NVS.end();

    Serial.printf("Smartconfig Done. Received SSID: %s, PWD: %s\nReboot..", SSID, PWD);
    delay(1000);
    ESP.restart();
}

void Wifi_Module::Get_Network_Time(int timezone, int &Netwrok_Status, struct tm &timeinfo)
{
    if (WiFi.status() == WL_CONNECTED)
    {
        configTime(timezone * 3600, 0, NTP1, NTP2, NTP3);
        getLocalTime(&timeinfo);

        Serial.printf("Sync: %s\n", asctime(&timeinfo));
    }
    else if (!WiFi.status() == WL_CONNECTED)
    {
        Start_Connection(Netwrok_Status);
    }
}

void Wifi_Module::Disconnect(void)
{
    WiFi.disconnect();
}

void Wifi_Module::Reconnect(void)
{
    WiFi.reconnect();
}

void Wifi_Module::Save_Time(uint64_t time)
{
    NVS.begin("Time", false);
    NVS.putULong64("Second", time);
    NVS.end();
    Serial.printf("Save Time Successfully. Time: %u\n", time);
}