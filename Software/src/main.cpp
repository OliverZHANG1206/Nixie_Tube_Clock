#include <Arduino.h>
#include <Wire.h>
#include <SHT31.h>
#include <esp_pm.h>
#include <RtcDS3231.h>
#include <OneButton.h>
#include <ESP32Encoder.h>
#include <Network_Time.h>
#include <NixieTubeClock_Reg.h>

/* Pin Define */
#define Bright_Pin 1 // Analog Pin
#define SoundSW    3 // Switch & Button & Trigger
#define Button     4  
#define BackLCtrl  5
#define CLKINT     6
#define MIC        21
#define D1         16 // Dot Display
#define D2         8
#define D3         18
#define D4         17
#define DIN        11 // Register Pin
#define LE         12
#define CLK        13
#define BL         14
#define SCL        7  // I2C Bus
#define SDA        15
#define LED        47 // LED Indicator
#define EncoderA   9  // Encoder
#define EncoderB   46
#define Encoder_SW 10

/* Status Define */
#define Display_Mode1  0
#define Display_Mode2  1
#define Display_Mode3  2
#define Display_Mode4  3
#define Setting_Mode1  4
#define Setting_Mode2  5
#define Setting_Mode3  6
#define Setting_Mode4  7
#define Setting_Mode5  8
#define Setting_Mode6  9
#define Setting_Mode7  10
#define Sleep_Mode     11
#define NixieTubeLight 0
#define BackLight      1
#define ON             1
#define OFF            0

/* Structure Define */
struct Status_handler
{
  int Display_Mode;
  int Light_Mode;
  int Sound_Mode;
  int Network_Mode;

  int Display_flag;
  int Light_flag;
  int Sleep_flag;

  int Last_BackL;
  int Last_display_mode;
  int Sound_button_status;
  int Last_sound_button_status;
};

/* Parameter Settings */
#define ADC_Freq           10      // ADC sampling freq  (Hz)
#define CLKSpeed           4000000 // SPI Freq           (Hz)
#define I2C_Freq           100000  // I2C Freq           (Hz)
#define Fresh_Freq         20      // Seeting mode frash (Hz)
#define SHT31_ADDRESS      0x44    // SHT31 Address      
#define SYNC_TIME_INTERVAL 1       // Sync Network Time Interval (Days)
#define MAX_REMAIN_TIME    10      // Sound Trigger Remain Time (s)

/* Interrupt and Timer Settings */
hw_timer_t *ADCtimer = NULL;               // Timer For ADC Sampling and Sound trigger mode detection
hw_timer_t *freshtimer = NULL;             // Fast Freshing Timer
static void IRAM_ATTR ADC_Sample(void);    // ADC Sampling Event
static void IRAM_ATTR Clock_Tick(void);    // Clock Trigger Event
static void IRAM_ATTR Sound_Trigger(void); // Sound Trigger Event
static void IRAM_ATTR Quick_Display(void); // Fast Freshing Event

/* Global Parameter */
int Timezone = 8;
int Light_value = 0;
int Sound_time_remain = 0;
int Encoder_count_now = 0;
float Humidity = 0;
float Temperature = 0;
Status_handler status;
RtcDateTime Current_time;
RtcDateTime Last_sync_time;

/* Devices */
SHT31 SHT;
Wifi_Module wifi;
ESP32Encoder Encoder;
RtcDS3231<TwoWire> RTC(Wire);
NixieTubeClock_Register NTCRegister;
OneButton EncoderSW(Encoder_SW, true, true);
OneButton Display_Button(Button, true, true);

/* Global Function & Initial Functions Declearation */
void ADC_Init(void);                  // ADC Regular Sampling Initialize
void Freshtimer_Init(void);           // When Doing Settings, fast flashing mode timer
void Sync_Network_Time(void);         // Sync Network Time
void Power_management_set(void);      // Power Management
void EncoderSW_Shortclick(void);      // Short click callback for encoder
void EncoderSW_Longclick(void);       // Long click callback for encoder
void Display_Button_Shortclick(void); // Short click callback for button
void Display_Button_Longclick(void);  // Long click callback for button


/* Core */
void setup() 
{
  /* Stage 1: Basic and Interal Settings */
  // Power Management
  Power_management_set();

  // GPIO Configuration
  pinMode(SCL, OUTPUT);
  pinMode(SDA, OUTPUT);
  pinMode(CLK, OUTPUT);
  pinMode(LE, OUTPUT);
  pinMode(BL, OUTPUT);
  pinMode(DIN, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(MIC, INPUT_PULLUP);
  pinMode(CLKINT, INPUT_PULLUP);
  pinMode(Button, INPUT_PULLUP);
  pinMode(EncoderA, INPUT_PULLUP);
  pinMode(EncoderB, INPUT_PULLUP);
  pinMode(SoundSW, INPUT_PULLUP);

  // Communication Configuration
  Serial.begin(115200);
  Wire.begin(SDA, SCL, I2C_Freq);

  // Interrupt and Timer Configuration
  ADC_Init();
  Freshtimer_Init();
  attachInterrupt(CLKINT, Clock_Tick, RISING);
  attachInterrupt(MIC, Sound_Trigger, RISING);

  // Hint
  Serial.println("GPIO and Communication Initialized Successfully");
  delay(100);

  /* Stage 2: Devices and Peripheral */
  // Nixie Tube Clock Register
  int Dot[4] = {D1, D2, D3, D4};
  status.Display_Mode = Display_Mode1;
  status.Light_Mode = NixieTubeLight;
  status.Sound_Mode = OFF;
  status.Network_Mode = Online_Mode;
  status.Display_flag = false;
  status.Light_flag = false;
  status.Sleep_flag = false;
  status.Sound_button_status = HIGH;
  status.Last_sound_button_status = -1;

  NTCRegister.Init(CLK, DIN, LE, BL, CLKSpeed);
  NTCRegister.BL_Init(BackLCtrl);
  NTCRegister.Dot_Init(4, Dot);
  Serial.println("Register Initialized Successfully");
  delay(100);

  // DS3231
  RTC.Begin();
  if (RTC.IsDateTimeValid() == false && RTC.LastError() != 0)
    Serial.printf("Initialize DS3231 Failed. Error: %d\n", RTC.LastError());
  else 
  {
    RTC.SetSquareWavePinClockFrequency(DS3231SquareWaveClock_1Hz); delay(1);
    RTC.SetSquareWavePin(DS3231SquareWavePin_ModeClock, false);    delay(1);
    Current_time = RTC.GetDateTime();
    Serial.println("DS3231 Initialized Successfully");
  }
  delay(100);

  // SHT31
  SHT.begin(SDA, SCL);
  if (SHT.isConnected())
    Serial.println("SHT30 Initialized Successfully");
  else
    Serial.printf("Initialize SHT30 Failed. Error: %d\n", SHT.getError());
  delay(100);

  // Encoder
  Encoder.attachSingleEdge(EncoderA, EncoderB);
  Encoder.clearCount();
  EncoderSW.attachClick(EncoderSW_Shortclick);
  EncoderSW.attachLongPressStart(EncoderSW_Longclick);
  Serial.println("Encoder Initialized Successfully");
  delay(100);

  // Button
  Display_Button.attachClick(Display_Button_Shortclick);
  Display_Button.attachLongPressStart(Display_Button_Longclick);
  Serial.println("Button Initialized Successfully");
  delay(100);

  // WIFI
  wifi.Init();
  wifi.Gettimestamp(Last_sync_time);
  wifi.Start_Connection(status.Network_Mode);
  if (status.Network_Mode == Online_Mode) Sync_Network_Time();
  Serial.println("Wifi Initialized Successfully");

  // Setup Finished
  delay(500);
}


void loop() 
{
  /* Button Tick */
  EncoderSW.tick();
  Display_Button.tick();

  /* Sleep mode */
  if (status.Sleep_flag)
  {
    switch (status.Display_Mode)
    {
      case Sleep_Mode:
        NTCRegister.Clear();
        NTCRegister.Bright_BackL(0);
        NTCRegister.Update_Dot(OFF, OFF, OFF, OFF);
        break;
      
      default:
        NTCRegister.Bright_BackL(status.Last_BackL);
    }
    status.Sleep_flag = false;
  }

  /* Brightnees control */
  if (status.Light_flag)
  {
    if (status.Light_Mode == NixieTubeLight)
    {
      NTCRegister.Bright_Nixie(Light_value);
      NTCRegister.Update_Dot();
    }
    else 
      NTCRegister.Bright_BackL(Light_value);
    
    status.Light_flag = false;
  }
  
  /* Number / data control */
  int array[6] = {10, 10, 10, 10, 10, 10};

  if (status.Display_flag)
  {
    switch (status.Display_Mode)
    {
      case Display_Mode1:
        NTCRegister.Update(6, Current_time.Hour() * 10000 + Current_time.Minute() * 100 + Current_time.Second()); 
        NTCRegister.Update_Dot(ON, ON, ON, ON);
        break;

      case Display_Mode2:
        NTCRegister.Update(6, (Current_time.Year() % 100) * 10000 + Current_time.Month() * 100 + Current_time.Day());
        NTCRegister.Update_Dot(OFF, ON, OFF, ON);
        break;

      case Display_Mode3:
        NTCRegister.Update(4, round(Temperature * 100));
        NTCRegister.Update_Dot(OFF, OFF, OFF, ON);
        break;

      case Display_Mode4:
        NTCRegister.Update(4, round(Humidity * 100));
        NTCRegister.Update_Dot(OFF, OFF, OFF, ON);
        break;

      case Setting_Mode1:
        array[0] = Encoder_count_now / 10;
        array[1] = Encoder_count_now % 10;
        NTCRegister.Update(array);
        break;

      case Setting_Mode2:
        array[2] = Encoder_count_now / 10;
        array[3] = Encoder_count_now % 10;
        NTCRegister.Update(array);
        break;

      case Setting_Mode3:
      case Setting_Mode5:
      case Setting_Mode6:
        NTCRegister.Update(2, Encoder_count_now);
        break;

      case Setting_Mode4:
        NTCRegister.Update(4, Encoder_count_now);
        break;

      case Setting_Mode7:
        if (Encoder_count_now < 0) array[0] = 0;
        array[4] = abs(Encoder_count_now) / 10;
        array[5] = abs(Encoder_count_now) % 10;
        NTCRegister.Update(array);
        break;

      default:
        break;
    }
    status.Display_flag = false;
    
    SHT.read();
    Temperature = SHT.getTemperature();
    Humidity = SHT.getHumidity();
  }

  /* Sound Trigger Mode */
  if (status.Sound_button_status == HIGH)
  {
    if (status.Sound_button_status != status.Last_sound_button_status) 
    {
      Serial.println("Start Sound Trigger");
      status.Sound_Mode = ON;
      status.Last_sound_button_status = status.Sound_button_status;
    }
  }
  else
  {
    if (status.Sound_button_status != status.Last_sound_button_status) 
    { 
      Serial.println("Stop Sound Trigger");
      status.Sound_Mode = OFF;
      status.Display_Mode = Display_Mode1;
      status.Last_sound_button_status = status.Sound_button_status;
    }
  }

  /* Time Regular Sync */
  if (Current_time >= Last_sync_time + SYNC_TIME_INTERVAL * 3600 * 24 && status.Network_Mode == Online_Mode)
  {
    Sync_Network_Time();
  }

  /* WIFI Offline Mode */
}

/* Function Define */
void ADC_Init(void)
{
  // ADC Settings
  analogReadResolution(Resolution);

  // Set Timer -> Regular ADC Sample
  ADCtimer = timerBegin(0, 80, true);
  timerAttachInterrupt(ADCtimer, ADC_Sample, true);
  timerAlarmWrite(ADCtimer, 1000000 / ADC_Freq, true);
  timerAlarmEnable(ADCtimer);
}

void Freshtimer_Init(void)
{
  freshtimer = timerBegin(1, 80, true);
  timerAttachInterrupt(freshtimer, Quick_Display, true);
  timerAlarmWrite(freshtimer, 1000000 / Fresh_Freq, true);
}

void Power_management_set(void)
{
  esp_pm_config_esp32s3_t pm_config = {
    .max_freq_mhz = 160,
    .min_freq_mhz = 80,
    .light_sleep_enable = true,
  };
  esp_pm_configure(&pm_config);
}

void Sync_Network_Time()
{
  static struct tm current_time;
  do
  {
    delay(100);
    wifi.Get_Network_Time(Timezone, status.Network_Mode, current_time);
  } while (mktime(&current_time) == 0);
    
  RtcDateTime sync_current_time(current_time.tm_year - 100, current_time.tm_mon + 1, current_time.tm_mday, 
                                current_time.tm_hour, current_time.tm_min, current_time.tm_sec);
    
  RTC.SetDateTime(sync_current_time);
  wifi.Save_Time(sync_current_time.TotalSeconds64());

  Last_sync_time = Current_time;
  Current_time = sync_current_time;
}

void EncoderSW_Shortclick(void)
{
  static int time_box[6] = {0};
  switch (status.Display_Mode)
  {
    case Display_Mode1:
    case Display_Mode2:
    case Display_Mode3:
    case Display_Mode4:
      status.Display_Mode = Setting_Mode1;
      NTCRegister.Update_Dot(OFF, OFF, OFF, OFF);
      Serial.println("Setting Mode");
      Serial.println("Setting - Hours");
      timerAlarmEnable(freshtimer);
      break;

    case Setting_Mode1:
      status.Display_Mode = Setting_Mode2;
      time_box[0] = Encoder.getCount();
      Encoder.clearCount();
      Serial.println("Setting - Minutes");
      break;

    case Setting_Mode2: 
      status.Display_Mode = Setting_Mode3;
      time_box[1] = Encoder.getCount();
      Encoder.clearCount();
      Serial.println("Setting - Second");
      break;

    case Setting_Mode3:
      status.Display_Mode = Setting_Mode4;
      time_box[2] = Encoder.getCount();
      Encoder.setCount(2000);
      Serial.println("Setting - Year");
      break;

    case Setting_Mode4:
      status.Display_Mode = Setting_Mode5;
      time_box[3] = Encoder.getCount();
      Encoder.clearCount();
      Serial.println("Setting - Month");
      break;

    case Setting_Mode5:
      status.Display_Mode = Setting_Mode6;
      time_box[4] = Encoder.getCount();
      Encoder.clearCount();
      Serial.println("Setting - Day");
      break;

    case Setting_Mode6:
      status.Display_Mode = Setting_Mode7;
      time_box[5] = Encoder.getCount();
      Encoder.clearCount();
      Serial.println("Setting - Timezone");
      break;

    case Setting_Mode7:
      Timezone = Encoder.getCount();
      Encoder.clearCount();
      Serial.println("Finished Settings");
      Serial.printf("%d %d %d %d %d %d\n", time_box[0], time_box[1], time_box[2], time_box[3], time_box[4], time_box[5]);

      if (time_box[3] == 0) time_box[3] = Current_time.Year();
      if (time_box[4] == 0) time_box[4] = Current_time.Month();
      if (time_box[5] == 0) time_box[5] = Current_time.Day();

      RtcDateTime modified_time(time_box[3], time_box[4], time_box[5], time_box[0], time_box[1], time_box[2]);
      RTC.SetDateTime(modified_time);
      delayMicroseconds(1000);
      Current_time = RTC.GetDateTime();
      Serial.printf("Set time: %04d-%02d-%02d %02d:%02d:%02d Timezone: ", 
                    Current_time.Year(), Current_time.Month(), Current_time.Day(), 
                    Current_time.Hour(), Current_time.Minute(), Current_time.Second());
      if (Timezone >= 0) Serial.printf("+%02d\n", Timezone);
      else Serial.printf("-%02d\n", abs(Timezone));

      timerAlarmDisable(freshtimer);
      Serial.println("Display Time");
      status.Display_Mode = Display_Mode1;
      break;
  }
}

void EncoderSW_Longclick(void)
{
  status.Light_Mode = !status.Light_Mode;
  
  if (status.Light_Mode == NixieTubeLight) 
    Serial.println("Contorl Nixie Tuble Brightness");
  else 
    Serial.println("Contorl Backlight Brightness");
}

void Display_Button_Shortclick(void)
{
  switch (status.Display_Mode)
  {
    case Display_Mode1:
      status.Display_Mode = Display_Mode2;
      Serial.println("Display Date");
      break;

    case Display_Mode2:
      status.Display_Mode = Display_Mode3;
      Serial.println("Display Temperature");
      break;

    case Display_Mode3:
      status.Display_Mode = Display_Mode4;
      Serial.println("Display Humidity");
      break;

    case Display_Mode4:
      status.Display_Mode = Display_Mode1;
      Serial.println("Display Time");
  
    default:
      break;
  }
}

void Display_Button_Longclick(void)
{
  status.Network_Mode = !status.Network_Mode;

  if (status.Network_Mode == Offline_Mode) 
    Serial.println("Offline Mode");
  else 
    Serial.println("Online Mode");
}

/* Interrupt Functions Define */
static void IRAM_ATTR ADC_Sample(void)
{
  Light_value = analogRead(Bright_Pin);
  status.Light_flag = true;
  status.Sound_button_status = digitalRead(SoundSW);
}

static void IRAM_ATTR Clock_Tick(void)
{
  if (status.Sound_Mode)
  {
    // Start sleep mode
    if (Sound_time_remain <= 0 && status.Display_Mode != Sleep_Mode)
    {
      status.Last_display_mode = status.Display_Mode;
      status.Last_BackL = ledcRead(BackL_PWM_CH);
      status.Display_Mode = Sleep_Mode;
      status.Sleep_flag = true;
    }
    // End sleep mode
    else if (Sound_time_remain > 0 && status.Display_Mode == Sleep_Mode)
    {
      status.Display_Mode = status.Last_display_mode;
      status.Sleep_flag = true;
    }
    // Not in sleep mode, still activate
    else if (Sound_time_remain > 0 && status.Display_Mode != Sleep_Mode)
      Sound_time_remain--;
  }

  switch (status.Display_Mode)
  {
    case Display_Mode1:
    case Display_Mode2:
    case Display_Mode3:
    case Display_Mode4:
      status.Display_flag = true;
      Current_time = Current_time + 1;
      break;

    default:
      Current_time = Current_time + 1;
      break;
  }
}

static void IRAM_ATTR Sound_Trigger(void)
{
  if (!status.Sound_Mode) return;
  Sound_time_remain = MAX_REMAIN_TIME;
}

static void IRAM_ATTR Quick_Display(void)
{
  switch (status.Display_Mode)
  {
    case Setting_Mode1:
    case Setting_Mode2:
    case Setting_Mode3:
    case Setting_Mode4:
    case Setting_Mode5:
    case Setting_Mode6:
    case Setting_Mode7:
      Encoder_count_now = Encoder.getCount();
      status.Display_flag = true;
    break;
  
    default:
      break;
  }
}