#include <NixieTubeClock_Reg.h>

// Register Shifting Table
const int nixieBitTable[6][11]=
{
  { 8,  9,  0,  1,  2,  3,  4,  5,  6,  7, 24},
  {18, 19, 10, 11, 12, 13, 14, 15, 16, 17, 24},
  {32, 33, 20, 21, 22, 23, 28, 29, 30, 31, 24},
  {42, 43, 34, 35, 36, 37, 38, 39, 40, 41, 24},
  {52, 53, 44, 45, 46, 47, 48, 49, 50, 51, 24},
  {62, 63, 54, 55, 56, 57, 58, 59, 60, 61, 24}
};

void NixieTubeClock_Register::Init(int CLK, int DIN, int LE, int BL, int Speed)
{
    // Global Parameters
    Latch_En = LE;
    Block = BL;
    memset(dot_status, true, sizeof(dot_status));

    // Block Pin - Brightness Control
    ledcSetup(Bright_PWM_CH, PWM_Freq, Resolution);
    ledcAttachPin(Block, Bright_PWM_CH);
    ledcWrite(Bright_PWM_CH, 0);
    delayMicroseconds(1);
  
    // Register Init
    SPI.begin(CLK, DOUT, DIN, LE);
    SPI.beginTransaction(SPISettings(Speed, MSBFIRST, SPI_MODE0));
    SPI.setHwCs(false);
    delayMicroseconds(1);
    NixieTubeClock_Register::Clear();
}

void NixieTubeClock_Register::BL_Init(int BLpin)
{
    ledcSetup(BackL_PWM_CH, PWM_Freq, Resolution);
    ledcAttachPin(BLpin, BackL_PWM_CH);
    ledcWrite(BackL_PWM_CH, 1100);
}

void NixieTubeClock_Register::Dot_Init(int num, int dotpin[])
{
    Dot_num = num;

    ledcSetup(DotHL_PWM_CH, PWM_Dot_Freq, Resolution);
    ledcSetup(DotHR_PWM_CH, PWM_Dot_Freq, Resolution);
    ledcSetup(DotLL_PWM_CH, PWM_Dot_Freq, Resolution);
    ledcSetup(DotLR_PWM_CH, PWM_Dot_Freq, Resolution);
    ledcWrite(DotHL_PWM_CH, 0);
    ledcWrite(DotHR_PWM_CH, 0);
    ledcWrite(DotLL_PWM_CH, 0);
    ledcWrite(DotLR_PWM_CH, 0);

    switch (num)
    {
        case 1:
            ledcAttachPin(dotpin[0], DotHL_PWM_CH);
            break;

        default:
            ledcAttachPin(dotpin[0], DotHL_PWM_CH);
            ledcAttachPin(dotpin[1], DotLL_PWM_CH);
            ledcAttachPin(dotpin[2], DotHR_PWM_CH);
            ledcAttachPin(dotpin[3], DotLR_PWM_CH);
    }
}

void NixieTubeClock_Register::Update(int displayNum[])
{
    // Preparing Register Serial
    nixieBitData = 0;
    for (int i=0; i<6; i++)
    {
        int offset = nixieBitTable[i][displayNum[i]];
        nixieBitData |= ((uint64_t)1) << offset;
    }

    nixieBitDataH = nixieBitData >> 32;
    nixieBitDataL = nixieBitData & 0x00000000FFFFFFFF;
  
    // Transfer
    digitalWrite(Latch_En, HIGH);
    delayMicroseconds(1);
    SPI.transfer32(nixieBitDataH);
    SPI.transfer32(nixieBitDataL);
    delayMicroseconds(1);
    digitalWrite(Latch_En, LOW);  
}

void NixieTubeClock_Register::Update(int bit, int num)
{
    int array[6] = {10, 10, 10, 10, 10, 10};
    if (bit >= 1) array[5] = num % 10;
    if (bit >= 2) array[4] = num / 10 % 10;
    if (bit >= 3) array[3] = num / 100 % 10;
    if (bit >= 4) array[2] = num / 1000 % 10;
    if (bit >= 5) array[1] = num / 10000 % 10;
    if (bit >= 6) array[0] = num / 100000 % 10;
    //Serial.printf("%d %d %d %d %d %d\n", array[0], array[1], array[2], array[3], array[4], array[5]);
    NixieTubeClock_Register::Update(array);
}

void NixieTubeClock_Register::Update_Dot(void)
{
    int duty = ledcRead(Bright_PWM_CH);
    ledcWrite(DotHL_PWM_CH, dot_status[0] * duty);
    ledcWrite(DotLL_PWM_CH, dot_status[1] * duty);
    ledcWrite(DotHR_PWM_CH, dot_status[2] * duty);
    ledcWrite(DotLR_PWM_CH, dot_status[3] * duty);
}

void NixieTubeClock_Register::Update_Dot(int d1, int d2, int d3, int d4)
{
    dot_status[0] = d1;
    dot_status[1] = d2;
    dot_status[2] = d3;
    dot_status[3] = d4;
    NixieTubeClock_Register::Update_Dot(); 
}

void NixieTubeClock_Register::Clear(void)
{
    digitalWrite(Latch_En, HIGH);
    delayMicroseconds(1);
    SPI.transfer32((uint32_t)0);
    SPI.transfer32((uint32_t)0);
    delayMicroseconds(1);
    digitalWrite(Latch_En, LOW);
}

void NixieTubeClock_Register::Bright_Nixie(int Brightness)
{
    ledcWrite(Bright_PWM_CH, Brightness);
}

void NixieTubeClock_Register::Bright_BackL(int Brightness)
{
    ledcWrite(BackL_PWM_CH, min(Brightness, MaxBackLight));
}
