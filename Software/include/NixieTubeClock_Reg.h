#ifndef NIXIETUBECLOCK_REG_H
#define NIXIETUBECLOCK_REG_H

#include <SPI.h>
#include <Arduino.h>

// PWM parameter define
#define Bright_PWM_CH  0
#define BackL_PWM_CH   1
#define DotHL_PWM_CH   3
#define DotHR_PWM_CH   4
#define DotLL_PWM_CH   5
#define DotLR_PWM_CH   6
#define Resolution     12
#define PWM_Freq       500
#define PWM_Dot_Freq   500

#define DOUT           45
#define MaxBackLight   2800

extern const int nixieBitTable[6][11];

class NixieTubeClock_Register
{
    private:
        int Block;
        int Dot_num;
        int Latch_En;
        int dot_status[4];
        uint64_t nixieBitData;
        uint32_t nixieBitDataH;
        uint32_t nixieBitDataL;

    public:
        void Init(int, int, int, int, int);
        void Dot_Init(int, int []);
        void BL_Init(int);
        void Clear(void);
        void Update(int []);
        void Update(int, int);
        void Update_Dot(void);
        void Update_Dot(int, int, int, int);
        void Bright_Nixie(int);
        void Bright_BackL(int);
};

#endif