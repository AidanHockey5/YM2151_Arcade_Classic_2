#ifndef _YM2151_H_
#define _YM2151_H_
#include "Bus.h"
#include "SpinSleep.h"
#include "clocks.h"


class YM2151
{
public:
    YM2151(Bus* _bus, uint8_t _CS, uint8_t _RD, uint8_t _WR, uint8_t _A0, uint8_t _IC);
    void write(uint8_t addr, uint8_t data);
    void reset();
    void spin(int w);
    void setClock(uint32_t frq);
    //void setYMTimerA(uint16_t value); //10 bit timer. Reg 0x24,0x25. Frq = (0x400 - TMRA) × 18.77µs
    //void clearYMTimerA(); 
    ~YM2151();
private:
    //ChipClock* clk;
    uint32_t clkfrq;
    Bus* bus;
    uint8_t CS;
    uint8_t RD;
    uint8_t WR;
    uint8_t A0;
    uint8_t IC;
};



#endif

//YM_CS = PB3
//YM_RD = PA15
//YM_WR = PA12
//YM_A0 = PA11
//YM_A1 = PB0
//YM_IC = PB4 
