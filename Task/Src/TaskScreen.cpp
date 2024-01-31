#include "TaskScreen.h"
#include "tx_api.h"
#include "main.h"

#include "spi.h"
#include "libspi-i-hal-1.0.hpp"
#include "lib-ws2812screen-1.0.hpp"

static uint8_t display_buf[12*128+1];
static SPI::cSPI spi_screen(&hspi2, nullptr, 0, UINT32_MAX);
static Screen::Display screen64(&spi_screen, display_buf, sizeof(display_buf), 8, 16);
TX_THREAD ScreenThread;
uint8_t ScreenThreadStack[512] = {0};

[[noreturn]] void ScreenThreadFun(ULONG initial_input) {
    uint8_t rgb[3] = {0, 60, 120};
    uint8_t center[2]={4,7};
    uint32_t pst[2]={0,0};
    for (;;) {
        if (++rgb[0] > 180) {
            rgb[0] = 0;
        }
        if (++rgb[1] > 180) {
            rgb[1] = 0;
        }
        if (++rgb[2] > 180) {
            rgb[2] = 0;
        }

        if(pst[0] < screen64.GetWidth()) {
            pst[0]++;
        }else{
            pst[0]=0;
        }
        if(pst[1] < screen64.GetHigh()) {
            pst[1]++;
        }else{
            pst[1]=0;
        }

        screen64.SetAllPixel(Screen::BLACK);
        screen64.SetPixel(pst[0],pst[1],rgb);


        screen64.Refresh();
        tx_thread_sleep(2);
    }
}