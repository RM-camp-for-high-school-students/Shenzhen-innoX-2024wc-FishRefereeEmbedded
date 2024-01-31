#include "lib-ws2812screen-1.0.hpp"
#include "spi.h"

using namespace Screen;

void Display::Refresh() {
    _spi->TransmitDMA(_frame_buff, _frame_buff_len);
}

uint8_t Display::SetPixel(uint32_t x, uint32_t y, uint8_t *rgb) {
    if ((x >= _pixel_width) || (y >= _pixel_high) || (rgb == nullptr)) {
        return 1;
    }
    uint8_t rgb_t[3]={rgb[1],rgb[0],rgb[2]};
    uint32_t index = (x + y * _pixel_width) * 12;

    for (uint8_t i = 0; i < 3; i++) {
        uint32_t tmp = 0;
        uint8_t cmp = 0x80;
        for (uint8_t j = 0; j < 8; j++) {
            tmp |= ((rgb_t[i] & cmp) ? 0b1110 : 0b1000);
            cmp >>= 1;
            if(j < 7){tmp <<= 4;}
        }
        _frame_buff[index + 4 * i]     = (uint8_t) ((tmp >> 24) & 0xFF);
        _frame_buff[index + 4 * i + 1] = (uint8_t) ((tmp >> 16) & 0xFF);
        _frame_buff[index + 4 * i + 2] = (uint8_t) ((tmp >> 8) & 0xFF);
        _frame_buff[index + 4 * i + 3] = (uint8_t) ((tmp) & 0xFF);
    }
    return 0;
}

void Display::SetAllPixel(uint8_t* rgb){
    for (uint32_t i = 0; i < _pixel_width; ++i) {
        for (uint32_t j = 0; j < _pixel_high; ++j) {
            SetPixel(i,j,rgb);
        }
    }
}