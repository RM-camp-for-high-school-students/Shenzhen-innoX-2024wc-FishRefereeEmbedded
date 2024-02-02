/*
	DM Driver
	
	2320/8/06
	Fix temperatur decoder

	2320/7/29
	DM Driver
*/
#include "main.h"

#ifndef DMDRIVER_H
#define DMDRIVER_H
#ifdef __cplusplus

#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

/* DM4310
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define T_MIN -10.0f
#define T_MAX 10.0f
*/

/* DM4310 */
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

typedef struct {
    uint8_t state;
    float pos;
    float vel;
    float toq;
    uint8_t Tmos;
    uint8_t Tcoil;
    float Kp;
    float Kd;
} Motor_Inf;

class cDMMotor {
protected:
    uint16_t _id;
    uint8_t _txbuf[8];

    virtual uint8_t CAN_Transmit(uint16_t head, uint8_t *pdata, uint8_t len) = 0;

    Motor_Inf MTR = {0}, CMD = {0};

    uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
        /// Converts a float to an unsigned int, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return (uint16_t) ((x - offset) * ((float) ((1 << bits) - 1)) / span);
    }

    float uint_to_float(int x_int, float x_min, float x_max, int bits) {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
    }

public:
    cDMMotor(uint16_t id) : _id(id) {}

    uint8_t EnableMotor();

    uint8_t DisableMotor();

    uint8_t SetZero();

    void MITUpdate(float Position, float Velocity, float KP, float KD, float Torque);

    uint8_t MITTransmit();

    uint8_t PstVelTransmit(float pst, float vel);

    uint8_t MessageDecode(const uint8_t *buf);

    inline float GetRadian() { return this->MTR.pos; }

    inline float GetVelocity() { return this->MTR.vel; }

    inline uint8_t GetTem() { return this->MTR.Tcoil; }

    inline float GetToqReal() { return this->MTR.toq; }
};

#endif
#endif