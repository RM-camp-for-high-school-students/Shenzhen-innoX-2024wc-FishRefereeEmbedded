/*
	DM Driver
	
	2320/8/06
	Fix temperatur decoder

	2320/7/29
	DM Driver
*/
#include "DMDriver.h"
#include "cstring"

uint8_t cDMMotor::EnableMotor()
{
    _txbuf[0]=0xFF;
    _txbuf[1]=0xFF;
    _txbuf[2]=0xFF;
    _txbuf[3]=0xFF;
    _txbuf[4]=0xFF;
    _txbuf[5]=0xFF;
    _txbuf[6]=0xFF;
    _txbuf[7]=0xFC;

	return this->CAN_Transmit(_id+0x100, _txbuf, 8);
}

uint8_t cDMMotor::DisableMotor()
{
    _txbuf[0]=0xFF;
    _txbuf[1]=0xFF;
    _txbuf[2]=0xFF;
    _txbuf[3]=0xFF;
    _txbuf[4]=0xFF;
    _txbuf[5]=0xFF;
    _txbuf[6]=0xFF;
    _txbuf[7]=0xFD;

    return this->CAN_Transmit(_id+0x100, _txbuf, 8);
}

uint8_t cDMMotor::SetZero()
{
    _txbuf[0]=0xFF;
    _txbuf[1]=0xFF;
    _txbuf[2]=0xFF;
    _txbuf[3]=0xFF;
    _txbuf[4]=0xFF;
    _txbuf[5]=0xFF;
    _txbuf[6]=0xFF;
    _txbuf[7]=0xFE;

    return this->CAN_Transmit(_id+0x100, _txbuf, 8);
}
	
void cDMMotor::MITUpdate(float Position, float Velocity, float KP, float KD, float Torque)
{
	this->CMD.pos = Position;
	this->CMD.vel = Velocity;
	this->CMD.Kp = KP;
	this->CMD.Kd = KD;
	this->CMD.toq = Torque;
}

uint8_t cDMMotor::PstVelTransmit(float pst, float vel){

    memcpy(_txbuf, &pst, 4);
    memcpy(_txbuf+4, &vel, 4);

    return this->CAN_Transmit(0x100+_id, (uint8_t*)_txbuf, 8);
}

uint8_t cDMMotor::MITTransmit()
{
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = this->float_to_uint(this->CMD.pos, P_MIN, P_MAX, 16);
	vel_tmp = this->float_to_uint(this->CMD.vel, V_MIN, V_MAX, 12);
	kp_tmp  = this->float_to_uint(this->CMD.Kp, KP_MIN, KP_MAX, 12);
	kd_tmp  = this->float_to_uint(this->CMD.Kd, KD_MIN, KD_MAX, 12);
	tor_tmp = this->float_to_uint(this->CMD.toq, T_MIN, T_MAX, 12);

    _txbuf[0] = (pos_tmp >> 8);
    _txbuf[1] = pos_tmp;
	_txbuf[2] = (vel_tmp >> 4);
	_txbuf[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	_txbuf[4] = kp_tmp;
	_txbuf[5] = (kd_tmp >> 4);
	_txbuf[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	_txbuf[7] = tor_tmp;
	
	return this->CAN_Transmit(_id, (uint8_t*)_txbuf, 8);
}

uint8_t cDMMotor::MessageDecode(const uint8_t* buf)
{
    if(buf== nullptr){
        return 1;
    }
	this->MTR.state = buf[0]>>4;
	uint16_t tmp;
	
	tmp = (buf[1]<<8)|buf[2];
	this->MTR.pos = this->uint_to_float(tmp,P_MIN,P_MAX,16);
	tmp = (buf[3]<<4)|(buf[4]>>4);
	this->MTR.vel = this->uint_to_float(tmp,V_MIN,V_MAX,12);
	tmp = ((buf[4]&0xF)<<8)|buf[5];
	this->MTR.toq = this->uint_to_float(tmp,V_MIN,V_MAX,12);
	this->MTR.Tmos = buf[6];
	this->MTR.Tcoil = buf[7];

    return 0;
}

