#include "ZDT.hpp"
#include "arm_math.h"

using namespace ZDT;

uint8_t
cZDT::Enable(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, uint8_t enable_flag, bool syn_flag, uint8_t check_sum) {

    if (pbuf == nullptr) {
        return FDB_ERROR_POINTER;
    }

    exID = _id << 8;
    datalen = 5;

    pbuf[0] = 0xF3;
    pbuf[1] = 0xAB;
    pbuf[2] = enable_flag;
    pbuf[3] = syn_flag;
    pbuf[4] = check_sum;

    return FDB_NORMAL;
}

uint8_t cZDT::SetZeroPst(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, bool storage, uint8_t check_sum) {
    if (pbuf == nullptr) {
        return FDB_ERROR_POINTER;
    }

    exID = _id << 8;
    datalen = 4;

    pbuf[0] = 0x93;
    pbuf[1] = 0x88;
    pbuf[2] = storage;
    pbuf[4] = check_sum;

    return FDB_NORMAL;
}

uint8_t
cZDT::ZeroTrigger(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, bool move_mode, bool syn_flag, uint8_t check_sum) {
    if (pbuf == nullptr) {
        return FDB_ERROR_POINTER;
    }

    exID = _id << 8;
    datalen = 4;

    pbuf[0] = 0x9A;
    pbuf[1] = move_mode;
    pbuf[2] = syn_flag;
    pbuf[4] = check_sum;

    return FDB_NORMAL;
}

uint8_t
cZDT::TpzPst(uint32_t &exID0, uint32_t &exID1, uint32_t &datalen0, uint32_t &datalen1, uint8_t *pbuf0, uint8_t *pbuf1,
             bool dir, uint16_t accel,
             uint16_t deaccel, float maxspeed, float pst, bool opposite_mode, bool syn_flag,
             uint8_t check_sum) {
    if (pbuf0 == nullptr || pbuf1 == nullptr) {
        return FDB_ERROR_POINTER;
    }

    uint16_t _vel = (uint16_t) fabs(10 * maxspeed);
    uint32_t _pst = (uint16_t) fabs(10 * pst);

    exID0 = _id << 8;
    exID1 = _id << 8 | 0x01;
    datalen0 = 8;
    datalen1 = 8;

    pbuf0[0] = 0xFD;
    pbuf0[1] = dir;
    pbuf0[2] = accel >> 8;
    pbuf0[3] = accel & 0xFF;
    pbuf0[4] = deaccel >> 8;
    pbuf0[5] = deaccel & 0xFF;
    pbuf0[6] = _vel >> 8;
    pbuf0[7] = _vel & 0xFF;

    pbuf1[0] = 0xFD;
    pbuf1[1] = _pst >> 24 & 0xFF;
    pbuf1[2] = _pst >> 16 & 0xFF;
    pbuf1[3] = _pst >> 8 & 0xFF;
    pbuf1[4] = _pst & 0xFF;
    pbuf1[5] = opposite_mode;
    pbuf1[6] = syn_flag;
    pbuf1[7] = check_sum;

    return FDB_NORMAL;
}

uint8_t cZDT::TpzPstFDB(uint8_t *pdata, uint8_t check_sum) {
    if (pdata == nullptr) {
        return FDB_ERROR_POINTER;
    }

    if (pdata[0] != 0xFD) {
        return FDB_NOT_THIS;
    }

    if (pdata[2] != check_sum) {
        _state = STATE_ERROR;
        return FDB_ERROR_CHECK;
    }

    if (pdata[1] != 0x02) {
        _state = STATE_ERROR;
        return pdata[1];
    } else {
        _state = FDB_NORMAL;
    }

    return FDB_NORMAL;
}

uint8_t cZDT::ReadEncoder(uint32_t &exID, uint32_t &datalen, uint8_t *pbuf, uint8_t check_sum) {
    if (pbuf == nullptr) {
        return FDB_ERROR_POINTER;
    }

    exID = _id << 8;
    datalen = 2;

    pbuf[0] = 0x31;
    pbuf[1] = check_sum;
    return FDB_NORMAL;
}

uint8_t cZDT::ReadEncoderFDB(uint8_t *pdata, uint8_t check_sum) {
    if (pdata == nullptr) {
        return FDB_ERROR_POINTER;
    }

    if (pdata[0] != 0x31) {
        return FDB_NOT_THIS;
    }

    if (pdata[3] != check_sum) {
        _state = STATE_ERROR;
        return FDB_ERROR_CHECK;
    }

    if (pdata[1] != 0x02) {
        _state = pdata[1];
        return pdata[1];
    } else {
        _state = FDB_NORMAL;
    }

    _encoder = pdata[1] << 8 | pdata[2];

    return FDB_NORMAL;
}

//uint8_t cZDT::SetPID(uint32_t &exID0, uint32_t &exID1, uint32_t &exID2, uint32_t &datalen0, uint32_t &datalen1,
//                     uint32_t &datalen2, uint8_t *pbuf0, uint8_t *pbuf1, uint8_t *pbuf2,
//                     bool save_flag, uint32_t kp_trape, uint32_t kp_dirct, uint32_t kp_v, uint32_t ki_v,
//                     uint8_t check_sum) {
//
//    if ((pbuf0 == nullptr) || (pbuf1 == nullptr) || (pbuf2 == nullptr)) {
//        return FDB_ERROR_POINTER;
//    }
//
//    exID0 = _id << 8;
//    exID1 = _id << 8 | 0x01;
//    exID2 = _id << 8 | 0x02;
//    datalen0 = 8;
//    datalen1 = 8;
//    datalen2 = 8;
//
//    pbuf0[0] = 0x4A;
//    pbuf0[1] = 0xC3;
//    pbuf0[2] = save_flag;
//    pbuf0[3] = (kp_trape >> 24) & 0xFF;
//    pbuf0[4] = (kp_trape >> 16) & 0xFF;
//    pbuf0[5] = (kp_trape >> 8) & 0xFF;
//    pbuf0[6] = kp_trape & 0xFF;
//    pbuf0[7] = (kp_dirct >> 24) & 0xFF;
//
//    pbuf1[0] = 0x4A;
//    pbuf1[1] = (kp_dirct >> 16) & 0xFF;
//    pbuf1[2] = (kp_dirct >> 8) & 0xFF;
//    pbuf1[3] = kp_dirct & 0xFF;
//    pbuf1[4] = (kp_v >> 24) & 0xFF;
//    pbuf1[5] = (kp_v >> 16) & 0xFF;
//    pbuf1[6] = (kp_v >> 8) & 0xFF;
//    pbuf1[7] = kp_v & 0xFF;
//
//    pbuf2[0] = 0x4A;
//    pbuf2[1] = (ki_v >> 24) & 0xFF;
//    pbuf2[2] = (ki_v >> 16) & 0xFF;
//    pbuf2[3] = (ki_v >> 8) & 0xFF;
//    pbuf2[4] = ki_v & 0xFF;
//    pbuf2[5] = check_sum;
//
//    return FDB_NORMAL;
//}
//
//uint8_t cZDT::SetPIDFDB(uint8_t *pdata, uint8_t check_sum){
//    if (pdata == nullptr) {
//        return FDB_ERROR_POINTER;
//    }
//
//    if (pdata[0] != 0x4A) {
//        return FDB_NOT_THIS;
//    }
//
//    if (pdata[2] != check_sum) {
//        _state = STATE_ERROR;
//        return FDB_ERROR_CHECK;
//    }
//
//    if (pdata[1] != 0x02) {
//        _state = STATE_ERROR;
//        return pdata[1];
//    } else {
//        _state = FDB_NORMAL;
//    }
//
//    return FDB_NORMAL;
//}

uint8_t cZDT::TimeUpdate(uint64_t new_time) {
    _last_update_time = new_time;
    return FDB_NORMAL;
}

uint64_t cZDT::GetTime() {
    return _last_update_time;
}

uint8_t cZDT::GetState() {
    return _state;
}