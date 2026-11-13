#include "lp_can.h"

#define PI_F 3.14159265f

static int16_t le16(const uint8_t *b) {
    return (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}

bool lp_decode(const can_frame_t *frame, pose_t *pose) {
    if (frame->id != LP_CAN_ID || frame->dlc < LP_CAN_DLC) return false;
    pose->x = le16(&frame->data[0]) / 1000.0f;
    pose->y = le16(&frame->data[2]) / 1000.0f;
    pose->theta = le16(&frame->data[4]) / 100.0f * PI_F / 180.0f;
    return true;
}
