#include "lp_can.h"

#define PI_F 3.14159265f

// Byte alto primero (big-endian), igual que put_i16() en la Pico 2 W.
static int16_t be16(const uint8_t *b) {
    return (int16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);
}

bool lp_decode(const can_frame_t *frame, pose_t *pose) {
    if (frame->id != LP_CAN_ID || frame->dlc < LP_CAN_DLC) return false;
    pose->x = be16(&frame->data[0]) / 1000.0f;
    pose->y = be16(&frame->data[2]) / 1000.0f;
    pose->theta = be16(&frame->data[4]) / 10.0f * PI_F / 180.0f;
    return true;
}

bool wp_goto_decode(const can_frame_t *frame, uint8_t *waypoint) {
    if (frame->id != WP_GOTO_ID || frame->dlc < WP_GOTO_DLC) return false;
    *waypoint = frame->data[0];
    return true;
}

can_frame_t wp_status_frame(wp_state_t state, uint8_t waypoint) {
    can_frame_t frame = { .id = WP_STATUS_ID, .dlc = WP_STATUS_DLC };
    frame.data[0] = (uint8_t)state;
    frame.data[1] = waypoint;
    return frame;
}
