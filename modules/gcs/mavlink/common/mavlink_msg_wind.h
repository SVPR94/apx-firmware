#pragma once
// MESSAGE WIND PACKING

#include <stdint.h>

#define MAVLINK_MSG_ID_WIND 168

typedef struct __mavlink_wind_t {
 float direction; /*< [deg] Wind direction (that wind is coming from).*/
 float speed; /*< [m/s] Wind speed in ground plane.*/
 float speed_z; /*< [m/s] Vertical wind speed.*/
} mavlink_wind_t;

#define MAVLINK_MSG_ID_WIND_LEN 12
#define MAVLINK_MSG_ID_WIND_MIN_LEN 12
#define MAVLINK_MSG_ID_168_LEN 12
#define MAVLINK_MSG_ID_168_MIN_LEN 12

#define MAVLINK_MSG_ID_WIND_CRC 1
#define MAVLINK_MSG_ID_168_CRC 1

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_WIND { \
    168, \
    "WIND", \
    3, \
    {  { "direction", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_wind_t, direction) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_wind_t, speed) }, \
         { "speed_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wind_t, speed_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_WIND { \
    "WIND", \
    3, \
    {  { "direction", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_wind_t, direction) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_wind_t, speed) }, \
         { "speed_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wind_t, speed_z) }, \
         } \
}
#endif

static inline uint16_t mavlink_msg_wind_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float direction, float speed, float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIND_LEN];
    _mav_put_float(buf, 0, direction);
    _mav_put_float(buf, 4, speed);
    _mav_put_float(buf, 8, speed_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIND_LEN);
#else
    mavlink_wind_t packet;
    packet.direction = direction;
    packet.speed = speed;
    packet.speed_z = speed_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WIND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WIND_MIN_LEN, MAVLINK_MSG_ID_WIND_LEN, MAVLINK_MSG_ID_WIND_CRC);
}

static inline uint16_t mavlink_msg_wind_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float direction,float speed,float speed_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_WIND_LEN];
    _mav_put_float(buf, 0, direction);
    _mav_put_float(buf, 4, speed);
    _mav_put_float(buf, 8, speed_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WIND_LEN);
#else
    mavlink_wind_t packet;
    packet.direction = direction;
    packet.speed = speed;
    packet.speed_z = speed_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WIND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_WIND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WIND_MIN_LEN, MAVLINK_MSG_ID_WIND_LEN, MAVLINK_MSG_ID_WIND_CRC);
}

static inline uint16_t mavlink_msg_wind_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wind_t* wind)
{
    return mavlink_msg_wind_pack(system_id, component_id, msg, wind->direction, wind->speed, wind->speed_z);
}

static inline void mavlink_msg_wind_decode(const mavlink_message_t* msg, mavlink_wind_t* wind)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    wind->direction = _mav_get_float(msg, 0);
    wind->speed = _mav_get_float(msg, 4);
    wind->speed_z = _mav_get_float(msg, 8);
#else
    uint8_t len = msg->len < MAVLINK_MSG_ID_WIND_LEN? msg->len : MAVLINK_MSG_ID_WIND_LEN;
    memset(wind, 0, MAVLINK_MSG_ID_WIND_LEN);
    memcpy(wind, _MAV_PAYLOAD(msg), len);
#endif
}
