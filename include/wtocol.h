#pragma once

/*
 * This is super simplified library just to handle wind sensor and nothing more
 */

#include <stdint.h>
#include <crc8.h>

#define ALIGN_TO_BYTE __attribute__((packed))

#define BUCKETS_COUNT (6)

typedef struct
{
    uint8_t sensor_id;
    uint8_t readings[BUCKETS_COUNT];
} wind_speed_sensor_t;

typedef struct
{
    uint8_t sensor_id;
    uint8_t readings[BUCKETS_COUNT];
    union {
        uint8_t directions;
        struct {
            uint8_t north_south:4;
            uint8_t west_east:4;
        };
    };
} ALIGN_TO_BYTE wind_sensor_t;

typedef struct {
    uint8_t sensor_id;
    // uint8_t payload[]; /* payload is dynamic, so exclude it from struct */
} ALIGN_TO_BYTE compound_sensors_t;

typedef struct {
    uint8_t sync_byte;
    uint8_t version;
    uint8_t device_id;
    uint8_t sensors_count;
} ALIGN_TO_BYTE weather_packet_v1_header_t;

typedef struct
{
    weather_packet_v1_header_t base;
    wind_sensor_t wind_sensor;
} ALIGN_TO_BYTE weather_wind_packet_t;

typedef struct {
    weather_packet_v1_header_t base;
    compound_sensors_t compoint_sensor;
} ALIGN_TO_BYTE weather_compound_packet_t;

typedef struct
{
    // Version
    struct
    {
        uint8_t version_zero : 4; // reserved should be 0
        uint8_t version : 4;      // should be 1
    };
    // Packet specification for version 1
    // Size, payload without CRC
    struct
    {
        uint8_t size_zero : 2; // reserved should be 0
        uint8_t size : 6;      // 0..63
    };
    // Let's this surprise me, when i gonna need more more than 15 stations
    // (1 for receiver and 14 for senders)
    struct
    {
        uint8_t packet_to : 4;   // 0..15
        uint8_t packet_from : 4; // 0..15
    };
} HC12_HDR_V1_T;

// V2 is stripped from packet direction
typedef struct
{
    // Version
    struct
    {
        uint8_t version_zero : 4; // reserved should be 0
        uint8_t version : 4;      // should be 2
    };
    // Packet specification for version 2
    // Size, payload without CRC
    struct
    {
        uint8_t size_zero : 2; // reserved should be 0
        uint8_t size : 6;      // 0..63
    };
} ALIGN_TO_BYTE HC12_HDR_V2_T;

typedef struct
{
    HC12_HDR_V2_T hdr; /* 2 */
    weather_compound_packet_t payload; /* 5 */
     /* V2: CRC: HDR + PAYLOAD */
    uint8_t crc8;
} ALIGN_TO_BYTE hc12_wire_t;

#define calculate_crc8(data, length) \
  crc8(CRC8_POLY_DVB_S2, (data), (length));

const int xx = sizeof(HC12_HDR_V2_T);
const int xz = sizeof(weather_compound_packet_t);