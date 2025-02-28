#ifndef PACKET_SIZES_H
#define PACKET_SIZES_H

#include "sensorPacketIdEnum.h"
#include <Arduino.h>

int getPacketSize(SensorPacketId packetID) {
    switch (packetID) {
        case BUMPS_AND_WHEEL_DROPS: return 1;
        case WALL: return 1;
        case CLIFF_LEFT: return 1;
        case CLIFF_FRONT_LEFT: return 1;
        case CLIFF_FRONT_RIGHT: return 1;
        case CLIFF_RIGHT: return 1;
        case VIRTUAL_WALL: return 1;
        case WHEEL_OVERCURRENTS: return 1;
        case DIRT_DETECT: return 1;
        case INFRARED_CHARACTER_OMNI: return 1;
        case BUTTONS_SENSOR: return 1;
        case DISTANCE: return 2;
        case ANGLE: return 2;
        case CHARGING_STATE: return 1;
        case VOLTAGE: return 2;
        case CURRENT: return 2;
        case TEMPERATURE: return 1;
        case BATTERY_CHARGE: return 2;
        case BATTERY_CAPACITY: return 2;
        case WALL_SIGNAL: return 2;
        case CLIFF_LEFT_SIGNAL: return 2;
        case CLIFF_FRONT_LEFT_SIGNAL: return 2;
        case CLIFF_FRONT_RIGHT_SIGNAL: return 2;
        case CLIFF_RIGHT_SIGNAL: return 2;
        case CHARGING_SOURCES_AVAILABLE: return 1;
        case OI_MODE: return 1;
        case SONG_NUMBER: return 1;
        case SONG_PLAYING: return 1;
        case NUMBER_OF_STREAM_PACKETS: return 1;
        case REQUESTED_VELOCITY: return 2;
        case REQUESTED_RADIUS: return 2;
        case REQUESTED_RIGHT_VELOCITY: return 2;
        case REQUESTED_LEFT_VELOCITY: return 2;
        case LEFT_ENCODER_COUNTS: return 2;
        case RIGHT_ENCODER_COUNTS: return 2;
        case LIGHT_BUMPER: return 1;
        case LIGHT_BUMP_LEFT_SIGNAL: return 2;
        case LIGHT_BUMP_FRONT_LEFT_SIGNAL: return 2;
        case LIGHT_BUMP_CENTER_LEFT_SIGNAL: return 2;
        case LIGHT_BUMP_CENTER_RIGHT_SIGNAL: return 2;
        case LIGHT_BUMP_FRONT_RIGHT_SIGNAL: return 2;
        case LIGHT_BUMP_RIGHT_SIGNAL: return 2;
        case INFRARED_CHARACTER_LEFT: return 1;
        case INFRARED_CHARACTER_RIGHT: return 1;
        case LEFT_MOTOR_CURRENT: return 2;
        case RIGHT_MOTOR_CURRENT: return 2;
        case MAIN_BRUSH_MOTOR_CURRENT: return 2;
        case SIDE_BRUSH_MOTOR_CURRENT: return 2;
        case STASIS: return 1;
        default: return 0;
    }
}

#endif // PACKET_SIZES_H