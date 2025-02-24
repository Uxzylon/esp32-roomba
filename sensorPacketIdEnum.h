#ifndef SENSOR_PACKET_ID_H
#define SENSOR_PACKET_ID_H

enum SensorPacketId {
    BUMPS_AND_WHEEL_DROPS = 7,
    WALL = 8,
    CLIFF_LEFT = 9,
    CLIFF_FRONT_LEFT = 10,
    CLIFF_FRONT_RIGHT = 11,
    CLIFF_RIGHT = 12,
    VIRTUAL_WALL = 13,
    WHEEL_OVERCURRENTS = 14,
    DIRT_DETECT = 15,
    INFRARED_CHARACTER_OMNI = 17,
    BUTTONS = 18,
    DISTANCE = 19,
    ANGLE = 20,
    CHARGING_STATE = 21,
    VOLTAGE = 22,
    CURRENT = 23,
    TEMPERATURE = 24,
    BATTERY_CHARGE = 25,
    BATTERY_CAPACITY = 26,
    WALL_SIGNAL = 27,
    CLIFF_LEFT_SIGNAL = 28,
    CLIFF_FRONT_LEFT_SIGNAL = 29,
    CLIFF_FRONT_RIGHT_SIGNAL = 30,
    CLIFF_RIGHT_SIGNAL = 31,
    CHARGING_SOURCES_AVAILABLE = 34,
    OI_MODE = 35,
    SONG_NUMBER = 36,
    SONG_PLAYING = 37,
    NUMBER_OF_STREAM_PACKETS = 38,
    REQUESTED_VELOCITY = 39,
    REQUESTED_RADIUS = 40,
    REQUESTED_RIGHT_VELOCITY = 41,
    REQUESTED_LEFT_VELOCITY = 42,
    LEFT_ENCODER_COUNTS = 43,
    RIGHT_ENCODER_COUNTS = 44,
    LIGHT_BUMPER = 45,
    LIGHT_BUMP_LEFT_SIGNAL = 46,
    LIGHT_BUMP_FRONT_LEFT_SIGNAL = 47,
    LIGHT_BUMP_CENTER_LEFT_SIGNAL = 48,
    LIGHT_BUMP_CENTER_RIGHT_SIGNAL = 49,
    LIGHT_BUMP_FRONT_RIGHT_SIGNAL = 50,
    LIGHT_BUMP_RIGHT_SIGNAL = 51,
    INFRARED_CHARACTER_LEFT = 52,
    INFRARED_CHARACTER_RIGHT = 53,
    LEFT_MOTOR_CURRENT = 54,
    RIGHT_MOTOR_CURRENT = 55,
    MAIN_BRUSH_MOTOR_CURRENT = 56,
    SIDE_BRUSH_MOTOR_CURRENT = 57,
    STASIS = 58
};

#endif // SENSOR_PACKET_ID_H