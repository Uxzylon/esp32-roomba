#ifndef SENSOR_PACKETS_H
#define SENSOR_PACKETS_H

#include <Arduino.h>

enum SensorPacketId
{
    GROUP_0 = 0,
    GROUP_1 = 1,
    GROUP_2 = 2,
    GROUP_3 = 3,
    GROUP_4 = 4,
    GROUP_5 = 5,
    GROUP_6 = 6,
    BUMPS_AND_WHEEL_DROPS = 7,
    WALL = 8,
    CLIFF_LEFT = 9,
    CLIFF_FRONT_LEFT = 10,
    CLIFF_FRONT_RIGHT = 11,
    CLIFF_RIGHT = 12,
    VIRTUAL_WALL = 13,
    WHEEL_OVERCURRENTS = 14,
    DIRT_DETECT = 15,
    UNUSED_1 = 16,
    INFRARED_CHARACTER_OMNI = 17,
    BUTTONS_SENSOR = 18,
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
    UNUSED_2 = 32,
    UNUSED_3 = 33,
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
    STASIS = 58,
    GROUP_100 = 100,
    GROUP_101 = 101,
    GROUP_106 = 106,
    GROUP_107 = 107
};

// Struct to hold packet information: ID, name, and size
struct SensorPacketInfo
{
    SensorPacketId id;
    const char *name;
    int size;
    bool isTwoBytes; // Whether the value is a 2-byte value
    bool isSigned;   // Whether the value is signed
};

// Array of sensor packet information
const SensorPacketInfo sensorPackets[] = {
    {BUMPS_AND_WHEEL_DROPS, "bumps_wheeldrops", 1, false, false},
    {WALL, "wall", 1, false, false},
    {CLIFF_LEFT, "cliff_left", 1, false, false},
    {CLIFF_FRONT_LEFT, "cliff_front_left", 1, false, false},
    {CLIFF_FRONT_RIGHT, "cliff_front_right", 1, false, false},
    {CLIFF_RIGHT, "cliff_right", 1, false, false},
    {VIRTUAL_WALL, "virtual_wall", 1, false, false},
    {WHEEL_OVERCURRENTS, "wheel_overcurrents", 1, false, false},
    {DIRT_DETECT, "dirt_detect", 1, false, false},
    {UNUSED_1, "unused_1", 1, false, false},
    {INFRARED_CHARACTER_OMNI, "infrared_omni", 1, false, false},
    {BUTTONS_SENSOR, "buttons", 1, false, false},
    {DISTANCE, "distance", 2, true, true},
    {ANGLE, "angle", 2, true, true},
    {CHARGING_STATE, "charging_state", 1, false, false},
    {VOLTAGE, "voltage", 2, true, false},
    {CURRENT, "current", 2, true, true},
    {TEMPERATURE, "temperature", 1, false, true},
    {BATTERY_CHARGE, "battery_charge", 2, true, false},
    {BATTERY_CAPACITY, "battery_capacity", 2, true, false},
    {WALL_SIGNAL, "wall_signal", 2, true, false},
    {CLIFF_LEFT_SIGNAL, "cliff_left_signal", 2, true, false},
    {CLIFF_FRONT_LEFT_SIGNAL, "cliff_front_left_signal", 2, true, false},
    {CLIFF_FRONT_RIGHT_SIGNAL, "cliff_front_right_signal", 2, true, false},
    {CLIFF_RIGHT_SIGNAL, "cliff_right_signal", 2, true, false},
    {UNUSED_2, "unused_2", 1, false, false},
    {UNUSED_3, "unused_3", 2, false, false},
    {CHARGING_SOURCES_AVAILABLE, "charging_sources_available", 1, false, false},
    {OI_MODE, "oi_mode", 1, false, false},
    {SONG_NUMBER, "song_number", 1, false, false},
    {SONG_PLAYING, "song_playing", 1, false, false},
    {NUMBER_OF_STREAM_PACKETS, "number_of_stream_packets", 1, false, false},
    {REQUESTED_VELOCITY, "requested_velocity", 2, true, true},
    {REQUESTED_RADIUS, "requested_radius", 2, true, true},
    {REQUESTED_RIGHT_VELOCITY, "requested_right_velocity", 2, true, true},
    {REQUESTED_LEFT_VELOCITY, "requested_left_velocity", 2, true, true},
    {LEFT_ENCODER_COUNTS, "left_encoder_counts", 2, true, false},
    {RIGHT_ENCODER_COUNTS, "right_encoder_counts", 2, true, false},
    {LIGHT_BUMPER, "light_bumper", 1, false, false},
    {LIGHT_BUMP_LEFT_SIGNAL, "light_bump_left_signal", 2, true, false},
    {LIGHT_BUMP_FRONT_LEFT_SIGNAL, "light_bump_front_left_signal", 2, true, false},
    {LIGHT_BUMP_CENTER_LEFT_SIGNAL, "light_bump_center_left_signal", 2, true, false},
    {LIGHT_BUMP_CENTER_RIGHT_SIGNAL, "light_bump_center_right_signal", 2, true, false},
    {LIGHT_BUMP_FRONT_RIGHT_SIGNAL, "light_bump_front_right_signal", 2, true, false},
    {LIGHT_BUMP_RIGHT_SIGNAL, "light_bump_right_signal", 2, true, false},
    {INFRARED_CHARACTER_LEFT, "infrared_character_left", 1, false, false},
    {INFRARED_CHARACTER_RIGHT, "infrared_character_right", 1, false, false},
    {LEFT_MOTOR_CURRENT, "left_motor_current", 2, true, true},
    {RIGHT_MOTOR_CURRENT, "right_motor_current", 2, true, true},
    {MAIN_BRUSH_MOTOR_CURRENT, "main_brush_motor_current", 2, true, true},
    {SIDE_BRUSH_MOTOR_CURRENT, "side_brush_motor_current", 2, true, true},
    {STASIS, "stasis", 1, false, false}};

// Function to get the packet size
int getPacketSize(SensorPacketId packetID)
{
    for (const auto &sensor : sensorPackets)
    {
        if (sensor.id == packetID)
        {
            return sensor.size;
        }
    }

    // Default sizes for groups or unrecognized IDs
    switch (packetID)
    {
    case GROUP_0:
        return 26;
    case GROUP_1:
        return 10;
    case GROUP_2:
        return 6;
    case GROUP_3:
        return 10;
    case GROUP_4:
        return 14;
    case GROUP_5:
        return 12;
    case GROUP_6:
        return 52;
    case GROUP_100:
        return 80;
    case GROUP_101:
        return 28;
    case GROUP_106:
        return 12;
    case GROUP_107:
        return 9;
    default:
        return 0;
    }
}

// Function to get sensor name from ID
const char *getSensorName(SensorPacketId packetID)
{
    for (const auto &sensor : sensorPackets)
    {
        if (sensor.id == packetID)
        {
            return sensor.name;
        }
    }
    return ""; // Empty string for unrecognized IDs
}

// Function to parse sensor value based on type
int parseSensorValue(SensorPacketId packetID, byte *sensorData)
{
    for (const auto &sensor : sensorPackets)
    {
        if (sensor.id == packetID)
        {
            if (sensor.size == 2 && sensor.isTwoBytes)
            {
                // Handle 2-byte values
                if (sensor.isSigned)
                {
                    return (int16_t)((sensorData[0] << 8) | sensorData[1]);
                }
                else
                {
                    return (sensorData[0] << 8) | sensorData[1];
                }
            }
            else if (sensor.size == 1 && sensor.isSigned)
            {
                // Handle 1-byte signed values
                return (int8_t)sensorData[0];
            }
            else
            {
                // Handle 1-byte values
                return sensorData[0];
            }
        }
    }
    return 0; // Default value for unrecognized IDs
}

// Group packet definitions
struct GroupPacket
{
    SensorPacketId id;
    int packetSize;
    SensorPacketId packets[52];
    int packetCount;
};

const GroupPacket groupPackets[] = {
    {GROUP_0, 26, {BUMPS_AND_WHEEL_DROPS, WALL, CLIFF_LEFT, CLIFF_FRONT_LEFT, CLIFF_FRONT_RIGHT, CLIFF_RIGHT, VIRTUAL_WALL, WHEEL_OVERCURRENTS, DIRT_DETECT, UNUSED_1, INFRARED_CHARACTER_OMNI, BUTTONS_SENSOR, DISTANCE, ANGLE, CHARGING_STATE, VOLTAGE, CURRENT, TEMPERATURE, BATTERY_CHARGE, BATTERY_CAPACITY}, 20},
    {GROUP_1, 10, {BUMPS_AND_WHEEL_DROPS, WALL, CLIFF_LEFT, CLIFF_FRONT_LEFT, CLIFF_FRONT_RIGHT, CLIFF_RIGHT, VIRTUAL_WALL, WHEEL_OVERCURRENTS, DIRT_DETECT, UNUSED_1}, 10},
    {GROUP_2, 6, {INFRARED_CHARACTER_OMNI, BUTTONS_SENSOR, DISTANCE, ANGLE}, 4},
    {GROUP_3, 10, {CHARGING_STATE, VOLTAGE, CURRENT, TEMPERATURE, BATTERY_CHARGE, BATTERY_CAPACITY}, 6},
    {GROUP_4, 14, {WALL_SIGNAL, CLIFF_LEFT_SIGNAL, CLIFF_FRONT_LEFT_SIGNAL, CLIFF_FRONT_RIGHT_SIGNAL, CLIFF_RIGHT_SIGNAL, UNUSED_2, UNUSED_3, CHARGING_SOURCES_AVAILABLE}, 8},
    {GROUP_5, 12, {OI_MODE, SONG_NUMBER, SONG_PLAYING, NUMBER_OF_STREAM_PACKETS, REQUESTED_VELOCITY, REQUESTED_RADIUS, REQUESTED_RIGHT_VELOCITY, REQUESTED_LEFT_VELOCITY}, 8},
    {GROUP_6, 52, {BUMPS_AND_WHEEL_DROPS, WALL, CLIFF_LEFT, CLIFF_FRONT_LEFT, CLIFF_FRONT_RIGHT, CLIFF_RIGHT, VIRTUAL_WALL, WHEEL_OVERCURRENTS, DIRT_DETECT, UNUSED_1, INFRARED_CHARACTER_OMNI, BUTTONS_SENSOR, DISTANCE, ANGLE, CHARGING_STATE, VOLTAGE, CURRENT, TEMPERATURE, BATTERY_CHARGE, BATTERY_CAPACITY, WALL_SIGNAL, CLIFF_LEFT_SIGNAL, CLIFF_FRONT_LEFT_SIGNAL, CLIFF_FRONT_RIGHT_SIGNAL, CLIFF_RIGHT_SIGNAL, UNUSED_2, UNUSED_3, CHARGING_SOURCES_AVAILABLE, OI_MODE, SONG_NUMBER, SONG_PLAYING, NUMBER_OF_STREAM_PACKETS, REQUESTED_VELOCITY, REQUESTED_RADIUS, REQUESTED_RIGHT_VELOCITY, REQUESTED_LEFT_VELOCITY}, 36},
    {GROUP_100, 80, {BUMPS_AND_WHEEL_DROPS, WALL, CLIFF_LEFT, CLIFF_FRONT_LEFT, CLIFF_FRONT_RIGHT, CLIFF_RIGHT, VIRTUAL_WALL, WHEEL_OVERCURRENTS, DIRT_DETECT, UNUSED_1, INFRARED_CHARACTER_OMNI, BUTTONS_SENSOR, DISTANCE, ANGLE, CHARGING_STATE, VOLTAGE, CURRENT, TEMPERATURE, BATTERY_CHARGE, BATTERY_CAPACITY, WALL_SIGNAL, CLIFF_LEFT_SIGNAL, CLIFF_FRONT_LEFT_SIGNAL, CLIFF_FRONT_RIGHT_SIGNAL, CLIFF_RIGHT_SIGNAL, UNUSED_2, UNUSED_3, CHARGING_SOURCES_AVAILABLE, OI_MODE, SONG_NUMBER, SONG_PLAYING, NUMBER_OF_STREAM_PACKETS, REQUESTED_VELOCITY, REQUESTED_RADIUS, REQUESTED_RIGHT_VELOCITY, REQUESTED_LEFT_VELOCITY, LEFT_ENCODER_COUNTS, RIGHT_ENCODER_COUNTS, LIGHT_BUMPER, LIGHT_BUMP_LEFT_SIGNAL, LIGHT_BUMP_FRONT_LEFT_SIGNAL, LIGHT_BUMP_CENTER_LEFT_SIGNAL, LIGHT_BUMP_CENTER_RIGHT_SIGNAL, LIGHT_BUMP_FRONT_RIGHT_SIGNAL, LIGHT_BUMP_RIGHT_SIGNAL, INFRARED_CHARACTER_LEFT, INFRARED_CHARACTER_RIGHT, LEFT_MOTOR_CURRENT, RIGHT_MOTOR_CURRENT, MAIN_BRUSH_MOTOR_CURRENT, SIDE_BRUSH_MOTOR_CURRENT, STASIS}, 52},
    {GROUP_101, 28, {LEFT_ENCODER_COUNTS, RIGHT_ENCODER_COUNTS, LIGHT_BUMPER, LIGHT_BUMP_LEFT_SIGNAL, LIGHT_BUMP_FRONT_LEFT_SIGNAL, LIGHT_BUMP_CENTER_LEFT_SIGNAL, LIGHT_BUMP_CENTER_RIGHT_SIGNAL, LIGHT_BUMP_FRONT_RIGHT_SIGNAL, LIGHT_BUMP_RIGHT_SIGNAL, INFRARED_CHARACTER_LEFT, INFRARED_CHARACTER_RIGHT, LEFT_MOTOR_CURRENT, RIGHT_MOTOR_CURRENT, MAIN_BRUSH_MOTOR_CURRENT, SIDE_BRUSH_MOTOR_CURRENT, STASIS}, 16},
    {GROUP_106, 12, {LIGHT_BUMP_LEFT_SIGNAL, LIGHT_BUMP_FRONT_LEFT_SIGNAL, LIGHT_BUMP_CENTER_LEFT_SIGNAL, LIGHT_BUMP_CENTER_RIGHT_SIGNAL, LIGHT_BUMP_FRONT_RIGHT_SIGNAL, LIGHT_BUMP_RIGHT_SIGNAL}, 6},
    {GROUP_107, 9, {LEFT_MOTOR_CURRENT, RIGHT_MOTOR_CURRENT, MAIN_BRUSH_MOTOR_CURRENT, SIDE_BRUSH_MOTOR_CURRENT, STASIS}, 5}};

// Check if this is a group packet
bool isGroupPacket(SensorPacketId packetID)
{
    for (const auto &groupPacket : groupPackets)
    {
        if (groupPacket.id == packetID)
        {
            return true;
        }
    }
    return false;
}

#endif // SENSOR_PACKETS_H