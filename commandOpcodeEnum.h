#ifndef COMMAND_OPCODE_H
#define COMMAND_OPCODE_H

#include <Arduino.h>

enum CommandOpcode {
    NONE = 0,
    RESET = 7,
    START = 128,
    BAUD = 129,
    SAFE = 131,
    FULL = 132,
    POWER = 133,
    SPOT = 134,
    CLEAN = 135,
    MAX = 136,
    DRIVE = 137,
    MOTORS = 138,
    LEDS = 139,
    SONG = 140,
    PLAY = 141,
    SENSORS = 142,
    SEEK_DOCK = 143,
    PWM_MOTORS = 144,
    DRIVE_DIRECT = 145,
    DRIVE_PWM = 146,
    STREAM = 148,
    QUERY_LIST = 149,
    PAUSE_RESUME_STREAM = 150,
    SCHEDULING_LEDS = 162,
    DIGIT_LEDS_RAW = 163,
    DIGIT_LEDS_ASCII = 164,
    BUTTONS = 165,
    SCHEDULE = 167,
    SET_DAY_TIME = 168,
    STOP = 173,
    WAKEUP = 200 // Custom command
};

struct CommandMapping {
    const char* name;
    CommandOpcode opcode;
    int dataBytes; // Number of data bytes expected for the command
};

const CommandMapping commandMap[] = {
    {"reset", RESET, 0},
    {"start", START, 0},
    {"baud", BAUD, 1},
    {"safe", SAFE, 0},
    {"full", FULL, 0},
    {"power", POWER, 0},
    {"spot", SPOT, 0},
    {"clean", CLEAN, 0},
    {"max", MAX, 0},
    {"drive", DRIVE, 4},
    {"motors", MOTORS, 1},
    {"leds", LEDS, 3},
    {"song", SONG, -3}, // 2N+2 bytes
    {"play", PLAY, 1},
    {"sensors", SENSORS, 1},
    {"seek_dock", SEEK_DOCK, 0},
    {"pwm_motors", PWM_MOTORS, 3},
    {"drive_direct", DRIVE_DIRECT, 4},
    {"drive_pwm", DRIVE_PWM, 4},
    {"stream", STREAM, -2}, // N+1 bytes
    {"query_list", QUERY_LIST, -2}, // N+1 bytes
    {"pause_resume_stream", PAUSE_RESUME_STREAM, 1},
    {"scheduling_leds", SCHEDULING_LEDS, 2},
    {"digit_leds_raw", DIGIT_LEDS_RAW, 4},
    {"digit_leds_ascii", DIGIT_LEDS_ASCII, 4},
    {"buttons", BUTTONS, 1},
    {"schedule", SCHEDULE, 15},
    {"set_day_time", SET_DAY_TIME, 3},
    {"stop", STOP, 0},
    {"wakeup", WAKEUP, 0}
};

CommandOpcode getOpcodeByName(const String& name) {
    for (const auto& mapping : commandMap) {
        if (name.equalsIgnoreCase(mapping.name)) {
            return mapping.opcode;
        }
    }
    return static_cast<CommandOpcode>(-1); // Return an invalid opcode if not found
}

int getDataBytesByOpcode(CommandOpcode opcode) {
    for (const auto& mapping : commandMap) {
        if (mapping.opcode == opcode) {
            return mapping.dataBytes;
        }
    }
    return -1; // Return -1 if opcode not found
}

#endif // COMMAND_OPCODE_H