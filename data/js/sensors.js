export const SensorPacketId = {
    GROUP_0: 0,
    GROUP_1: 1,
    GROUP_2: 2,
    GROUP_3: 3,
    GROUP_4: 4,
    GROUP_5: 5,
    GROUP_6: 6,
    BUMPS_AND_WHEEL_DROPS: 7,
    WALL: 8,
    CLIFF_LEFT: 9,
    CLIFF_FRONT_LEFT: 10,
    CLIFF_FRONT_RIGHT: 11,
    CLIFF_RIGHT: 12,
    VIRTUAL_WALL: 13,
    WHEEL_OVERCURRENTS: 14,
    DIRT_DETECT: 15,
    INFRARED_CHARACTER_OMNI: 17,
    BUTTONS_SENSOR: 18,
    DISTANCE: 19,
    ANGLE: 20,
    CHARGING_STATE: 21,
    VOLTAGE: 22,
    CURRENT: 23,
    TEMPERATURE: 24,
    BATTERY_CHARGE: 25,
    BATTERY_CAPACITY: 26,
    WALL_SIGNAL: 27,
    CLIFF_LEFT_SIGNAL: 28,
    CLIFF_FRONT_LEFT_SIGNAL: 29,
    CLIFF_FRONT_RIGHT_SIGNAL: 30,
    CLIFF_RIGHT_SIGNAL: 31,
    CHARGING_SOURCES_AVAILABLE: 34,
    OI_MODE: 35,
    SONG_NUMBER: 36,
    SONG_PLAYING: 37,
    NUMBER_OF_STREAM_PACKETS: 38,
    REQUESTED_VELOCITY: 39,
    REQUESTED_RADIUS: 40,
    REQUESTED_RIGHT_VELOCITY: 41,
    REQUESTED_LEFT_VELOCITY: 42,
    LEFT_ENCODER_COUNTS: 43,
    RIGHT_ENCODER_COUNTS: 44,
    LIGHT_BUMPER: 45,
    LIGHT_BUMP_LEFT_SIGNAL: 46,
    LIGHT_BUMP_FRONT_LEFT_SIGNAL: 47,
    LIGHT_BUMP_CENTER_LEFT_SIGNAL: 48,
    LIGHT_BUMP_CENTER_RIGHT_SIGNAL: 49,
    LIGHT_BUMP_FRONT_RIGHT_SIGNAL: 50,
    LIGHT_BUMP_RIGHT_SIGNAL: 51,
    INFRARED_CHARACTER_LEFT: 52,
    INFRARED_CHARACTER_RIGHT: 53,
    LEFT_MOTOR_CURRENT: 54,
    RIGHT_MOTOR_CURRENT: 55,
    MAIN_BRUSH_MOTOR_CURRENT: 56,
    SIDE_BRUSH_MOTOR_CURRENT: 57,
    STASIS: 58,
    GROUP_100: 100,
    GROUP_101: 101,
    GROUP_106: 106,
    GROUP_107: 107
};

export const SensorConfig = {
    bumps_wheeldrops: {
        type: "bitflags",
        values: {
            0x01: "Bump Right",       // Bit 0
            0x02: "Bump Left",        // Bit 1
            0x04: "Wheel Drop Right", // Bit 2
            0x08: "Wheel Drop Left"   // Bit 3
        }
    },
    wall: { type: "boolean" },
    cliff_left: { type: "boolean" },
    cliff_front_left: { type: "boolean" },
    cliff_front_right: { type: "boolean" },
    cliff_right: { type: "boolean" },
    virtual_wall: { type: "boolean" },
    wheel_overcurrents: {
        type: "bitflags",
        values: {
            0x01: "Side brush",    // Bit 0
            0x04: "Main brush",    // Bit 2
            0x08: "Right wheel",   // Bit 3
            0x10: "Left wheel"     // Bit 4
        }
    },
    infrared_omni: { type: "ir_character" },
    infrared_character_left: { type: "ir_character" },
    infrared_character_right: { type: "ir_character" },
    buttons: {
        type: "bitflags",
        values: {
            0x01: "Clean",
            0x02: "Spot", 
            0x04: "Dock",
            0x08: "Minute",
            0x10: "Hour",
            0x20: "Day",
            0x40: "Schedule",
            0x80: "Clock"
        }
    },
    distance: { type: "unit", unit: "mm" },
    angle: { type: "unit", unit: "degrees" },
    charging_state: {
        type: "enum",
        values: {
            0: "Not charging",
            1: "Reconditioning Charging",
            2: "Full Charging", 
            3: "Trickle Charging",
            4: "Waiting",
            5: "Charging Fault Condition"
        }
    },
    voltage: { type: "unit", unit: "mV" },
    current: { type: "unit", unit: "mA" },
    temperature: { type: "unit", unit: "°C" },
    battery_charge: { type: "unit", unit: "mAh" },
    battery_capacity: { type: "unit", unit: "mAh" },
    charging_sources_available: {
        type: "bitflags",
        values: {
            0x01: "Internal charger",
            0x02: "Home base"
        }
    },
    oi_mode: {
        type: "enum",
        values: {
            0: "Off",
            1: "Passive",
            2: "Safe",
            3: "Full"
        }
    },
    song_playing: { type: "boolean" },
    requested_velocity: { type: "unit", unit: "mm/s" },
    requested_radius: { type: "unit", unit: "mm" },
    requested_right_velocity: { type: "unit", unit: "mm/s" },
    requested_left_velocity: { type: "unit", unit: "mm/s" },
    light_bumper: {
        type: "bitflags",
        values: {
            0x01: "Left",
            0x02: "Front Left",
            0x04: "Center Left",
            0x08: "Center Right",
            0x10: "Front Right",
            0x20: "Right"
        }
    },
    left_motor_current: { type: "unit", unit: "mA" },
    right_motor_current: { type: "unit", unit: "mA" },
    main_brush_motor_current: { type: "unit", unit: "mA" },
    side_brush_motor_current: { type: "unit", unit: "mA" },
    stasis: { type: "boolean" },
};

export function formatSensorValue(sensorName, value) {
    if (value === undefined || value === null) {
        return "N/A";
    }
    
    const config = SensorConfig[sensorName];
    if (!config) {
        return value;
    }
    
    switch (config.type) {
        case "enum":
            return `${config.values[value] || value}`;

        case "bitflags":
            if (value === 0) return "None";
            
            const flags = [];
            for (const [bit, description] of Object.entries(config.values)) {
                if ((value & parseInt(bit)) !== 0) {
                    flags.push(description);
                }
            }
            return `${flags.join(", ")}`;
            
        case "boolean":
            return value === 1 ? true : false;

        case "unit":
            return `${value} ${config.unit}`;
            
        case "ir_character":
            const irMapping = ir_character_values[value];

            // Special handling for Auto-on Virtual Wall format (0LLLL0BB)
            if (!irMapping && ((value & 0b10001100) === 0)) {
                const id = (value >> 2) & 0b1111;  // Extract LLLL bits
                const beam = value & 0b11;         // Extract BB bits
                
                let idText = (id >= 1 && id <= 10) ? `ID ${id}` : 
                             (id === 11) ? "Unbound" : "Reserved";
                             
                let beamText = ["Fence", "Force Field", "Green Buoy", "Red Buoy"][beam];
                
                return `Auto-on Virtual Wall: ${idText}, ${beamText}`;
            }
            
            return `${irMapping || "Unknown"}`;
            
        default:
            return value;
    }
}

const ir_character_values = {
    0: "None",
    129: "Remote: Left",
    130: "Remote: Forward",
    131: "Remote: Right",
    132: "Remote: Spot",
    133: "Remote: Max",
    134: "Remote: Small",
    135: "Remote: Medium",
    136: "Remote: Large / Clean",
    137: "Remote: Stop",
    138: "Remote: Power",
    139: "Remote: Arc Left",
    140: "Remote: Arc Right",
    141: "Remote: Stop",
    142: "Remote: Download",
    143: "Remote: Seek Dock",
    240: "Discovery Charger: Reserved",
    242: "Discovery Charger: Force Field",
    244: "Discovery Charger: Green Buoy",
    246: "Discovery Charger: Green Buoy and Force Field",
    248: "Discovery Charger: Red Buoy",
    250: "Discovery Charger: Red Buoy and Force Field",
    252: "Discovery Charger: Red Buoy and Green Buoy",
    254: "Discovery Charger: Red Buoy, Green Buoy and Force Field",
    160: "600 Charger: Reserved",
    161: "600 Charger: Force Field",
    164: "600 Charger: Green Buoy",
    165: "600 Charger: Green Buoy and Force Field",
    168: "600 Charger: Red Buoy",
    169: "600 Charger: Red Buoy and Force Field",
    172: "600 Charger: Red Buoy and Green Buoy",
    173: "600 Charger: Red Buoy, Green Buoy and Force Field",
    162: "600 Virtual Wall",
}