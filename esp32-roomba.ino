#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "config.h"
#include "baudRateEnum.h"
#include "dayEnum.h"
#include "sensorPacketIdEnum.h"
#include "sensorPacketsDataBytes.h"

HardwareSerial mySerial(1);

const int wakeupPin = 5;
const int baudRate = 115200;

const SensorPacketId packetIDs[] = {
    BUMPS_AND_WHEEL_DROPS,
    DISTANCE,
    ANGLE,
    CHARGING_STATE,
    CURRENT,
    TEMPERATURE,
    BATTERY_CHARGE,
    BATTERY_CAPACITY,
    OI_MODE,
    LIGHT_BUMPER,
    LEFT_MOTOR_CURRENT,
    RIGHT_MOTOR_CURRENT,
    MAIN_BRUSH_MOTOR_CURRENT,
    SIDE_BRUSH_MOTOR_CURRENT
};
const int packetsSize = sizeof(packetIDs) / sizeof(packetIDs[0]);

WebSocketsServer webSocket = WebSocketsServer(81);

void setup()
{
    // Initialize serial communication with ESP32 and Roomba
    Serial.begin(baudRate);
    mySerial.begin(baudRate, SERIAL_8N1, 16, 17); // RX, TX

    // Increase the serial receive buffer size
    mySerial.setRxBufferSize(256);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

void loop()
{
    webSocket.loop();
    readDataFromRoomba();
}

int parseCommandParameters(const String &command, int *params, int maxParams)
{
    int paramCount = 0;
    int start = command.indexOf(' ') + 1;
    while (start > 0 && paramCount < maxParams)
    {
        int end = command.indexOf(' ', start);
        if (end == -1)
        {
            end = command.length();
        }
        String param = command.substring(start, end);
        params[paramCount++] = param.toInt();
        start = end + 1;
    }
    return paramCount;
}

int16_t clamp(int16_t value, int16_t min, int16_t max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    return value;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    // on websocket connect
    if (type == WStype_CONNECTED)
    {
        Serial.println("Connected");
        // Start streaming data from Roomba
        stream(packetsSize, (int *)packetIDs);
    }
    
    if (type == WStype_TEXT)
    {
        String command = String((char *)payload);
        String action = command.substring(0, command.indexOf(' '));

        const int maxParams = 16; // Adjust this based on the maximum number of parameters you expect
        int params[maxParams];
        int paramCount = parseCommandParameters(command, params, maxParams);

        if (action == "wakeup")
        {
            wakeUp();
        }
        else if (action == "start")
        {
            start();
        }
        else if (action == "reset")
        {
            reset();
        }
        else if (action == "stop")
        {
            stop();
        }
        else if (action == "baud")
        {
            baud(static_cast<BaudRate>(params[0]));
        }
        else if (action == "safe")
        {
            safe();
        }
        else if (action == "full")
        {
            full();
        }
        else if (action == "clean")
        {
            clean();
        }
        else if (action == "max")
        {
            max();
        }
        else if (action == "spot")
        {
            spot();
        }
        else if (action == "seek_dock")
        {
            seekDock();
        }
        else if (action == "power")
        {
            power();
        }
        else if (action == "schedule")
        {
            schedule(params[0], params[1], params[2], params[3], params[4], params[5], params[6], params[7], params[8], params[9], params[10], params[11], params[12], params[13], params[14]);
        }
        else if (action == "set_day_time")
        {
            setDayTime(static_cast<Day>(params[0]), params[1], params[2]);
        }
        else if (action == "drive")
        {
            drive(params[0], params[1]);
        }
        else if (action == "drive_direct")
        {
            driveDirect(params[0], params[1]);
        }
        else if (action == "drive_pwm")
        {
            drivePWM(params[0], params[1]);
        }
        else if (action == "motors")
        {
            motors(params[0]);
        }
        else if (action == "pwm_motors")
        {
            pwmMotors(params[0], params[1], params[2]);
        }
        else if (action == "leds")
        {
            leds(params[0], params[1], params[2]);
        }
        else if (action == "scheduling_leds")
        {
            schedulingLeds(params[0], params[1]);
        }
        else if (action == "digit_leds_raw")
        {
            digitLEDsRaw(params[0], params[1], params[2], params[3]);
        }
        else if (action == "buttons")
        {
            buttons(params[0]);
        }
        else if (action == "digit_leds_ascii")
        {
            digitLEDsASCII(params[0], params[1], params[2], params[3]);
        }
        else if (action == "song")
        {
            byte songData[paramCount - 2];
            for (int i = 0; i < (paramCount - 2); i++)
            {
                songData[i] = params[i + 2];
            }
            song(params[0], params[1], songData);
        }
        else if (action == "play")
        {
            play(params[0]);
        }
        else if (action == "sensors")
        {
            sensors(params[0]);
        }
        else if (action == "query_list")
        {
            byte packetIDs[paramCount - 1];
            for (int i = 0; i < (paramCount - 1); i++)
            {
                packetIDs[i] = params[i + 1];
            }
            queryList(params[0], packetIDs);
        }
        else if (action == "stream")
        {
            byte packetIDs[paramCount - 1];
            for (int i = 0; i < (paramCount - 1); i++)
            {
                packetIDs[i] = params[i + 1];
            }
            stream(params[0], (int *)packetIDs);
        }
        else if (action == "pause_resume_stream")
        {
            pauseResumeStream(params[0] == 0);
        }
    }
}

void wakeUp()
{
    pinMode(wakeupPin, OUTPUT);
    digitalWrite(wakeupPin, HIGH);
    delay(500);
    digitalWrite(wakeupPin, LOW);
    delay(500);
    stream(packetsSize, (int *)packetIDs);
}

// Commands

void start()
{
    mySerial.write(128);
}

void reset()
{
    mySerial.write(7);
}

void stop()
{
    mySerial.write(173);
}

void baud(BaudRate baudCode)
{
    mySerial.write(129);
    mySerial.write(static_cast<byte>(baudCode));
}

void safe()
{
    mySerial.write(131);
}

void full()
{
    mySerial.write(132);
}

void clean()
{
    mySerial.write(135);
}

void max()
{
    mySerial.write(136);
}

void spot()
{
    mySerial.write(134);
}

void seekDock()
{
    mySerial.write(143);
}

void power()
{
    mySerial.write(133); // Powers down the Roomba
}

void schedule(byte days, byte sunHour, byte sunMinute, byte monHour, byte monMinute, byte tueHour, byte tueMinute, byte wedHour, byte wedMinute, byte thuHour, byte thuMinute, byte friHour, byte friMinute, byte satHour, byte satMinute)
{
    byte cmd[16];
    cmd[0] = 167;
    cmd[1] = days;
    cmd[2] = sunHour;
    cmd[3] = sunMinute;
    cmd[4] = monHour;
    cmd[5] = monMinute;
    cmd[6] = tueHour;
    cmd[7] = tueMinute;
    cmd[8] = wedHour;
    cmd[9] = wedMinute;
    cmd[10] = thuHour;
    cmd[11] = thuMinute;
    cmd[12] = friHour;
    cmd[13] = friMinute;
    cmd[14] = satHour;
    cmd[15] = satMinute;
    mySerial.write(cmd, 16);
}

void setDayTime(Day day, byte hour, byte minute)
{
    byte cmd[3];
    cmd[0] = 168;
    cmd[1] = static_cast<byte>(day);
    cmd[2] = hour;
    cmd[3] = minute;
    mySerial.write(cmd, 4);
}

void drive(int16_t velocity, int16_t radius)
{
    velocity = clamp(velocity, -500, 500);
    if (radius != -1 && radius != 1)
    {
        radius = clamp(radius, -2000, 2000);
    }

    byte cmd[5];
    cmd[0] = 137;
    cmd[1] = highByte(velocity);
    cmd[2] = lowByte(velocity);
    cmd[3] = highByte(radius);
    cmd[4] = lowByte(radius);
    mySerial.write(cmd, 5);
}

void driveDirect(int16_t leftWheelSpeed, int16_t rightWheelSpeed)
{
    leftWheelSpeed = clamp(leftWheelSpeed, -500, 500);
    rightWheelSpeed = clamp(rightWheelSpeed, -500, 500);

    byte cmd[5];
    cmd[0] = 145;
    cmd[1] = highByte(rightWheelSpeed);
    cmd[2] = lowByte(rightWheelSpeed);
    cmd[3] = highByte(leftWheelSpeed);
    cmd[4] = lowByte(leftWheelSpeed);
    mySerial.write(cmd, 5);
}

void drivePWM(int16_t rightWheelPWM, int16_t leftWheelPWM)
{
    rightWheelPWM = clamp(rightWheelPWM, -255, 255);
    leftWheelPWM = clamp(leftWheelPWM, -255, 255);

    byte cmd[5];
    cmd[0] = 146;
    cmd[1] = highByte(rightWheelPWM);
    cmd[2] = lowByte(rightWheelPWM);
    cmd[3] = highByte(leftWheelPWM);
    cmd[4] = lowByte(leftWheelPWM);
    mySerial.write(cmd, 5);
}

void motors(int motorBits)
{
    byte cmd[2];
    cmd[0] = 138;
    cmd[1] = motorBits;
    mySerial.write(cmd, 2);
}

void pwmMotors(int8_t mainBrushPWM, int8_t sideBrushPWM, uint8_t vacuumPWM)
{
    mainBrushPWM = clamp(mainBrushPWM, -127, 127);
    sideBrushPWM = clamp(sideBrushPWM, -127, 127);
    vacuumPWM = clamp(vacuumPWM, 0, 127);

    byte cmd[4];
    cmd[0] = 144;
    cmd[1] = mainBrushPWM;
    cmd[2] = sideBrushPWM;
    cmd[3] = vacuumPWM;
    mySerial.write(cmd, 4);
}

void leds(byte ledBits, int powerColor, int powerIntensity)
{
    ledBits = clamp(ledBits, 0, 255);
    powerColor = clamp(powerColor, 0, 255);
    powerIntensity = clamp(powerIntensity, 0, 255);

    byte cmd[4];
    cmd[0] = 139;
    cmd[1] = ledBits;
    cmd[2] = powerColor;
    cmd[3] = powerIntensity;
    mySerial.write(cmd, 4);
}

void schedulingLeds(byte weekdayLedBits, byte schedulingLedBits)
{
    weekdayLedBits = clamp(weekdayLedBits, 0, 255);
    schedulingLedBits = clamp(schedulingLedBits, 0, 255);

    byte cmd[3];
    cmd[0] = 162;
    cmd[1] = weekdayLedBits;
    cmd[2] = schedulingLedBits;
    mySerial.write(cmd, 3);
}

void digitLEDsRaw(byte digit3, byte digit2, byte digit1, byte digit0)
{
    digit3 = clamp(digit3, 0, 255);
    digit2 = clamp(digit2, 0, 255);
    digit1 = clamp(digit1, 0, 255);
    digit0 = clamp(digit0, 0, 255);

    byte cmd[5];
    cmd[0] = 163;
    cmd[1] = digit3;
    cmd[2] = digit2;
    cmd[3] = digit1;
    cmd[4] = digit0;
    mySerial.write(cmd, 5);
}

void buttons(byte buttonBits)
{
    buttonBits = clamp(buttonBits, 0, 255);

    byte cmd[2];
    cmd[0] = 165;
    cmd[1] = buttonBits;
    mySerial.write(cmd, 2);
}

void digitLEDsASCII(byte digit3, byte digit2, byte digit1, byte digit0)
{
    digit3 = clamp(digit3, 32, 126);
    digit2 = clamp(digit2, 32, 126);
    digit1 = clamp(digit1, 32, 126);
    digit0 = clamp(digit0, 32, 126);

    byte cmd[5];
    cmd[0] = 164;
    cmd[1] = digit3;
    cmd[2] = digit2;
    cmd[3] = digit1;
    cmd[4] = digit0;
    mySerial.write(cmd, 5);
}

// Song Opcode: 140 Data Bytes: 2N+2,
// where N is the number of notes in the song
// This command lets you specify up to four songs to the OI that you can play at a later time. Each song is
// associated with a song number. The Play command uses the song number to identify your song selection.
// Each song can contain up to sixteen notes. Each note is associated with a note number that uses MIDI
// note definitions and a duration that is specified in fractions of a second. The number of data bytes varies,
// depending on the length of the song specified. A one note song is specified by four data bytes. For each
// additional note within a song, add two data bytes.
//  Serial sequence: [140] [Song Number] [Song Length] [Note Number 1] [Note Duration 1] [Note
// Number 2] [Note Duration 2], etc.
//  Available in modes: Passive, Safe, or Full
//  Changes mode to: No Change
//  Song Number (0 – 4)
// The song number associated with the specific song. If you send a second Song command, using the
// same song number, the old song is overwritten.
//  Song Length (1 – 16)
// The length of the song, according to the number of musical notes within the song.
//  Song data bytes 3, 5, 7, etc.: Note Number (31 – 127)
// The pitch of the musical note Roomba will play, according to the MIDI note numbering scheme. The
// lowest musical note that Roomba will play is Note #31. Roomba considers all musical notes outside
// the range of 31 – 127 as rest notes, and will make no sound during the duration of those notes.
//  Song data bytes 4, 6, 8, etc.: Note Duration (0 – 255)
// The duration of a musical note, in increments of 1/64th of a second. Example: a half-second long
// musical note has a duration value of 32.
// Number 31 = Note G, Frequency 49.0
// Number 32 = Note G#, Frequency 51.9
// Number 33 = Note A, Frequency 55.0
// ...
void song(byte songNumber, byte songLength, byte *songData)
{
    byte cmd[2 + 2 * songLength];
    cmd[0] = 140;
    cmd[1] = songNumber;
    cmd[2] = songLength;
    for (int i = 0; i < songLength; i++)
    {
        cmd[3 + 2 * i] = songData[2 * i];
        cmd[4 + 2 * i] = songData[2 * i + 1];
    }
    mySerial.write(cmd, 2 + 2 * songLength);
}

void play(byte songNumber)
{
    songNumber = clamp(songNumber, 0, 4);
    byte cmd[2];
    cmd[0] = 141;
    cmd[1] = songNumber;
    mySerial.write(cmd, 2);
}

void sensors(byte packetID)
{
    byte cmd[2];
    cmd[0] = 142;
    cmd[1] = packetID;
    mySerial.write(cmd, 2);
}

void queryList(byte numberOfPackets, byte *packetIDs)
{
    byte cmd[2 + numberOfPackets];
    cmd[0] = 149;
    cmd[1] = numberOfPackets;
    for (int i = 0; i < numberOfPackets; i++)
    {
        cmd[2 + i] = packetIDs[i];
    }
    mySerial.write(cmd, 2 + numberOfPackets);
}

void stream(byte numberOfPackets, const int *packetIDs)
{
    byte cmd[2 + numberOfPackets];
    cmd[0] = 148;
    cmd[1] = numberOfPackets;
    for (int i = 0; i < numberOfPackets; i++)
    {
        cmd[2 + i] = packetIDs[i];
    }
    mySerial.write(cmd, 2 + numberOfPackets);
}

void pauseResumeStream(bool pause)
{
    byte cmd[2];
    cmd[0] = 150;
    cmd[1] = pause ? 0 : 1;
    mySerial.write(cmd, 2);
}

void readDataFromRoomba() {
    enum State {
        WAIT_FOR_HEADER,
        READ_NBYTES,
        READ_DATA,
        READ_CHECKSUM,
        VALIDATE_CHECKSUM,
        PARSE_DATA
    };

    static State state = WAIT_FOR_HEADER;
    static byte header;
    static byte nBytes;
    static byte checksum;
    static byte streamedData[256];
    static int dataIndex = 0;

    while (mySerial.available()) {
        switch (state) {
            case WAIT_FOR_HEADER:
                header = mySerial.read();
                if (header == 19) {
                    state = READ_NBYTES;
                }
                break;

            case READ_NBYTES:
                nBytes = mySerial.read();
                if (nBytes != 255 && nBytes != 0) {
                    state = READ_DATA;
                    dataIndex = 0;
                } else {
                    state = WAIT_FOR_HEADER;
                }
                break;

            case READ_DATA:
                if (dataIndex < nBytes) {
                    streamedData[dataIndex++] = mySerial.read();
                } else {
                    state = READ_CHECKSUM;
                }
                break;

            case READ_CHECKSUM:
                checksum = mySerial.read();
                if (checksum != 255 && checksum != 0) {
                    state = VALIDATE_CHECKSUM;
                } else {
                    state = WAIT_FOR_HEADER;
                }
                break;

            case VALIDATE_CHECKSUM:
                {
                    byte computedChecksum = header + nBytes - 1;
                    for (int i = 0; i < nBytes; i++) {
                        computedChecksum += streamedData[i];
                    }
                    computedChecksum = ~computedChecksum;

                    if (checksum == computedChecksum) {
                        state = PARSE_DATA;
                    } else {
                        state = WAIT_FOR_HEADER;
                    }
                }
                break;

            case PARSE_DATA:
                {
                    String jsonData = parseSensorData(streamedData, nBytes);
                    webSocket.broadcastTXT(jsonData);
                    state = WAIT_FOR_HEADER;
                }
                break;
        }
    }
}

String parseSensorData(byte *streamedData, int nBytes) {
    String jsonData = "{";
    int dataIndex = 0;

    while (dataIndex < nBytes) {
        SensorPacketId packetID = static_cast<SensorPacketId>(streamedData[dataIndex]);
        int packetSize = getPacketSize(packetID);
        byte *sensorData = &streamedData[dataIndex + 1];

        switch (packetID) {
            case BUMPS_AND_WHEEL_DROPS:
                jsonData += "\"bumps_wheeldrops\":" + String(sensorData[0]) + ",";
                break;
            case WALL:
                jsonData += "\"wall\":" + String(sensorData[0]) + ",";
                break;
            case CLIFF_LEFT:
                jsonData += "\"cliff_left\":" + String(sensorData[0]) + ",";
                break;
            case CLIFF_FRONT_LEFT:
                jsonData += "\"cliff_front_left\":" + String(sensorData[0]) + ",";
                break;
            case CLIFF_FRONT_RIGHT:
                jsonData += "\"cliff_front_right\":" + String(sensorData[0]) + ",";
                break;
            case CLIFF_RIGHT:
                jsonData += "\"cliff_right\":" + String(sensorData[0]) + ",";
                break;
            case VIRTUAL_WALL:
                jsonData += "\"virtual_wall\":" + String(sensorData[0]) + ",";
                break;
            case WHEEL_OVERCURRENTS:
                jsonData += "\"motor_overcurrents\":" + String(sensorData[0]) + ",";
                break;
            case DIRT_DETECT:
                jsonData += "\"dirt_detect\":" + String(sensorData[0]) + ",";
                break;
            case INFRARED_CHARACTER_OMNI:
                jsonData += "\"infrared_omni\":" + String(sensorData[0]) + ",";
                break;
            case BUTTONS:
                jsonData += "\"buttons\":" + String(sensorData[0]) + ",";
                break;
            case DISTANCE:
                jsonData += "\"distance\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case ANGLE:
                jsonData += "\"angle\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case CHARGING_STATE:
                jsonData += "\"charging_state\":" + String(sensorData[0]) + ",";
                break;
            case VOLTAGE:
                jsonData += "\"voltage\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case CURRENT:
                jsonData += "\"current\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case TEMPERATURE:
                jsonData += "\"temperature\":" + String(sensorData[0]) + ",";
                break;
            case BATTERY_CHARGE:
                jsonData += "\"battery_charge\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case BATTERY_CAPACITY:
                jsonData += "\"battery_capacity\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case WALL_SIGNAL:
                jsonData += "\"wall_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case CLIFF_LEFT_SIGNAL:
                jsonData += "\"cliff_left_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case CLIFF_FRONT_LEFT_SIGNAL:
                jsonData += "\"cliff_front_left_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case CLIFF_FRONT_RIGHT_SIGNAL:
                jsonData += "\"cliff_front_right_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case CLIFF_RIGHT_SIGNAL:
                jsonData += "\"cliff_right_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case CHARGING_SOURCES_AVAILABLE:
                jsonData += "\"charging_sources_available\":" + String(sensorData[0]) + ",";
                break;
            case OI_MODE:
                jsonData += "\"oi_mode\":" + String(sensorData[0]) + ",";
                break;
            case SONG_NUMBER:
                jsonData += "\"song_number\":" + String(sensorData[0]) + ",";
                break;
            case SONG_PLAYING:
                jsonData += "\"song_playing\":" + String(sensorData[0]) + ",";
                break;
            case NUMBER_OF_STREAM_PACKETS:
                jsonData += "\"number_of_stream_packets\":" + String(sensorData[0]) + ",";
                break;
            case REQUESTED_VELOCITY:
                jsonData += "\"requested_velocity\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case REQUESTED_RADIUS:
                jsonData += "\"requested_radius\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case REQUESTED_RIGHT_VELOCITY:
                jsonData += "\"requested_right_velocity\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case REQUESTED_LEFT_VELOCITY:
                jsonData += "\"requested_left_velocity\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case LEFT_ENCODER_COUNTS:
                jsonData += "\"left_encoder_counts\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case RIGHT_ENCODER_COUNTS:
                jsonData += "\"right_encoder_counts\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case LIGHT_BUMPER:
                jsonData += "\"light_bumper\":" + String(sensorData[0]) + ",";
                break;
            case LIGHT_BUMP_LEFT_SIGNAL:
                jsonData += "\"light_bump_left_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case LIGHT_BUMP_FRONT_LEFT_SIGNAL:
                jsonData += "\"light_bump_front_left_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case LIGHT_BUMP_CENTER_LEFT_SIGNAL:
                jsonData += "\"light_bump_center_left_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case LIGHT_BUMP_CENTER_RIGHT_SIGNAL:
                jsonData += "\"light_bump_center_right_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case LIGHT_BUMP_FRONT_RIGHT_SIGNAL:
                jsonData += "\"light_bump_front_right_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case LIGHT_BUMP_RIGHT_SIGNAL:
                jsonData += "\"light_bump_right_signal\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case INFRARED_CHARACTER_LEFT:
                jsonData += "\"infrared_character_left\":" + String(sensorData[0]) + ",";
                break;
            case INFRARED_CHARACTER_RIGHT:
                jsonData += "\"infrared_character_right\":" + String(sensorData[0]) + ",";
                break;
            case LEFT_MOTOR_CURRENT:
                jsonData += "\"left_motor_current\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case RIGHT_MOTOR_CURRENT:
                jsonData += "\"right_motor_current\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case MAIN_BRUSH_MOTOR_CURRENT:
                jsonData += "\"main_brush_motor_current\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case SIDE_BRUSH_MOTOR_CURRENT:
                jsonData += "\"side_brush_motor_current\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
                break;
            case STASIS:
                jsonData += "\"stasis\":" + String(sensorData[0]) + ",";
                break;
            default:
                break;
        }

        dataIndex += packetSize + 1;
    }

    // Remove the trailing comma and close the JSON string
    if (jsonData.endsWith(",")) {
        jsonData = jsonData.substring(0, jsonData.length() - 1);
    }
    jsonData += "}";

    return jsonData;
}