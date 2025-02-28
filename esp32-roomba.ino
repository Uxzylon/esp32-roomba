#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "config.h"
#include "baudRateEnum.h"
#include "sensorPacketIdEnum.h"
#include "sensorPacketsDataBytes.h"
#include "commandOpcodeEnum.h"

HardwareSerial mySerial(1);

const int wakeupPin = 5;
const int baudRate = 115200;

WebSocketsServer webSocket = WebSocketsServer(81);

void setup()
{
    Serial.begin(baudRate);

    setupSerial(baudRate);

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

void setupSerial(int newBaudRate)
{
    mySerial.begin(newBaudRate, SERIAL_8N1, 16, 17); // RX, TX
}

void loop()
{
    webSocket.loop();
    readDataFromRoomba();
}

int parseCommandParameters(String command, byte* dataBytes, int maxParams)
{
    // commands are like "command param1 param2 param3 ..."
    // commands can also have no parameters (ex: start)
    // params are single bytes, however, for the drive commands, the first two params are int16_t (ex: drive 100 -200). Also these commands needs to be changed to have 4 parameters (low byte and high byte for each int16_t)
    // ex: 'start' -> dataBytes = {} and returns 0
    // ex: 'baud 11' -> dataBytes = {11} and returns 1
    // ex: 'drive 100 -200' -> dataBytes = {100, 0, 56, 255} and returns 4

    int paramCount = 0;
    int start = command.indexOf(' ') + 1;
    bool isDriveCommand = command.startsWith("drive");

    if (start == 0)
    {
        return paramCount;
    }

    while (start < command.length() && paramCount < maxParams)
    {
        int end = command.indexOf(' ', start);
        if (end == -1)
        {
            end = command.length();
        }
        String param = command.substring(start, end);
        int16_t value = param.toInt();

        if (isDriveCommand && paramCount < 4)
        {
            dataBytes[paramCount++] = highByte(value);
            dataBytes[paramCount++] = lowByte(value);
        }
        else
        {
            dataBytes[paramCount++] = value;
        }

        start = end + 1;
    }

    return paramCount;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    // on websocket connect
    if (type == WStype_CONNECTED)
    {
        Serial.println("Connected");
    }
    
    if (type == WStype_TEXT)
    {
        String command = String((char *)payload);
        String action = command.substring(0, command.indexOf(' '));

        const int maxParams = 16; // Adjust this based on the maximum number of parameters you expect
        byte dataBytes[maxParams];
        int paramCount = parseCommandParameters(command, dataBytes, maxParams);

        if (action == "wakeup")
        {
            wakeUp();
        }
        else if (action == "baud")
        {
            baud(static_cast<BaudRate>(dataBytes[0]));
        } 
        else if (action != "")
        {
            CommandOpcode opcode = getOpcodeByName(action);
            if (opcode != static_cast<CommandOpcode>(-1))
            {
                int expectedDataBytes = getDataBytesByOpcode(opcode);
                if (areDataBytesValid(expectedDataBytes, dataBytes, paramCount))
                {
                    runCommand(static_cast<byte>(opcode), dataBytes, paramCount);
                }
                else
                {
                    Serial.println("Invalid parameters");
                }
            } 
            else
            {
                Serial.println("Invalid command");
            }
        }
    }
}

bool areDataBytesValid(int expectedDataBytes, byte *dataBytes, int paramCount)
{
    if (expectedDataBytes == -1)
    {
        return false;
    }
    if (expectedDataBytes == -2)
    {
        return paramCount > 0 && paramCount == dataBytes[0] + 1;
    }
    if (expectedDataBytes == -3)
    {
        return areSongDataBytesValid(dataBytes, paramCount);
    }
    return paramCount == expectedDataBytes;
}

bool areSongDataBytesValid(byte *dataBytes, int paramCount)
{
    if (paramCount < 2)
    {
        return false;
    }
    int songLength = dataBytes[1];
    return paramCount == 2 * songLength + 2;
}

void wakeUp()
{
    pinMode(wakeupPin, OUTPUT);
    digitalWrite(wakeupPin, HIGH);
    delay(500);
    digitalWrite(wakeupPin, LOW);
}

void runCommand(byte command, byte *data, int dataLength)
{
    byte cmd[dataLength + 1];
    cmd[0] = command;
    for (int i = 0; i < dataLength; i++)
    {
        cmd[i + 1] = data[i];
    }
    mySerial.write(cmd, dataLength + 1);
}

void baud(BaudRate baudCode)
{
    byte cmd[2];
    cmd[0] = 129;

    byte baudRateCode = static_cast<byte>(baudCode);
    cmd[1] = baudRateCode;
    mySerial.write(cmd, 2);

    delay(200);
    mySerial.end();
    int baudRate = baudRates[baudRateCode];
    Serial.println("Changing baud rate to " + String(baudRate));
    setupSerial(baudRates[baudRateCode]);
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