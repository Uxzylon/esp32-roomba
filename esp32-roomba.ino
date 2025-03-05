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

bool streamPaused = false;
CommandOpcode sensorQueryListCommand = CommandOpcode::NONE;
const int maxRequestedSensorPackets = 1024;
SensorPacketId requestedSensorPackets[maxRequestedSensorPackets];

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

        const int maxParams = 33; // Adjust this based on the maximum number of parameters you expect
        byte dataBytes[maxParams];
        int paramCount = parseCommandParameters(command, dataBytes, maxParams);

        CommandOpcode opcode = getOpcodeByName(action);
            
        if (opcode != static_cast<CommandOpcode>(-1)) {
            int expectedDataBytes = getDataBytesByOpcode(opcode);
            if (areDataBytesValid(expectedDataBytes, dataBytes, paramCount)) {
                if (opcode == CommandOpcode::WAKEUP) {
                    wakeUp();
                } else if (opcode == CommandOpcode::BAUD) {
                    baud(static_cast<BaudRate>(dataBytes[0]));
                } else if (opcode == CommandOpcode::PAUSE_RESUME_STREAM) {
                    bool pause = dataBytes[0] == 0;
                    pauseStream(pause);
                    streamPaused = pause;
                } else {
                    if (opcode == CommandOpcode::SENSORS || opcode == CommandOpcode::QUERY_LIST) {
                        pauseStream(true);
                        storeRequestedSensorPackets(dataBytes, paramCount, opcode);
                        clearSerialBuffer();
                        // printRequestedSensorPackets();
                    }
                    runCommand(static_cast<byte>(opcode), dataBytes, paramCount);
                }
            }
            else {
                Serial.println("Invalid parameters");
            }
        } 
        else {
            Serial.println("Invalid command");
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

void clearSerialBuffer()
{
    while (mySerial.available())
    {
        mySerial.read();
    }
}

void storeRequestedSensorPackets(byte *dataBytes, int paramCount, CommandOpcode opcode) {
    sensorQueryListCommand = opcode;
    
    // sensors command only has one parameter, which is the id of the sensor packet
    // query_list command has multiple parameters, first is the number of packets, and the rest are the ids of the sensor packets
    // ex: 'sensors 7' -> requestedSensorPackets = {7}
    // ex: 'query_list 3 7 8 9' -> requestedSensorPackets = {7, 8, 9}
    // you also need to check if a packet id is a group packet, and if it is, you need to add all the packets in that group to the requestedSensorPackets array
    // ex: 'query_list 1 100' -> requestedSensorPackets = {BUMPS_AND_WHEEL_DROPS, WALL, CLIFF_LEFT, CLIFF_FRONT_LEFT, CLIFF_FRONT_RIGHT, CLIFF_RIGHT, VIRTUAL_WALL, ...
    for (int i = 0; i < maxRequestedSensorPackets; i++) {
        requestedSensorPackets[i] = static_cast<SensorPacketId>(-1);
    }

    int index = 0;

    // Skip the first parameter for query_list command
    int startIndex = 0;
    if (paramCount > 1 && dataBytes[0] == paramCount - 1) {
        startIndex = 1;
    }

    for (int i = startIndex; i < paramCount; i++) {
        SensorPacketId packetID = static_cast<SensorPacketId>(dataBytes[i]);
        if (isGroupPacket(packetID)) {
            // Find the group packet and add all its sensor packets to the requestedSensorPackets array
            for (const auto& groupPacket : groupPackets) {
                if (groupPacket.id == packetID) {
                    for (int j = 0; j < groupPacket.packetCount; j++) {
                        requestedSensorPackets[index++] = groupPacket.packets[j];
                    }
                    break;
                }
            }
        } else {
            // Add the individual sensor packet to the requestedSensorPackets array
            requestedSensorPackets[index++] = packetID;
        }
    }
}

void printRequestedSensorPackets()
{
    Serial.println("Requested sensor packets:");
    for (int i = 0; i < maxRequestedSensorPackets; i++) {
        if (requestedSensorPackets[i] != static_cast<SensorPacketId>(-1)) {
            Serial.println(requestedSensorPackets[i]);
        }
    }
    Serial.println();
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

void pauseStream(bool pause)
{
    byte cmd[2];
    cmd[0] = 150;
    cmd[1] = pause ? 0 : 1;
    mySerial.write(cmd, 2);
    delay(500);
}

void readDataFromRoomba() {
    if (sensorQueryListCommand != CommandOpcode::NONE) {
        // At this point we expect a sensors or query_list command response
        // use requestedSensorPackets to know which sensor packets to expect
        // parse the data and send it to the websocket
        // There are no headers in the response, it's just the data bytes one after the other corresponding to the requested sensor packets
        // The data bytes are sent in the same order as the requested sensor packets
        
        // query_list 3 7 35 100
        
        
        if (mySerial.available()) {
            Serial.println("Sensor or query_list command response");

            byte data[maxRequestedSensorPackets];
            int dataIndex = 0;

            // Read the data bytes corresponding to the requested sensor packets
            for (int i = 0; i < maxRequestedSensorPackets; i++) {
                if (requestedSensorPackets[i] != static_cast<SensorPacketId>(-1)) {
                    int packetSize = getPacketSize(requestedSensorPackets[i]);
                    //Serial.println("Packet ID: " + String(requestedSensorPackets[i]) + ", Packet size: " + String(packetSize));
                    data[dataIndex] = requestedSensorPackets[i];
                    // data[dataIndex] = static_cast<byte>(requestedSensorPackets[i]);
                    for (int j = 0; j < packetSize; j++) {
                        if (mySerial.available()) {
                            data[dataIndex + 1] = mySerial.read();
                            dataIndex++;
                        }
                        delay(10);
                    }
                    dataIndex++;
                }
            }

            // Parse the data and send it to the websocket
            String jsonData = parseSensorData(data, dataIndex, sensorQueryListCommand);
            Serial.println(jsonData);
            webSocket.broadcastTXT(jsonData);

            sensorQueryListCommand = CommandOpcode::NONE;
            if (!streamPaused) {
                pauseStream(false);
            }
        }

        return;
    } 
    
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
    static byte streamedData[maxRequestedSensorPackets];
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
                    String jsonData = parseSensorData(streamedData, nBytes, CommandOpcode::STREAM);
                    webSocket.broadcastTXT(jsonData);
                    state = WAIT_FOR_HEADER;
                }
                break;
        }
    }
}

String parseSensorData(byte *streamedData, int nBytes, CommandOpcode dataType) {
    // {"type":"sensors","data":{"bumps_wheeldrops":0,"wall":0,"cliff_left":0,"cliff_front_left":0,"cliff_front_right":0,"cliff_right":0,"virtual_wall":0,"wheel_overcurrents":0,"dirt_detect":0,"infrared_omni":0,"buttons":0,"distance":0,"angle":0,"charging_state":0,"voltage":0,"current":0,"temperature":0,"battery_charge":0,"battery_capacity":0,"wall_signal":0,"cliff_left_signal":0,"cliff_front_left_signal":0,"cliff_front_right_signal":0,"cliff_right_signal":0,"charging_sources_available":0,"oi_mode":0,"song_number":0,"song_playing":0,"number_of_stream_packets":0,"requested_velocity":0,"requested_radius":0,"requested_right_velocity":0,"requested_left_velocity":0,"left_encoder_counts":0,"right_encoder_counts":0,"light_bumper":0,"light_bump_left_signal":0,"light_bump_front_left_signal":0,"light_bump_center_left_signal":0,"light_bump_center_right_signal":0,"light_bump_front_right_signal":0,"light_bump_right_signal":0,"infrared_character_left":0,"infrared_character_right":0,"left_motor_current":0,"right_motor_current":0,"main_brush_motor_current":0,"side_brush_motor_current":0,"stasis":0}}
    String jsonData = "{\"type\":\"";
    if (dataType == CommandOpcode::SENSORS) {
        jsonData += "sensors";
    } else if (dataType == CommandOpcode::QUERY_LIST) {
        jsonData += "query_list";
    } else if (dataType == CommandOpcode::STREAM) {
        jsonData += "stream";
    } else {
        jsonData += "unknown";
    }
    jsonData += "\",\"data\":{";

    // String jsonData = "{";
    int dataIndex = 0;

    // Serial.println("nBytes: " + String(nBytes));

    while (dataIndex < nBytes) {
        SensorPacketId packetID = static_cast<SensorPacketId>(streamedData[dataIndex]);
        int packetSize = getPacketSize(packetID);
        byte *sensorData = &streamedData[dataIndex + 1];

        // Print the sensor data
        // Serial.println("Packet ID: " + String(packetID) + ", Packet size: " + String(packetSize));
        // Serial.print("Data bytes: ");
        // for (int i = 0; i < packetSize; i++) {
        //     Serial.print(sensorData[i]);
        //     Serial.print(" ");
        // }
        // Serial.println();

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
                jsonData += "\"wheel_overcurrents\":" + String(sensorData[0]) + ",";
                break;
            case DIRT_DETECT:
                jsonData += "\"dirt_detect\":" + String(sensorData[0]) + ",";
                break;
            case INFRARED_CHARACTER_OMNI:
                jsonData += "\"infrared_omni\":" + String(sensorData[0]) + ",";
                break;
            case BUTTONS_SENSOR:
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
    jsonData += "}}";

    return jsonData;
}