#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <HardwareSerial.h>
#include "config.h"
#include "baudRateEnum.h"
#include "sensorPacketIdEnum.h"
#include "sensorPacketsDataBytes.h"
#include "commandOpcodeEnum.h"
#include <ArduinoOTA.h>
#include "esp32/spiram.h"
#include "SPIFFS.h"

#ifdef PORT
const int port = PORT;
#else
const int port = 80;
#endif

AsyncWebServer server(port);
AsyncWebSocket ws("/ws");

SemaphoreHandle_t fileTransferSemaphore = NULL;

#define PART_BOUNDARY "123456789000000000000987654321"
#define MIN(a, b) ((a) < (b) ? (a) : (b))

HardwareSerial mySerial(1);

const int baudRate = 115200;

bool streamPaused = false;
CommandOpcode sensorQueryListCommand = CommandOpcode::NONE;
const int maxRequestedSensorPackets = 1024;
SensorPacketId requestedSensorPackets[maxRequestedSensorPackets];

unsigned long songStartTime = 0;
unsigned long currentSongDuration = 0;

const int maxNotesPerSong = 40;
const int nbSongSlotsUsed = 2;
bool isRoombaPlayingSong = false;

struct Song
{
    byte songNumber;
    byte songLength;
    byte notes[maxNotesPerSong * 2];
    Song *next;
};

Song *songHead = nullptr;
Song *songTail = nullptr;

const int currentStreamSensorsSize = 59;
SensorPacketId currentStreamSensors[currentStreamSensorsSize];

unsigned long lastWebSocketSend = 0;
const unsigned long WS_SEND_INTERVAL = 100;
const int WS_MAX_QUEUE = 5;
String wsMessageQueue[WS_MAX_QUEUE];
int wsQueueHead = 0;
int wsQueueTail = 0;
bool wsQueueFull = false;

void setupSerial(int newBaudRate)
{
    mySerial.begin(newBaudRate, SERIAL_8N1, RX_PIN, TX_PIN);
}

void handleCameraStream(AsyncWebServerRequest *request) {
    static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
    static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
    static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    AsyncWebServerResponse *response = request->beginChunkedResponse(
        _STREAM_CONTENT_TYPE,
        [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
            static size_t currentIndex = 0;
            static size_t jpegSize = 0;
            static uint8_t *jpegBuf = NULL;
            static bool isNewFrame = true;
            static int64_t lastFrameTime = 0;

            if (isNewFrame) {
                if (jpegBuf) {
                    free(jpegBuf);
                    jpegBuf = NULL;
                }

                int64_t frameTime = 0;
                int32_t frame_delay = 0;
                
                #ifdef MIN_FRAME_TIME_MS
                    int minFrameTimeMs = MIN_FRAME_TIME_MS;
                #else
                    int minFrameTimeMs = 0;
                #endif
                
                if (lastFrameTime > 0) {
                    frameTime = esp_timer_get_time() - lastFrameTime;
                    frameTime /= 1000;
                    
                    frame_delay = (minFrameTimeMs > frameTime) ? minFrameTimeMs - frameTime : 0;
                    if (frame_delay > 0) {
                        delay(frame_delay);
                    }
                    
                    #ifdef DEBUG_CAMERA_STREAM
                    Serial.printf("MJPG: %uB %lldms%s%s [%.1ffps]\r\n",
                        (unsigned int)jpegSize,
                        (long long)frameTime,
                        (frame_delay > 0) ? (" (delay: " + String(frame_delay) + "ms)").c_str() : "",
                        (frameTime > 0) ? "" : "",
                        (frameTime > 0) ? (1000.0f / (float)frameTime) : 0.0f
                    );
                    #endif
                }
                
                lastFrameTime = esp_timer_get_time();

                camera_fb_t *fb = esp_camera_fb_get();
                if (!fb) {
                    Serial.println("Failed to get camera frame");
                    return 0;
                }

                jpegSize = fb->len;
                
                jpegBuf = (uint8_t*)malloc(jpegSize);
                if (!jpegBuf) {
                    Serial.println("Failed to allocate JPEG buffer memory");
                    esp_camera_fb_return(fb);
                    return 0;
                }
                
                memcpy(jpegBuf, fb->buf, jpegSize);
                esp_camera_fb_return(fb);

                currentIndex = 0;
                isNewFrame = false;

                size_t hlen = 0;
                hlen += snprintf((char *)buffer, maxLen, "%s", _STREAM_BOUNDARY);
                hlen += snprintf((char *)(buffer + hlen), maxLen - hlen, _STREAM_PART, jpegSize);
                
                if (hlen >= maxLen) {
                    free(jpegBuf);
                    jpegBuf = NULL;
                    isNewFrame = true;
                    return 0;
                }
                
                return hlen;
            }
            else {
                size_t remainingBytes = jpegSize - currentIndex;
                size_t bytesToSend = (remainingBytes > maxLen) ? maxLen : remainingBytes;
                
                if (bytesToSend > 0) {
                    memcpy(buffer, jpegBuf + currentIndex, bytesToSend);
                    currentIndex += bytesToSend;
                    return bytesToSend;
                }
                else {
                    free(jpegBuf);
                    jpegBuf = NULL;
                    isNewFrame = true;
                    return 0;
                }
            }
        }
    );
    
    response->addHeader("Access-Control-Allow-Origin", "*");
    request->send(response);
}

void webSocketBroadcast(const String &jsonData) {
    if (ws.count() == 0) {
        return;
    }
    
    // Add message to queue
    unsigned long currentTime = millis();
    if ((currentTime - lastWebSocketSend < WS_SEND_INTERVAL) || ws.getClients().empty()) {
        // If we're sending too fast or no clients are ready, add to queue
        if (!wsQueueFull) {
            wsMessageQueue[wsQueueTail] = jsonData;
            wsQueueTail = (wsQueueTail + 1) % WS_MAX_QUEUE;
            if (wsQueueTail == wsQueueHead) {
                wsQueueFull = true;
            }
        } else {
            // If queue is full, drop oldest message and add new one
            wsQueueHead = (wsQueueHead + 1) % WS_MAX_QUEUE;
            wsMessageQueue[wsQueueTail] = jsonData;
            wsQueueTail = (wsQueueTail + 1) % WS_MAX_QUEUE;
        }
        return;
    }
    
    // If we reach here, it's time to send a message
    lastWebSocketSend = currentTime;
    
    // Check if there are queued messages to send first
    String messageToSend;
    if (wsQueueHead != wsQueueTail || wsQueueFull) {
        messageToSend = wsMessageQueue[wsQueueHead];
        wsQueueHead = (wsQueueHead + 1) % WS_MAX_QUEUE;
        wsQueueFull = false;
    } else {
        messageToSend = jsonData;
    }
    
    // Create a temporary buffer with the JSON data
    char* buffer = new char[messageToSend.length() + 1];
    if (!buffer) {
        Serial.println("Error: Failed to allocate memory for WebSocket message");
        return;
    }
    
    strcpy(buffer, messageToSend.c_str());
    
    // Send to all clients
    ws.textAll(buffer, messageToSend.length());
    
    delete[] buffer;
}

void processWebSocketQueue() {
    // If there are no messages queued or it's too soon to send, return
    if ((wsQueueHead == wsQueueTail && !wsQueueFull) || 
        (millis() - lastWebSocketSend < WS_SEND_INTERVAL) ||
        ws.count() == 0) {
        return;
    }
    
    // Send the oldest queued message
    lastWebSocketSend = millis();
    String messageToSend = wsMessageQueue[wsQueueHead];
    wsQueueHead = (wsQueueHead + 1) % WS_MAX_QUEUE;
    wsQueueFull = false;
    
    // Create a temporary buffer with the JSON data
    char* buffer = new char[messageToSend.length() + 1];
    if (!buffer) {
        Serial.println("Error: Failed to allocate memory for WebSocket message");
        return;
    }
    
    strcpy(buffer, messageToSend.c_str());
    
    // Send to all clients
    ws.textAll(buffer, messageToSend.length());
    
    delete[] buffer;
}

int parseCommandParameters(String command, int *dataBytes, int maxParams)
{
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

bool areDataBytesValid(int expectedDataBytes, int *dataBytes, int paramCount)
{
    switch (expectedDataBytes)
    {
    case -1:
        return false;
    case -2:
        return paramCount > 0 && paramCount == dataBytes[0] + 1;
    case -3:
        if (paramCount < 2)
        {
            return false;
        }
        return paramCount == 2 * dataBytes[1] + 2;
    case -4:
        return paramCount > 0 && paramCount == dataBytes[0] * 2 + 1;
    default:
        return paramCount == expectedDataBytes;
    }
}

void clearSerialBuffer()
{
    while (mySerial.available())
    {
        mySerial.read();
    }
}

void storeRequestedSensorPackets(int *dataBytes, int paramCount, CommandOpcode opcode)
{
    sensorQueryListCommand = opcode;

    for (int i = 0; i < maxRequestedSensorPackets; i++)
    {
        requestedSensorPackets[i] = static_cast<SensorPacketId>(-1);
    }

    int index = 0;

    // Skip the first parameter for query_list command
    int startIndex = 0;
    if (paramCount > 1 && dataBytes[0] == paramCount - 1)
    {
        startIndex = 1;
    }

    for (int i = startIndex; i < paramCount; i++)
    {
        SensorPacketId packetID = static_cast<SensorPacketId>(dataBytes[i]);
        if (isGroupPacket(packetID))
        {
            // Find the group packet and add all its sensor packets to the requestedSensorPackets array
            for (const auto &groupPacket : groupPackets)
            {
                if (groupPacket.id == packetID)
                {
                    for (int j = 0; j < groupPacket.packetCount; j++)
                    {
                        requestedSensorPackets[index++] = groupPacket.packets[j];
                    }
                    break;
                }
            }
        }
        else
        {
            // Add the individual sensor packet to the requestedSensorPackets array
            requestedSensorPackets[index++] = packetID;
        }
    }
}

void runCommand(byte command, int *data, int dataLength)
{
    byte cmd[dataLength + 1];
    cmd[0] = command;
    for (int i = 0; i < dataLength; i++)
    {
        cmd[i + 1] = data[i];
    }
    mySerial.write(cmd, dataLength + 1);
}

void sendSong(Song *song)
{
    byte cmd[131];
    cmd[0] = 140;
    cmd[1] = song->songNumber;
    cmd[2] = song->songLength;
    for (int i = 0; i < song->songLength * 2; i++)
    {
        cmd[3 + i] = song->notes[i];
    }
    mySerial.write(cmd, 3 + song->songLength * 2);
}

void wakeUp()
{
    pinMode(WAKEUP_PIN, OUTPUT);
    digitalWrite(WAKEUP_PIN, HIGH);
    delay(500);
    digitalWrite(WAKEUP_PIN, LOW);
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
}

void stream(int *data, int dataLength)
{
    byte cmd[dataLength + 1];
    cmd[0] = 148;
    cmd[1] = dataLength;
    for (int i = 0; i < dataLength; i++)
    {
        cmd[i + 2] = data[i];
    }
    mySerial.write(cmd, dataLength + 2);
    pauseStream(false);
}

void handleStreamSongCommand(int *dataBytes, int paramCount)
{
    int totalNotes = dataBytes[0];
    int remainingNotes = totalNotes;
    int noteIndex = 1;

    while (remainingNotes > 0)
    {
        Song *song = new Song;
        song->songNumber = 255;
        song->songLength = min(remainingNotes, maxNotesPerSong);

        if (paramCount != 1 + totalNotes * 2)
        {
            Serial.println("Invalid STREAM_SONG command");
            Serial.println("songLength: " + String(song->songLength));
            Serial.println("paramCount: " + String(paramCount));
            delete song;
            return;
        }

        for (int i = 0; i < song->songLength * 2; i++)
        {
            song->notes[i] = dataBytes[noteIndex++];
        }
        song->next = nullptr;

        if (songTail)
        {
            songTail->next = song;
        }
        else
        {
            songHead = song;
        }
        songTail = song;

        remainingNotes -= song->songLength;
    }
}

void addToCurrentStream(SensorPacketId packetID)
{
    if (currentStreamSensors[packetID] != static_cast<SensorPacketId>(-1))
    {
        return;
    }
    currentStreamSensors[packetID] = packetID;
    int dataLength = 0;
    for (int i = 0; i < currentStreamSensorsSize; i++)
    {
        if (currentStreamSensors[i] != static_cast<SensorPacketId>(-1))
        {
            dataLength++;
        }
    }
    int data[dataLength];
    int index = 0;
    for (int i = 0; i < currentStreamSensorsSize; i++)
    {
        if (currentStreamSensors[i] != static_cast<SensorPacketId>(-1))
        {
            data[index++] = currentStreamSensors[i];
        }
    }
    // print data and dataLength
    Serial.println("Data length: " + String(dataLength));
    for (int i = 0; i < dataLength; i++)
    {
        Serial.println(data[i]);
    }
    stream(data, dataLength);
}

void playNextSong()
{
    Song *song = songHead;
    songHead = songHead->next;
    if (!songHead)
    {
        songTail = nullptr;
    }

    addToCurrentStream(SensorPacketId::SONG_PLAYING);

    // Upload first song to slot 0
    if (song->songNumber == 255)
    {
        song->songNumber = 0;
        sendSong(song);
        isRoombaPlayingSong = true;
    }
    byte currentSongNumber = song->songNumber;

    // Play the song
    byte playCmd[2] = {141, song->songNumber};
    mySerial.write(playCmd, 2);

    // Set the start time
    songStartTime = millis();

    // Compute the duration of the song
    currentSongDuration = 0;
    int totalSongDuration = 0;
    for (int i = 0; i < song->songLength; i++)
    {
        int noteIndex = i * 2;
        int durationIndex = noteIndex + 1;
        totalSongDuration += song->notes[durationIndex];
    }

    // Convert to milliseconds
    currentSongDuration = totalSongDuration * 1000 / 64;

    // Set the next song number and upload next song
    Song *nextSong = song->next;
    for (int i = 0; nextSong && i < nbSongSlotsUsed - 1; i++)
    {
        if (nextSong->songNumber == 255)
        {
            int newSongNumber = 0;
            if (currentSongNumber < nbSongSlotsUsed - 1)
            {
                newSongNumber = currentSongNumber + 1;
            }
            currentSongNumber = newSongNumber;
            nextSong->songNumber = currentSongNumber;
            sendSong(nextSong);
        }
        else
        {
            currentSongNumber = nextSong->songNumber;
        }
        nextSong = nextSong->next;
    }

    delete song;
}

String parseSensorData(byte *streamedData, int nBytes, CommandOpcode dataType)
{
    String jsonData = "{\"type\":\"";
    if (dataType == CommandOpcode::SENSORS)
    {
        jsonData += "sensors";
    }
    else if (dataType == CommandOpcode::QUERY_LIST)
    {
        jsonData += "query_list";
    }
    else if (dataType == CommandOpcode::STREAM)
    {
        jsonData += "stream";
    }
    else
    {
        jsonData += "unknown";
    }
    jsonData += "\",\"data\":{";

    // clear currentStreamSensors
    for (int i = 0; i < currentStreamSensorsSize; i++)
    {
        currentStreamSensors[i] = static_cast<SensorPacketId>(-1);
    }

    int dataIndex = 0;
    while (dataIndex < nBytes)
    {
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

        switch (packetID)
        {
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
            jsonData += "\"distance\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
            break;
        case ANGLE:
            jsonData += "\"angle\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
            break;
        case CHARGING_STATE:
            jsonData += "\"charging_state\":" + String(sensorData[0]) + ",";
            break;
        case VOLTAGE:
            jsonData += "\"voltage\":" + String((sensorData[0] << 8) | sensorData[1]) + ",";
            break;
        case CURRENT:
            jsonData += "\"current\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
            break;
        case TEMPERATURE:
            jsonData += "\"temperature\":" + String((int8_t)(sensorData[0])) + ",";
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
            isRoombaPlayingSong = sensorData[0] == 1;
            jsonData += "\"song_playing\":" + String(sensorData[0]) + ",";
            break;
        case NUMBER_OF_STREAM_PACKETS:
            jsonData += "\"number_of_stream_packets\":" + String(sensorData[0]) + ",";
            break;
        case REQUESTED_VELOCITY:
            jsonData += "\"requested_velocity\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
            break;
        case REQUESTED_RADIUS:
            jsonData += "\"requested_radius\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
            break;
        case REQUESTED_RIGHT_VELOCITY:
            jsonData += "\"requested_right_velocity\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
            break;
        case REQUESTED_LEFT_VELOCITY:
            jsonData += "\"requested_left_velocity\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
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
            jsonData += "\"left_motor_current\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
            break;
        case RIGHT_MOTOR_CURRENT:
            jsonData += "\"right_motor_current\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
            break;
        case MAIN_BRUSH_MOTOR_CURRENT:
            jsonData += "\"main_brush_motor_current\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
            break;
        case SIDE_BRUSH_MOTOR_CURRENT:
            jsonData += "\"side_brush_motor_current\":" + String((int16_t)((sensorData[0] << 8) | sensorData[1])) + ",";
            break;
        case STASIS:
            jsonData += "\"stasis\":" + String(sensorData[0]) + ",";
            break;
        default:
            break;
        }

        dataIndex += packetSize + 1;

        if (packetID < currentStreamSensorsSize)
        {
            currentStreamSensors[packetID] = packetID;
        }
    }

    // Remove the trailing comma and close the JSON string
    if (jsonData.endsWith(","))
    {
        jsonData = jsonData.substring(0, jsonData.length() - 1);
    }
    jsonData += "}}";

    return jsonData;
}

void readDataFromRoomba()
{
    if (sensorQueryListCommand != CommandOpcode::NONE)
    {
        if (mySerial.available())
        {
            Serial.println("Sensor or query_list command response");

            byte data[maxRequestedSensorPackets];
            int dataIndex = 0;

            // Read the data bytes corresponding to the requested sensor packets
            for (int i = 0; i < maxRequestedSensorPackets; i++)
            {
                if (requestedSensorPackets[i] != static_cast<SensorPacketId>(-1))
                {
                    int packetSize = getPacketSize(requestedSensorPackets[i]);
                    data[dataIndex] = requestedSensorPackets[i];
                    for (int j = 0; j < packetSize; j++)
                    {
                        if (mySerial.available())
                        {
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
            webSocketBroadcast(jsonData);

            sensorQueryListCommand = CommandOpcode::NONE;
            if (!streamPaused)
            {
                pauseStream(false);
                delay(500);
            }
        }
        return;
    }

    enum State
    {
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

    while (mySerial.available())
    {
        switch (state)
        {
        case WAIT_FOR_HEADER:
            header = mySerial.read();
            if (header == 19)
            {
                state = READ_NBYTES;
            }
            break;

        case READ_NBYTES:
            nBytes = mySerial.read();
            if (nBytes != 255 && nBytes != 0)
            {
                state = READ_DATA;
                dataIndex = 0;
            }
            else
            {
                state = WAIT_FOR_HEADER;
            }
            break;

        case READ_DATA:
            if (dataIndex < nBytes)
            {
                streamedData[dataIndex++] = mySerial.read();
            }
            else
            {
                state = READ_CHECKSUM;
            }
            break;

        case READ_CHECKSUM:
            checksum = mySerial.read();
            if (checksum != 255 && checksum != 0)
            {
                state = VALIDATE_CHECKSUM;
            }
            else
            {
                state = WAIT_FOR_HEADER;
            }
            break;

        case VALIDATE_CHECKSUM:
        {
            byte computedChecksum = header + nBytes - 1;
            for (int i = 0; i < nBytes; i++)
            {
                computedChecksum += streamedData[i];
            }
            computedChecksum = ~computedChecksum;

            if (checksum == computedChecksum)
            {
                state = PARSE_DATA;
            }
            else
            {
                state = WAIT_FOR_HEADER;
            }
        }
        break;

        case PARSE_DATA:
        {
            String jsonData = parseSensorData(streamedData, nBytes, CommandOpcode::STREAM);
            webSocketBroadcast(jsonData);
            state = WAIT_FOR_HEADER;
        }
        break;
        }
    }
}

bool initPSRAM() {
    if (!psramInit() || !psramFound()) {
        return false;
    }

    const size_t testSize = 14316;
    uint8_t* testMem = (uint8_t*)ps_malloc(testSize);
    if (!testMem) {
        return false;
    }

    for (size_t i = 0; i < testSize; i++) {
        testMem[i] = i & 0xFF;
    }

    bool testPassed = true;
    for (size_t i = 0; i < testSize; i++) {
        if (testMem[i] != (i & 0xFF)) {
            testPassed = false;
            break;
        }
    }

    free(testMem);

    return testPassed;
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                     AwsEventType type, void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            Serial.printf("WebSocket client #%u connected from %s\n", client->id(), 
                         client->remoteIP().toString().c_str());
            break;
            
        case WS_EVT_DISCONNECT:
            Serial.printf("WebSocket client #%u disconnected\n", client->id());
            break;
            
        case WS_EVT_DATA:
        {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                data[len] = 0; // Null terminate
                String command = String((char*)data);
                String action = command.substring(0, command.indexOf(' '));

                const int maxParams = 1024;
                int dataBytes[maxParams];
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
                            delay(500);
                            streamPaused = pause;
                        } else if (opcode == CommandOpcode::STREAM_SONG) {
                            handleStreamSongCommand(dataBytes, paramCount);
                        } else {
                            if (opcode == CommandOpcode::SENSORS || opcode == CommandOpcode::QUERY_LIST) {
                                pauseStream(true);
                                delay(500);
                                storeRequestedSensorPackets(dataBytes, paramCount, opcode);
                                clearSerialBuffer();
                            }
                            runCommand(static_cast<byte>(opcode), dataBytes, paramCount);
                        }
                    } else {
                        Serial.println("Invalid parameters");
                    }
                } else {
                    Serial.println("Invalid command");
                }
            }
            break;
        }
        
        case WS_EVT_ERROR:
            Serial.printf("WebSocket error for client #%u: %u\n", client->id(), *((uint16_t*)arg));
            break;
            
        default:
            break;
    }
}

void startServer() {
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    
    server.on("/stream", HTTP_GET, handleCameraStream);
    
    server.serveStatic("/", SPIFFS, "/")
        .setDefaultFile("index.html")
        .setCacheControl("max-age=3600");

    server.begin();
    Serial.printf("HTTP server started on port %d\n", port);
}

void setup()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector

    delay(1000);

    Serial.begin(baudRate);
    Serial.setDebugOutput(false);

    Serial.println("Starting...\n");

    Serial.printf("%s (%d cores %d MHz)\n", ESP.getChipModel(), ESP.getChipCores(), ESP.getCpuFreqMHz());
    Serial.printf("RAM : %d KB\n", ESP.getMinFreeHeap() / 1024);
    Serial.printf("Flash : %d KB\n", ESP.getFlashChipSize() / 1024);
    
    if (initPSRAM())
    {
        Serial.printf("PSRAM : %d KB\n", ESP.getPsramSize() / 1024);
    } else
    {
        Serial.println("PSRAM : Disabled");
    }

    setupSerial(baudRate);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = XCLK_FREQ_MHZ * 1000000;
    config.frame_size = FRAMESIZE_VGA;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.jpeg_quality = 15;
    config.fb_count = 1;

    if (psramFound())
    {
        config.fb_location = CAMERA_FB_IN_PSRAM;
        config.fb_count = 3;
        config.jpeg_quality = 12;
        config.grab_mode = CAMERA_GRAB_LATEST;
        config.frame_size = FRAMESIZE_VGA; // QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    }

    // Camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
    }

    sensor_t * s = esp_camera_sensor_get();
    if (s) {
        s->set_framesize(s, config.frame_size);
        s->set_quality(s, config.jpeg_quality);
        s->set_hmirror(s, 1); // Horizontal mirror
        s->set_vflip(s, 0); // Vertical flip
        s->set_colorbar(s, 0);
        s->set_special_effect(s, 0);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_wb_mode(s, 0); // Auto white balance
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_brightness(s, 0);
        s->set_lenc(s, 0); // Lens correction
    }

    if(!SPIFFS.begin(true)) {
        Serial.println("ERROR: SPIFFS mount failed");
        return;
    } else {
        Serial.println("SPIFFS mounted successfully");
    }

    // Wi-Fi connection
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());

    startServer();

    // OTA setup
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
        {
            type = "sketch";
        }
        else
        { // U_SPIFFS
            type = "filesystem";
        }
        SPIFFS.end();
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
        ESP.restart();
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
        {
            Serial.println("Auth Failed");
        }
        else if (error == OTA_BEGIN_ERROR)
        {
            Serial.println("Begin Failed");
        }
        else if (error == OTA_CONNECT_ERROR)
        {
            Serial.println("Connect Failed");
        }
        else if (error == OTA_RECEIVE_ERROR)
        {
            Serial.println("Receive Failed");
        }
        else if (error == OTA_END_ERROR)
        {
            Serial.println("End Failed");
        }
    });
    ArduinoOTA.setTimeout(60000);
    ArduinoOTA.begin();
}

void loop()
{
    readDataFromRoomba();

    if (songHead && millis() - songStartTime >= (currentSongDuration * 0.25) && !isRoombaPlayingSong)
    {
        playNextSong();
    }

    ArduinoOTA.handle();

    processWebSocketQueue();

    ws.cleanupClients();

    delay(1);
}
