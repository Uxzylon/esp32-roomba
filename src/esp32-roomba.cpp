#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"
#include <WebSocketsServer.h>
#include <HardwareSerial.h>
#include "config.h"
#include "baudRateEnum.h"
#include "sensorPackets.h"
#include "commandOpcodeEnum.h"
#include <ArduinoOTA.h>
#include "esp32/spiram.h"
#include "SPIFFS.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef PORT
const int port = PORT;
#else
const int port = 80;
#endif

#ifdef WEBSOCKET_PORT
const int websocketPort = WEBSOCKET_PORT;
#else
const int websocketPort = 81;
#endif

httpd_handle_t httpServer = NULL;
WebSocketsServer webSocket = WebSocketsServer(websocketPort);

HardwareSerial mySerial(1);

const int baudRate = 115200;

bool streamPaused = false;
CommandOpcode sensorQueryListCommand = CommandOpcode::NONE;
const int maxRequestedSensorPackets = 1024;
SensorPacketId requestedSensorPackets[maxRequestedSensorPackets];

#define PART_BOUNDARY "123456789000000000000987654321"

#define MAX_SENSOR_ID 100 // Maximum sensor ID we'll track
struct PreviousSensorValue
{
    bool initialized = false;
    int value = 0;
    bool included = false; // Whether this sensor was in the last packet
};
PreviousSensorValue previousSensorValues[MAX_SENSOR_ID];
unsigned long lastFullUpdateTime = 0;
const unsigned long FULL_UPDATE_INTERVAL_MS = 3000;

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

bool isStreamingOverWS = false;

void setupSerial(int newBaudRate)
{
    mySerial.begin(newBaudRate, SERIAL_8N1, RX_PIN, TX_PIN);
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
    // Determine if we should do a full update
    bool fullUpdate = (millis() - lastFullUpdateTime >= FULL_UPDATE_INTERVAL_MS);
    if (fullUpdate)
    {
        lastFullUpdateTime = millis();
    }

    // Mark all sensors as not included in this packet
    for (int i = 0; i < MAX_SENSOR_ID; i++)
    {
        if (previousSensorValues[i].initialized)
        {
            previousSensorValues[i].included = false;
        }
    }

    // JSON to hold changed values
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

    // Process each packet in the data stream
    bool hasChanges = false;
    int dataIndex = 0;
    while (dataIndex < nBytes)
    {
        SensorPacketId packetID = static_cast<SensorPacketId>(streamedData[dataIndex]);
        int packetSize = getPacketSize(packetID);
        byte *sensorData = &streamedData[dataIndex + 1];

        // Set the current sensor as included
        if (packetID < MAX_SENSOR_ID)
        {
            previousSensorValues[packetID].included = true;
        }

        // Print the sensor data
        // Serial.println("Packet ID: " + String(packetID) + ", Packet size: " + String(packetSize));
        // Serial.print("Data bytes: ");
        // for (int i = 0; i < packetSize; i++) {
        //     Serial.print(sensorData[i]);
        //     Serial.print(" ");
        // }
        // Serial.println();

        // Get sensor name and value using the helper functions
        String sensorName = getSensorName(packetID);
        int sensorValue = parseSensorValue(packetID, sensorData);

        // Special case for SONG_PLAYING
        if (packetID == SONG_PLAYING)
        {
            isRoombaPlayingSong = sensorData[0] == 1;
        }

        // Only include the sensor if it changed or this is a full update
        if (!sensorName.isEmpty())
        {
            if (packetID < MAX_SENSOR_ID)
            {
                if (fullUpdate ||
                    !previousSensorValues[packetID].initialized ||
                    previousSensorValues[packetID].value != sensorValue)
                {
                    // Value changed or first time seeing this sensor
                    jsonData += "\"" + String(sensorName) + "\":" + String(sensorValue) + ",";
                    hasChanges = true;

                    // Store the new value
                    previousSensorValues[packetID].value = sensorValue;
                    previousSensorValues[packetID].initialized = true;
                }
            }
        }

        dataIndex += packetSize + 1;

        if (packetID < currentStreamSensorsSize)
        {
            currentStreamSensors[packetID] = packetID;
        }
    }

    // Check for sensors that were in previous packets but not in this one
    // Send null for them to indicate they're no longer present
    for (int i = 0; i < MAX_SENSOR_ID; i++)
    {
        if (previousSensorValues[i].initialized && !previousSensorValues[i].included)
        {
            String sensorName = getSensorName(static_cast<SensorPacketId>(i));

            if (!sensorName.isEmpty())
            {
                jsonData += "\"" + String(sensorName) + "\":null,";
                hasChanges = true;
                previousSensorValues[i].initialized = false;
            }
        }
    }

    // If no changes and not a full update, return empty
    if (!hasChanges && !fullUpdate)
    {
        return "";
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
            webSocket.broadcastTXT(jsonData);

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
            // Only send if there's data to send
            if (jsonData.length() > 0)
            {
                webSocket.broadcastTXT(jsonData);
            }
            state = WAIT_FOR_HEADER;
        }
        break;
        }
    }
}

bool initPSRAM()
{
    if (!psramInit() || !psramFound())
    {
        return false;
    }

    const size_t testSize = 14316;
    uint8_t *testMem = (uint8_t *)ps_malloc(testSize);
    if (!testMem)
    {
        return false;
    }

    for (size_t i = 0; i < testSize; i++)
    {
        testMem[i] = i & 0xFF;
    }

    bool testPassed = true;
    for (size_t i = 0; i < testSize; i++)
    {
        if (testMem[i] != (i & 0xFF))
        {
            testPassed = false;
            break;
        }
    }

    free(testMem);

    return testPassed;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_CONNECTED:
    {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        isStreamingOverWS = true;
        break;
    }
    case WStype_DISCONNECTED:
    {
        Serial.printf("[%u] Disconnected!\n", num);
        if (webSocket.connectedClients() == 0)
        {
            isStreamingOverWS = false;
        }
        break;
    }
    case WStype_TEXT:
    {
        String command = String((char *)payload);
        String action = command.substring(0, command.indexOf(' '));

        const int maxParams = 1024;
        int dataBytes[maxParams];
        int paramCount = parseCommandParameters(command, dataBytes, maxParams);

        CommandOpcode opcode = getOpcodeByName(action);

        if (opcode != static_cast<CommandOpcode>(-1))
        {
            int expectedDataBytes = getDataBytesByOpcode(opcode);
            if (areDataBytesValid(expectedDataBytes, dataBytes, paramCount))
            {
                if (opcode == CommandOpcode::WAKEUP)
                {
                    wakeUp();
                }
                else if (opcode == CommandOpcode::BAUD)
                {
                    baud(static_cast<BaudRate>(dataBytes[0]));
                }
                else if (opcode == CommandOpcode::PAUSE_RESUME_STREAM)
                {
                    bool pause = dataBytes[0] == 0;
                    pauseStream(pause);
                    delay(500);
                    streamPaused = pause;
                }
                else if (opcode == CommandOpcode::STREAM_SONG)
                {
                    handleStreamSongCommand(dataBytes, paramCount);
                }
                else
                {
                    if (opcode == CommandOpcode::SENSORS || opcode == CommandOpcode::QUERY_LIST)
                    {
                        pauseStream(true);
                        delay(500);
                        storeRequestedSensorPackets(dataBytes, paramCount, opcode);
                        clearSerialBuffer();
                    }
                    runCommand(static_cast<byte>(opcode), dataBytes, paramCount);
                }
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
        break;
    }
    default:
    {
        break;
    }
    }
}

static esp_err_t handleHealthCheck(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_set_hdr(req, "X-ESP32", "true");
    httpd_resp_send(req, "OK", strlen("OK"));
    return ESP_OK;
}

static esp_err_t handleMjpegStream(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t *_jpg_buf = NULL;
    char *part_buf[64];

#ifdef MIN_FRAME_TIME_MS
    int minFrameTimeMs = MIN_FRAME_TIME_MS;
#else
    int minFrameTimeMs = 0;
#endif

    static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
    static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
    static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

    static int64_t last_frame = 0;
    if (!last_frame)
    {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        Serial.println("STREAM: failed to set HTTP response type");
        return res;
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (res == ESP_OK)
    {
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    }

    while (true)
    {
        fb = esp_camera_fb_get();
        if (!fb)
        {
            Serial.println("STREAM: failed to acquire frame");
            res = ESP_FAIL;
        }
        else
        {
            if (fb->format != PIXFORMAT_JPEG)
            {
                Serial.println("STREAM: Non-JPEG frame returned by camera module");
                res = ESP_FAIL;
            }
            else
            {
                _jpg_buf_len = fb->len;
                _jpg_buf = fb->buf;
            }
        }
        if (res == ESP_OK)
        {
            size_t hlen = snprintf((char *)part_buf, 64, STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if (res == ESP_OK)
        {
            res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        }
        if (fb)
        {
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        }
        else if (_jpg_buf)
        {
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if (res != ESP_OK)
        {
            break;
        }
        int64_t frame_time = esp_timer_get_time() - last_frame;
        frame_time /= 1000;
        int32_t frame_delay = (minFrameTimeMs > frame_time) ? minFrameTimeMs - frame_time : 0;
        delay(frame_delay);

#ifdef DEBUG_CAMERA_STREAM
        Serial.printf("MJPG: %uB %lldms%s%s [%.1ffps]\r\n",
                      _jpg_buf_len,
                      frame_time,
                      (frame_delay > 0) ? (" (delay: " + String(frame_delay) + "ms)").c_str() : "",
                      (frame_time > 0) ? "" : "",
                      (frame_time > 0) ? (1000.0f / (float)frame_time) : 0.0f);
#endif

        last_frame = esp_timer_get_time();
    }

    last_frame = 0;
    return res;
}

static esp_err_t handleSpiffs(httpd_req_t *req)
{
    char filepath[128];

    strncpy(filepath, req->uri, sizeof(filepath) - 1);
    filepath[sizeof(filepath) - 1] = '\0';
    if (strcmp(filepath, "/") == 0)
    {
        strcpy(filepath, "/index.html");
    }

    File file = SPIFFS.open(filepath, "r");

    if (strstr(filepath, ".html"))
    {
        httpd_resp_set_type(req, "text/html");
    }
    else if (strstr(filepath, ".css"))
    {
        httpd_resp_set_type(req, "text/css");
    }
    else if (strstr(filepath, ".js"))
    {
        httpd_resp_set_type(req, "application/javascript");
    }
    else if (strstr(filepath, ".ico"))
    {
        httpd_resp_set_type(req, "image/x-icon");
    }
    else if (strstr(filepath, ".jpg") || strstr(filepath, ".jpeg"))
    {
        httpd_resp_set_type(req, "image/jpeg");
    }
    else if (strstr(filepath, ".png"))
    {
        httpd_resp_set_type(req, "image/png");
    }
    else
    {
        httpd_resp_set_type(req, "text/plain");
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    // Send file in chunks
    const size_t CHUNK_SIZE = 1024;
    uint8_t buffer[CHUNK_SIZE];
    size_t bytesRead;

    while ((bytesRead = file.read(buffer, CHUNK_SIZE)) > 0)
    {
        if (httpd_resp_send_chunk(req, (const char *)buffer, bytesRead) != ESP_OK)
        {
            file.close();
            return ESP_FAIL;
        }
    }

    // Send empty chunk to signal HTTP response completion
    httpd_resp_send_chunk(req, NULL, 0);
    file.close();
    return ESP_OK;
}

void startServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;
    config.lru_purge_enable = true; // Enable LRU algorithm for socket purging
    config.uri_match_fn = httpd_uri_match_wildcard;

    // Start the httpd server
    if (httpd_start(&httpServer, &config) != ESP_OK)
    {
        Serial.println("Failed to start HTTP server");
        return;
    }

    httpd_uri_t stream_uri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = handleMjpegStream,
        .user_ctx = NULL};
    httpd_register_uri_handler(httpServer, &stream_uri);

    httpd_uri_t health_check = {
        .uri = "/healthcheck",
        .method = HTTP_GET,
        .handler = handleHealthCheck,
        .user_ctx = NULL};
    httpd_register_uri_handler(httpServer, &health_check);

    httpd_uri_t file_download = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = handleSpiffs,
        .user_ctx = NULL};
    httpd_register_uri_handler(httpServer, &file_download);

    Serial.println("HTTP server started on port " + String(port));

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started on port " + String(websocketPort));
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
    }
    else
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

    sensor_t *s = esp_camera_sensor_get();
    if (s)
    {
        s->set_framesize(s, config.frame_size);
        s->set_quality(s, config.jpeg_quality);
        s->set_hmirror(s, 1); // Horizontal mirror
        s->set_vflip(s, 0);   // Vertical flip
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

    if (!SPIFFS.begin(true))
    {
        Serial.println("ERROR: SPIFFS mount failed");
        return;
    }
    else
    {
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
    ArduinoOTA.onStart([]()
                       {
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
        Serial.println("Start updating " + type); });
    ArduinoOTA.onEnd([]()
                     {
        Serial.println("\nEnd");
        ESP.restart(); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error)
                       {
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
        } });
    ArduinoOTA.setTimeout(60000);
    ArduinoOTA.begin();
}

void loop()
{
    webSocket.loop();
    readDataFromRoomba();

    if (songHead && millis() - songStartTime >= (currentSongDuration * 0.25) && !isRoombaPlayingSong)
    {
        playNextSong();
    }

    ArduinoOTA.handle();
}
