#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "config.h"

HardwareSerial mySerial(1);

const int wakeupPin = 5;
const int baudRate = 115200;

WebSocketsServer webSocket = WebSocketsServer(81);

void setup() {
    // Initialize serial communication with ESP32 and Roomba
    Serial.begin(baudRate);
    mySerial.begin(baudRate, SERIAL_8N1, 16, 17); // RX, TX
    
    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
}

void loop() {
    webSocket.loop();

    static unsigned long lastRequestTime = 0;
    unsigned long currentTime = millis();

    // Request data from Roomba every second
    if (currentTime - lastRequestTime >= 1000) {
        requestDataFromRoomba();
        lastRequestTime = currentTime;
    }

    // Read data from Roomba
    readDataFromRoomba();
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        String command = String((char *)payload);
        if (command.startsWith("drive")) {
            int16_t leftWheelSpeed = command.substring(6, command.indexOf(' ', 6)).toInt();
            int16_t rightWheelSpeed = command.substring(command.indexOf(' ', 6) + 1).toInt();
            driveRoomba(leftWheelSpeed, rightWheelSpeed);
        } else if (command.startsWith("pwm_motors")) {
            int8_t mainBrushPWM = command.substring(10, 13).toInt();
            int8_t sideBrushPWM = command.substring(14, 17).toInt();
            uint8_t vacuumPWM = command.substring(18, 21).toInt();
            pwmMotors(mainBrushPWM, sideBrushPWM, vacuumPWM);
        } else if (command == "play_song") {
            playSong();
        } else if (command.startsWith("set_leds")) {
            int ledBits = command.substring(8, 9).toInt();
            int powerColor = command.substring(10, 11).toInt();
            int powerIntensity = command.substring(12, 13).toInt();
            setLEDs(ledBits, powerColor, powerIntensity);
        } else if (command == "shutdown") {
            powerDownRoomba();
        } else if (command == "wakeup") {
            wakeUpRoomba();
        } else if (command == "start") {
            startRoomba();
        } else if (command == "stop") {
            stopRoomba();
        } else if (command.startsWith("change_mode")) {
            int mode = command.substring(12, 13).toInt();
            changeRoombaMode(mode);
        }
    }
}

void wakeUpRoomba() {
    pinMode(wakeupPin, OUTPUT);
    digitalWrite(wakeupPin, HIGH);
    delay(500);
    digitalWrite(wakeupPin, LOW);
}

void changeRoombaMode(byte mode) {
    mySerial.write(130 + mode); // 131 = Safe mode, 132 = Full mode
}

void driveRoomba(int16_t leftWheelSpeed, int16_t rightWheelSpeed) {
    byte cmd[5];
    cmd[0] = 145; // Drive Direct command
    cmd[1] = highByte(rightWheelSpeed);
    cmd[2] = lowByte(rightWheelSpeed);
    cmd[3] = highByte(leftWheelSpeed);
    cmd[4] = lowByte(leftWheelSpeed);
    mySerial.write(cmd, 5);
}

void playSong() {
    // Define a song with notes and durations
    byte song[] = {
        140, 0, 4, // Song number 0, 4 notes
        60, 16,    // Note C4, duration 16/64s
        62, 16,    // Note D4, duration 16/64s
        64, 16,    // Note E4, duration 16/64s
        65, 16     // Note F4, duration 16/64s
    };
    mySerial.write(song, sizeof(song));

    // Play the song
    byte playCmd[] = {141, 0}; // Play song number 0
    mySerial.write(playCmd, sizeof(playCmd));
}

void powerDownRoomba() {
    mySerial.write(133); // Power Off
}

void pwmMotors(int8_t mainBrushPWM, int8_t sideBrushPWM, uint8_t vacuumPWM) {
    byte cmd[4];
    cmd[0] = 144; // PWM Motors command
    cmd[1] = mainBrushPWM;
    cmd[2] = sideBrushPWM;
    cmd[3] = vacuumPWM;
    mySerial.write(cmd, 4);
}

void setLEDs(byte ledBits, byte powerColor, byte powerIntensity) {
    byte cmd[4];
    cmd[0] = 139; // LEDs command
    cmd[1] = ledBits;
    cmd[2] = powerColor;
    cmd[3] = powerIntensity;
    mySerial.write(cmd, 4);
}

void requestDataFromRoomba() {
    // Request sensor data from Roomba
    byte requestCmd[] = {142, 100}; // Request sensor data group packet id 100
    mySerial.write(requestCmd, sizeof(requestCmd));
}

void readDataFromRoomba() {
    if (mySerial.available() > 0) {
        // Read sensor data from Roomba
        byte sensorData[80]; // group packet 100 has size 80 bytes
        mySerial.readBytes(sensorData, sizeof(sensorData));

        // Convert sensor data to a JSON string
        String jsonData = "{";
        jsonData += "\"bumps_wheeldrops\":" + String(sensorData[0]) + ",";
        jsonData += "\"wall\":" + String(sensorData[1]) + ",";
        jsonData += "\"cliff_left\":" + String(sensorData[2]) + ",";
        jsonData += "\"cliff_front_left\":" + String(sensorData[3]) + ",";
        jsonData += "\"cliff_front_right\":" + String(sensorData[4]) + ",";
        jsonData += "\"cliff_right\":" + String(sensorData[5]) + ",";
        jsonData += "\"virtual_wall\":" + String(sensorData[6]) + ",";
        jsonData += "\"motor_overcurrents\":" + String(sensorData[7]) + ",";
        jsonData += "\"dirt_detect\":" + String(sensorData[8]) + ",";
        jsonData += "\"infrared_omni\":" + String(sensorData[10]) + ",";
        jsonData += "\"buttons\":" + String(sensorData[11]) + ",";
        jsonData += "\"distance\":" + String((sensorData[12] << 8) | sensorData[13]) + ",";
        jsonData += "\"angle\":" + String((sensorData[14] << 8) | sensorData[15]) + ",";
        jsonData += "\"charging_state\":" + String(sensorData[16]) + ",";
        jsonData += "\"voltage\":" + String((sensorData[17] << 8) | sensorData[18]) + ",";
        jsonData += "\"current\":" + String((sensorData[19] << 8) | sensorData[20]) + ",";
        jsonData += "\"temperature\":" + String(sensorData[21]) + ",";
        jsonData += "\"battery_charge\":" + String((sensorData[22] << 8) | sensorData[23]) + ",";
        jsonData += "\"battery_capacity\":" + String((sensorData[24] << 8) | sensorData[25]) + ",";
        jsonData += "\"wall_signal\":" + String((sensorData[26] << 8) | sensorData[27]) + ",";
        jsonData += "\"cliff_left_signal\":" + String((sensorData[28] << 8) | sensorData[29]) + ",";
        jsonData += "\"cliff_front_left_signal\":" + String((sensorData[30] << 8) | sensorData[31]) + ",";
        jsonData += "\"cliff_front_right_signal\":" + String((sensorData[32] << 8) | sensorData[33]) + ",";
        jsonData += "\"cliff_right_signal\":" + String((sensorData[34] << 8) | sensorData[35]) + ",";
        jsonData += "\"charging_sources_available\":" + String(sensorData[39]) + ",";
        jsonData += "\"oi_mode\":" + String(sensorData[40]) + ",";
        jsonData += "\"song_number\":" + String(sensorData[41]) + ",";
        jsonData += "\"song_playing\":" + String(sensorData[42]) + ",";
        jsonData += "\"number_of_stream_packets\":" + String(sensorData[43]) + ",";
        jsonData += "\"requested_velocity\":" + String((sensorData[44] << 8) | sensorData[45]) + ",";
        jsonData += "\"requested_radius\":" + String((sensorData[46] << 8) | sensorData[47]) + ",";
        jsonData += "\"requested_right_velocity\":" + String((sensorData[48] << 8) | sensorData[49]) + ",";
        jsonData += "\"requested_left_velocity\":" + String((sensorData[50] << 8) | sensorData[51]) + ",";
        jsonData += "\"left_encoder_counts\":" + String((sensorData[52] << 8) | sensorData[53]) + ",";
        jsonData += "\"right_encoder_counts\":" + String((sensorData[54] << 8) | sensorData[55]) + ",";
        jsonData += "\"light_bumper\":" + String(sensorData[56]) + ",";
        jsonData += "\"light_bump_left_signal\":" + String((sensorData[57] << 8) | sensorData[58]) + ",";
        jsonData += "\"light_bump_front_left_signal\":" + String((sensorData[59] << 8) | sensorData[60]) + ",";
        jsonData += "\"light_bump_center_left_signal\":" + String((sensorData[61] << 8) | sensorData[62]) + ",";
        jsonData += "\"light_bump_center_right_signal\":" + String((sensorData[63] << 8) | sensorData[64]) + ",";
        jsonData += "\"light_bump_front_right_signal\":" + String((sensorData[65] << 8) | sensorData[66]) + ",";
        jsonData += "\"light_bump_right_signal\":" + String((sensorData[67] << 8) | sensorData[68]) + ",";
        jsonData += "\"infrared_character_left\":" + String(sensorData[69]) + ",";
        jsonData += "\"infrared_character_right\":" + String(sensorData[70]) + ",";
        jsonData += "\"left_motor_current\":" + String((sensorData[71] << 8) | sensorData[72]) + ",";
        jsonData += "\"right_motor_current\":" + String((sensorData[73] << 8) | sensorData[74]) + ",";
        jsonData += "\"main_brush_motor_current\":" + String((sensorData[75] << 8) | sensorData[76]) + ",";
        jsonData += "\"side_brush_motor_current\":" + String((sensorData[77] << 8) | sensorData[78]) + ",";
        jsonData += "\"stasis\":" + String(sensorData[79]);
        jsonData += "}";

        // Send the JSON string to WebSocket clients
        webSocket.broadcastTXT(jsonData);

        // Send the JSON string to WebSocket clients
        webSocket.broadcastTXT(jsonData);
    }
}

void startRoomba() {
    mySerial.write(128); // Start
}

void stopRoomba() {
    mySerial.write(173); // Stop
}