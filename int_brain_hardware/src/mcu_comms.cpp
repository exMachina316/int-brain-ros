#include "int_brain_hardware/mcu_comms.hpp"

#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>

#include "int_brain_messages.h"

MCUComms::~MCUComms() {
    if (connected()) {
        disconnect();
    }
}

bool MCUComms::connect(const std::string& serial_device, int32_t timeout_ms) {
    timeout_ms_ = timeout_ms;

    try {
        serial_port.Open(serial_device);
    } catch (const LibSerial::OpenFailed& e) {
        std::cerr << "Failed to open serial port: " << e.what() << std::endl;
        return false;
    }

    return true;
}

void MCUComms::disconnect() { serial_port.Close(); }

bool MCUComms::connected() { return serial_port.IsOpen(); }

std::string MCUComms::sanitize_msg(const std::string& msg) {
    // Remove any null characters from the message
    std::string sanitized_msg = msg;
    sanitized_msg.erase(
        std::remove(sanitized_msg.begin(), sanitized_msg.end(), '\r'),
        sanitized_msg.end());

    sanitized_msg.erase(
        std::remove(sanitized_msg.begin(), sanitized_msg.end(), '\n'),
        sanitized_msg.end());

    return sanitized_msg;
}

template <typename T>
int MCUComms::req_data(uint8_t request_id, std::vector<T>& pcbData) {
    serial_port.FlushIOBuffers();

    uint8_t dataBuffer[256];
    uint8_t responseBuffer[256];
    static uint8_t unpacked_frame_buffer[MCU_USB_COMM_PAYLOAD_SIZE];

    uint8_t dataLength;
    uint8_t numberElements;

    std::size_t bytesToRead;

    DataFrame_TypeDef requestFrame = {
        .frameID = 0, .timestamp = 0, .dataLength = 0, .data = NULL};

    DataFrame_TypeDef responseFrame = {.data = unpacked_frame_buffer};

    switch (request_id) {
        case REQUEST_IMU_ACCEL_RAW:
            bytesToRead =
                BOT_SPEAK_MIN_PACKET_SIZE + SERIALIZED_IMU_ACCEL_RAW_BYTES;
            numberElements = UNSERIALIZED_IMU_ACCEL_RAW_SIZE;
            break;

        case REQUEST_IMU_GYRO_RAW:
            bytesToRead =
                BOT_SPEAK_MIN_PACKET_SIZE + SERIALIZED_IMU_GYRO_RAW_BYTES;
            numberElements = UNSERIALIZED_IMU_GYRO_RAW_SIZE;
            break;

        case REQUEST_IMU_PROCESSED:
            bytesToRead =
                BOT_SPEAK_MIN_PACKET_SIZE + SERIALIZED_IMU_PROCESSED_BYTES;
            numberElements = UNSERIALIZED_IMU_PROCESSED_SIZE;
            break;

        case REQUEST_ENCODER_POSITIONS:
            bytesToRead =
                BOT_SPEAK_MIN_PACKET_SIZE + SERIALIZED_ENCODER_POSITIONS_BYTES;
            numberElements = UNSERIALIZED_ENCODER_POSITIONS_SIZE;
            break;

        case REQUEST_ENCODER_VELOCITIES:
            bytesToRead =
                BOT_SPEAK_MIN_PACKET_SIZE + SERIALIZED_ENCODER_VELOCITIES_BYTES;
            numberElements = UNSERIALIZED_ENCODER_VELOCITIES_SIZE;
            break;

        case REQUEST_MOTOR_CURRENT:
            bytesToRead =
                BOT_SPEAK_MIN_PACKET_SIZE + SERIALIZED_MOTOR_CURRENT_BYTES;
            numberElements = UNSERIALIZED_MOTOR_CURRENT_SIZE;
            break;

        case REQUEST_BATTERY_VOLTAGE:
            bytesToRead =
                BOT_SPEAK_MIN_PACKET_SIZE + SERIALIZED_BATTERY_VOLTAGE_BYTES;
            numberElements = UNSERIALIZED_BATTERY_VOLTAGE_SIZE;
            break;

        case REQUEST_USER_DEFINED_BUTTON:
            bytesToRead = BOT_SPEAK_MIN_PACKET_SIZE +
                          SERIALIZED_USER_DEFINED_BUTTON_BYTES;
            numberElements = UNSERIALIZED_USER_DEFINED_BUTTON_SIZE;
            break;

        default:
            std::cerr << "Unknown request ID: " << request_id << std::endl;
            return 1;
    }

    T* readData = new T[numberElements];

    requestFrame.frameID = request_id;

    try {
        botSpeak_packFrame(&requestFrame, dataBuffer, &dataLength);
        serial_port.write((const char*)dataBuffer, dataLength);

        // wait for a response
        serial_port.read((char*)responseBuffer, bytesToRead);

        // once we get a response, parse it
        int result =
            botSpeak_unpackFrame(&responseFrame, responseBuffer, bytesToRead);

        if (result != 0) {
            std::cerr << "Failed to unpack frame: " << result << std::endl;
            return 1;
        }

        // if the packet could be unpacked (so it at least made some sense), get
        // the actual data from it
        botSpeak_deserialize(readData, &numberElements, sizeof(T),
                             responseFrame.data, responseFrame.dataLength);

        // now we pass this onto the caller
        pcbData.clear();

        for (uint8_t i = 0; i < numberElements; ++i) {
            pcbData.push_back(static_cast<T>(readData[i]));
        }
    }

    catch (const LibSerial::ReadTimeout& ex) {
        std::cerr << "Read timeout: " << ex.what() << std::endl;
        return 1;
    }

    delete[] readData;
    return 0;
}

template <typename T>
int MCUComms::send_data(uint8_t command_id, const std::vector<T>& data) {
    serial_port.FlushIOBuffers();  // Just in case

    T* trasmitBuffer = new T[data.size()];
    std::copy(data.begin(), data.end(), trasmitBuffer);

    uint8_t dataLength;
    uint8_t frameLength;
    uint8_t serializedDataBuffer[256];
    uint8_t frameBuffer[256];

    botSpeak_serialize(trasmitBuffer, data.size(), sizeof(T),
                       serializedDataBuffer, &dataLength);

    DataFrame_TypeDef frame = {.frameID = command_id,
                               .timestamp = 0,
                               .dataLength = dataLength,
                               .data = serializedDataBuffer};

    if (botSpeak_packFrame(&frame, frameBuffer, &frameLength) != 0) {
        std::cerr << "Failed to pack frame" << std::endl;
        return 1;
    }

    serial_port.write((const char*)frameBuffer, frameLength);

    delete[] trasmitBuffer;
    return 0;
}

template <typename T>
int MCUComms::send_config(uint8_t command_id, const std::vector<T>& data) {
    serial_port.FlushIOBuffers();  // Just in case

    T* trasmitBuffer = new T[data.size()];
    std::copy(data.begin(), data.end(), trasmitBuffer);

    uint8_t dataLength;
    uint8_t frameLength;
    uint8_t serializedDataBuffer[256];
    uint8_t frameBuffer[256];

    botSpeak_serialize(trasmitBuffer, data.size(), sizeof(T),
                       serializedDataBuffer, &dataLength);

    DataFrame_TypeDef frame = {.frameID = command_id,
                               .timestamp = 0,
                               .dataLength = dataLength,
                               .data = serializedDataBuffer};

    if (botSpeak_packFrame(&frame, frameBuffer, &frameLength) != 0) {
        std::cerr << "Failed to pack frame" << std::endl;
        return 1;
    }

    serial_port.write((const char*)frameBuffer, frameLength);

    // Add 10ms delay to ensure the command is processed
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return 0;
}


// --- Explicit Template Instantiations ---
// This tells the compiler to pre-build these specific versions of the templates

// For send_config
template int MCUComms::send_config<bool>(uint8_t command_id, const std::vector<bool>& data);
template int MCUComms::send_config<uint32_t>(uint8_t command_id, const std::vector<uint32_t>& data);
template int MCUComms::send_config<float>(uint8_t command_id, const std::vector<float>& data);
template int MCUComms::send_config<MotorControllerMode_TypeDef>(uint8_t command_id, const std::vector<MotorControllerMode_TypeDef>& data);

// For req_data
template int MCUComms::req_data<float>(uint8_t request_id, std::vector<float>& pcbData);
template int MCUComms::req_data<int64_t>(uint8_t request_id, std::vector<int64_t>& pcbData);

// For send_data
template int MCUComms::send_data<float>(uint8_t command_id, const std::vector<float>& data);