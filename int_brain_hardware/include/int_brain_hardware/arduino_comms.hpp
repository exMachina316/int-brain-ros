#ifndef int_brain_hardware__ARDUINO_COMMS_HPP
#define int_brain_hardware__ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include <algorithm>
#include "bot_speak.h"
#include "int_brain_messages.h"

using LibSerial::DataBuffer;

class ArduinoComms
{

public:
  ArduinoComms() = default;

  bool connect(const std::string &serial_device, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    try
    {
      serial_conn_.Open(serial_device);
    }
    catch (const LibSerial::OpenFailed &e)
    {
      std::cerr << "Failed to open serial port: " << e.what() << std::endl;
      return false;
    }

    return true;
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  std::string sanitize_msg(const std::string &msg)
  {
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
  int req_data(uint8_t request_id, std::vector<T> &pcbData)
  {
    serial_conn_.FlushIOBuffers(); // Just in case

    uint8_t dataBuffer[256];
    uint8_t responseBuffer[256];
    DataBuffer responseBufferVec;

    uint8_t dataLength;
    uint8_t numberElements;

    std::size_t bytesToRead;

    switch (request_id)
    {
    case REQUEST_IMU_RAW:
      bytesToRead = BOT_SPEAK_MIN_PACKET_SIZE + IMU_RAW_BYTES;
      numberElements = 6;
      break;
    case REQUEST_IMU_PROCESSED:
      bytesToRead = BOT_SPEAK_MIN_PACKET_SIZE + IMU_PROCESSED_BYTES;
      numberElements = 4;
      break;
    case REQUEST_ENCODER_POSITIONS:
      bytesToRead = BOT_SPEAK_MIN_PACKET_SIZE + ENCODER_POSITIONS_BYTES;
      numberElements = 4;
      break;
    case REQUEST_ENCODER_VELOCITIES:
      bytesToRead = BOT_SPEAK_MIN_PACKET_SIZE + ENCODER_VELOCITIES_BYTES;
      numberElements = 2;
      break;
    case REQUEST_MOTOR_CURRENT:
      bytesToRead = BOT_SPEAK_MIN_PACKET_SIZE + MOTOR_CURRENT_BYTES;
      numberElements = 2;
      break;
    case REQUEST_BATTERY_VOLTAGE:
      bytesToRead = BOT_SPEAK_MIN_PACKET_SIZE + BATTERY_VOLTAGE_BYTES;
      numberElements = 1;
      break;
    case REQUEST_USER_DEFINED_BUTTON:
      bytesToRead = BOT_SPEAK_MIN_PACKET_SIZE + USER_DEFINED_BUTTON_BYTES;
      numberElements = 1;
      break;
    default:
      std::cerr << "Unknown request ID: " << request_id << std::endl;
      return 1;
    }

    T *readData = new T[numberElements];

    DataFrame_TypeDef frame = {
        .frameID = request_id,
        .timestamp = 0,
        .dataLength = 0,
        .data = NULL
    };

    DataFrame_TypeDef responseFrame;

    try
    {
      botSpeak_packFrame(&frame, dataBuffer, &dataLength);
      DataBuffer dataBufferVec(dataBuffer, dataBuffer + dataLength);
      serial_conn_.Write(dataBufferVec);

      // Wait for a response
      serial_conn_.Read(responseBufferVec, bytesToRead, timeout_ms_);

      responseBufferVec.resize(bytesToRead);
      std::copy(responseBufferVec.begin(), responseBufferVec.end(), responseBuffer);

      int result = botSpeak_unpackFrame(&responseFrame, responseBuffer, bytesToRead);
      if (result != 0)
      {
        std::cerr << "Failed to unpack frame: " << result << std::endl;
        return 1;
      }

      botSpeak_deserialize(readData, &numberElements, sizeof(T), responseFrame.data, responseFrame.dataLength);
      pcbData.clear();
      for (uint8_t i = 0; i < numberElements; ++i)
      {
        pcbData.push_back(static_cast<T>(readData[i]));
      }
    }
    catch (const LibSerial::ReadTimeout &ex)
    {
      std::cerr << "Read timeout: " << ex.what() << std::endl;
      return 1;
    }

    delete[] readData;
    return 0;
  }

  template <typename T>
  int send_data(uint8_t command_id, const std::vector<T> &data)
  {
    serial_conn_.FlushIOBuffers(); // Just in case

    T *trasmitBuffer = new T[data.size()];
    std::copy(data.begin(), data.end(), trasmitBuffer);

    uint8_t dataLength;
    uint8_t frameLength;
    uint8_t serializedDataBuffer[256];
    uint8_t frameBuffer[256];

    botSpeak_serialize(trasmitBuffer, data.size(), sizeof(T), serializedDataBuffer, &dataLength);

    DataFrame_TypeDef frame = {
        .frameID = command_id,
        .timestamp = 0,
        .dataLength = dataLength,
        .data = serializedDataBuffer
    };

    if (botSpeak_packFrame(&frame, frameBuffer, &frameLength) != 0)
    {
      std::cerr << "Failed to pack frame" << std::endl;
      return 1;
    }

    DataBuffer frameBufferVec(frameBuffer, frameBuffer + frameLength);
    serial_conn_.Write(frameBufferVec);

    return 0;
  }

template <typename T>
  int send_config(uint8_t command_id, const std::vector<T> &data)
  {
    serial_conn_.FlushIOBuffers(); // Just in case

    T *trasmitBuffer = new T[data.size()];
    std::copy(data.begin(), data.end(), trasmitBuffer);

    uint8_t dataLength;
    uint8_t frameLength;
    uint8_t serializedDataBuffer[256];
    uint8_t frameBuffer[256];

    botSpeak_serialize(trasmitBuffer, data.size(), sizeof(T), serializedDataBuffer, &dataLength);

    DataFrame_TypeDef frame = {
        .frameID = command_id,
        .timestamp = 0,
        .dataLength = dataLength,
        .data = serializedDataBuffer
    };

    if (botSpeak_packFrame(&frame, frameBuffer, &frameLength) != 0)
    {
      std::cerr << "Failed to pack frame" << std::endl;
      return 1;
    }

    DataBuffer frameBufferVec(frameBuffer, frameBuffer + frameLength);
    serial_conn_.Write(frameBufferVec);

    // Add 10ms delay to ensure the command is processed
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    return 0;
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

#endif // int_brain_hardware__ARDUINO_COMMS_HPP