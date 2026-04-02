#include <libserial/SerialStream.h>
#include "bot_speak.h"

#ifndef int_brain_hardware__MCU_COMMS_HPP_
#define int_brain_hardware__MCU_COMMS_HPP_

// MCU comms config
#define MCU_USB_COMM_PAYLOAD_SIZE sizeof(uint64_t) * 10

/// @brief Maximum payload size for USB communication, used to create buffers.
#define	MCU_USB_COMM_MAX_PACKET_SIZE (BOT_SPEAK_MIN_PACKET_SIZE + MCU_USB_COMM_PAYLOAD_SIZE)

class MCUComms {
   private:
    LibSerial::SerialStream serial_port;
    int timeout_ms_;

   public:
    MCUComms() = default;
    ~MCUComms();

    bool connect(const std::string& serial_device, int32_t timeout_ms);
    void disconnect();
    bool connected();

    std::string sanitize_msg(const std::string& msg);

    template <typename T>
    int req_data(uint8_t request_id, std::vector<T>& pcbData);

    template <typename T>
    int send_data(uint8_t command_id, const std::vector<T>& data);

    template <typename T>
    int send_config(uint8_t command_id, const std::vector<T> &data);
};

#endif  // int_brain_hardware__MCU_COMMS_HPP_