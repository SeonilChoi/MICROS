#ifndef MICROS_COMMON_TYPE_HPP_
#define MICROS_COMMON_TYPE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace micros {

inline constexpr uint8_t MAX_INTERFACE_SIZE  = 16; // Maximum number of command or state entries per driver.
inline constexpr uint8_t MAX_CONTROLLER_SIZE = 16; // Maximum number of controllers for motor manager.

struct motor_state_t {
    uint8_t target_id[MAX_INTERFACE_SIZE]{0}; // Interface ID requried by control commands.
    uint8_t number_of_targets{0}; // Number of valid target IDs in target_id[].

    uint8_t id; // Controller ID
    uint16_t controlword{}; // Controlword
    uint16_t statusword{};  // Statusword
    uint16_t errorcode{};   // Error code
    double position{};      // Current or Target Position
    double velocity{};      // Current or Target Velocity
    double torque{};        // Current or Target Torque
};

struct motor_state_gate_t {
    std::atomic<uint64_t> count{0};
    motor_state_t data[MAX_CONTROLLER_SIZE]{};

    void write(const motor_state_t* state, uint8_t size) {
        for (uint8_t i = 0; i < size; ++i) data[i] = state[i];
        for (uint8_t i = size; i < MAX_CONTROLLER_SIZE; ++i) data[i] = motor_state_t{};
        count.fetch_add(1, std::memory_order_release);
    }

    bool read(motor_state_t* state, uint8_t size, uint64_t & last_count) {
        const uint64_t current_count = count.load(std::memory_order_acquire);
        if (current_count == last_count) return false;

        for (uint8_t i = 0; i < size; ++i) state[i] = data[i];
        last_count = current_count;
        return true;
    }
};

template <typename T>
inline void convert_to_ros_message(const motor_state_t* data, uint8_t size, T& msg)
{
    msg.data.resize(size);
    for (uint8_t i = 0; i < size; ++i) {
        const uint8_t n = data[i].number_of_targets;
        msg.data[i].target_id.assign(data[i].target_id, data[i].target_id + n);
        msg.data[i].number_of_targets = data[i].number_of_targets;
        msg.data[i].id = data[i].id;
        msg.data[i].controlword = data[i].controlword;
        msg.data[i].statusword = data[i].statusword;
        msg.data[i].errorcode = data[i].errorcode;
        msg.data[i].position = data[i].position;
        msg.data[i].velocity = data[i].velocity;
        msg.data[i].torque = data[i].torque;
    }
}

template <typename T>
inline void convert_from_ros_message(const T& msg, motor_state_t* data)
{
    for (uint8_t i = 0; i < msg.data.size(); ++i) {
        const auto& tid = msg.data[i].target_id;
        for (size_t j = 0; j < tid.size(); ++j) data[i].target_id[j] = tid[j];
        data[i].number_of_targets = msg.data[i].number_of_targets;
        data[i].id = msg.data[i].id;
        data[i].controlword = msg.data[i].controlword;
        data[i].statusword = msg.data[i].statusword;
        data[i].errorcode = msg.data[i].errorcode;
        data[i].position = msg.data[i].position;
        data[i].velocity = msg.data[i].velocity;
        data[i].torque = msg.data[i].torque;
    }
}

} // namespace micros
#endif // #ifndef MICROS_COMMON_TYPE_HPP_