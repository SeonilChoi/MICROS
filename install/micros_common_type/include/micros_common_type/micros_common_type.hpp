#ifndef MICROS_COMMON_TYPE_HPP_
#define MICROS_COMMON_TYPE_HPP_

#include <atomic>
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
    motor_state_t data[MAX_CONTROLLER_SIZE]{0};

    void write(const motor_state_t* state, uint8_t size) {
        for (uint8_t i = 0; i < size; ++i) data[i] = state[i];
        for (uint8_t i = size; i < MAX_CONTROLLER_SIZE; ++i) data[i] = motor_state_t{0};
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

} // namespace micros
#endif // #ifndef MICROS_COMMON_TYPE_HPP_