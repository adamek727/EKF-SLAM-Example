/* Note: Gamepad control inspired by Matous Hybl's code [https://github.com/matoushybl] */

#pragma once

#include <rclcpp/node.hpp>
#include <SDL2/SDL.h>
#include <chrono>

using namespace std::chrono_literals;

class GamepadHandler {

    enum class JoystickAxis {
        LEFT_X,
        LEFT_Y,
        RIGHT_X,
        RIGHT_Y,
    };


public:

    GamepadHandler() = delete;
    GamepadHandler(std::shared_ptr<rclcpp::Node> node);

    // switching X and Y axis (compatibility with right hand system (x - forward, y - left)
    float get_left_x() {return -axis_values_[JoystickAxis::LEFT_Y];}
    float get_left_y() {return -axis_values_[JoystickAxis::LEFT_X];}
    float get_right_x() {return -axis_values_[JoystickAxis::RIGHT_Y];}
    float get_right_y() {return -axis_values_[JoystickAxis::RIGHT_X];}

    void set_joystick_event_callback(std::function<void()> f) {
        joystick_event_callback_ = f;
    }

protected:

    void on_gamepad_timer_event();
    void on_joystick_axis_moved(uint8_t axis, int16_t value);
    void reset_states();

    std::shared_ptr<rclcpp::Node> node_;
    SDL_GameController *gamepadHandler_;
    rclcpp::TimerBase::SharedPtr gamepad_timer_;

    float gamepad_joystick_dead_zone_;
    std::map<JoystickAxis, float> axis_values_;

    std::function<void()> joystick_event_callback_ = {};
};