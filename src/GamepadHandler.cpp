#include "EKF-SLAM-Example/GamepadHandler.h"

GamepadHandler::GamepadHandler(std::shared_ptr<rclcpp::Node> node, const Config& conf)
        : node_{node}
        , conf_{conf}
        , gamepadHandler_{nullptr} {

    auto gamepad_ms = static_cast<size_t>(conf_.gamepad_period * 1000.0f);
    gamepad_timer_ = node_->create_wall_timer(100ms, std::bind(&GamepadHandler::on_gamepad_timer_event, this));

    gamepad_joystick_dead_zone_ = 0.1;
    reset_states();

    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        throw std::runtime_error("Failed to init SDL2.");
    }
}


void GamepadHandler::on_gamepad_timer_event() {

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_CONTROLLERAXISMOTION:
                on_joystick_axis_moved(event.caxis.axis, event.caxis.value);
                joystick_event_callback_();
                break;
            case SDL_CONTROLLERBUTTONDOWN:
                break;
            case SDL_CONTROLLERDEVICEADDED:
                if (gamepadHandler_ == nullptr) {
                    gamepadHandler_ = SDL_GameControllerOpen(0);
                    RCLCPP_INFO(node_->get_logger(), "New gamepad opened.");
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Attempted to open multiple gamepads at once.");
                }
                break;
            case SDL_CONTROLLERDEVICEREMOVED:
                SDL_GameControllerClose(gamepadHandler_);
                RCLCPP_INFO(node_->get_logger(), "Gamepad disconnected.");
                gamepadHandler_ = nullptr;

                reset_states();
                joystick_event_callback_();
                break;
            default:
                break;
        }
    }
}


void GamepadHandler::on_joystick_axis_moved(uint8_t axis, int16_t value) {
    switch (axis) {
        case SDL_CONTROLLER_AXIS_LEFTX:
            axis_values_[JoystickAxis::LEFT_X] = (value / 32767.0f);
            if (std::abs(axis_values_[JoystickAxis::LEFT_X]) < gamepad_joystick_dead_zone_) {
                axis_values_[JoystickAxis::LEFT_X] = 0;
            }
            break;
        case SDL_CONTROLLER_AXIS_LEFTY:
            axis_values_[JoystickAxis::LEFT_Y]  = (value / 32767.0f);
            if (std::abs(axis_values_[JoystickAxis::LEFT_Y]) < gamepad_joystick_dead_zone_) {
                axis_values_[JoystickAxis::LEFT_Y] = 0;
            }
            break;
        case SDL_CONTROLLER_AXIS_RIGHTX:
            axis_values_[JoystickAxis::RIGHT_X]  = (value / 32767.0f);
            if (std::abs(axis_values_[JoystickAxis::RIGHT_X]) < gamepad_joystick_dead_zone_) {
                axis_values_[JoystickAxis::RIGHT_X] = 0;
            }
            break;
        case SDL_CONTROLLER_AXIS_RIGHTY:
            axis_values_[JoystickAxis::RIGHT_Y] = (value / 32767.0f);
            if (std::abs(axis_values_[JoystickAxis::RIGHT_Y]) < gamepad_joystick_dead_zone_) {
                axis_values_[JoystickAxis::RIGHT_Y] = 0;
            }
            break;
        default:
            break;
    }
}

void GamepadHandler::reset_states() {
    axis_values_[JoystickAxis::LEFT_X] = 0;
    axis_values_[JoystickAxis::LEFT_Y] = 0;
    axis_values_[JoystickAxis::RIGHT_X] = 0;
    axis_values_[JoystickAxis::RIGHT_Y] = 0;
}
