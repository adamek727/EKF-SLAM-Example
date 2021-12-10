#pragma once

namespace Colors {

    struct Color {
        float r = 0.0;
        float g = 0.0;
        float b = 0.0;
        float a = 1.0;
    };

    static constexpr auto Black  = Color{.r = 0.0, .g = 0.0, .b = 0.0, .a = 1.0};
    static constexpr auto Red    = Color{.r = 1.0, .g = 0.0, .b = 0.0, .a = 1.0};
    static constexpr auto Yellow = Color{.r = 1.0, .g = 1.0, .b = 0.0, .a = 1.0};
    static constexpr auto Green  = Color{.r = 0.0, .g = 1.0, .b = 0.0, .a = 1.0};
    static constexpr auto Aqua   = Color{.r = 0.0, .g = 1.0, .b = 1.0, .a = 1.0};
    static constexpr auto Blue   = Color{.r = 0.0, .g = 0.0, .b = 1.0, .a = 1.0};
    static constexpr auto Purple = Color{.r = 1.0, .g = 0.0, .b = 1.0, .a = 1.0};
    static constexpr auto White  = Color{.r = 1.0, .g = 1.0, .b = 1.0, .a = 1.0};
    static constexpr auto Invisible  = Color{.r = 1.0, .g = 1.0, .b = 1.0, .a = 1.0};
}