#pragma once

#define local inline static

constexpr bool debugging = true;

[[nodiscard]] local constexpr double scaleOutput(double inputMin, double inputMax, double outputMin, double outputMax, double input)
{
    return ((input - inputMin) / (inputMax - inputMin)) * ((outputMax - outputMin)) + outputMin;
}