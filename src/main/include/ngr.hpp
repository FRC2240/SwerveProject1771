#pragma once

constexpr bool debugging = true;

[[nodiscard]] constexpr double scaleOutput(double inputMin, double inputMax, double outputMin, double outputMax, double input)
{
    return ((input - inputMin) / (inputMax - inputMin)) * ((outputMax - outputMin)) + outputMin;
}