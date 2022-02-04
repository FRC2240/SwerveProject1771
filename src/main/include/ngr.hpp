#pragma once

constexpr bool debugging = true;

// Helper functions
namespace ngr
{
    // floating point comparison at compile time
    [[nodiscard]] constexpr bool isCloseTo(double value,
                                           double target,
                                           double tol = 0.00001)
    {
        return fabs(value - target) < tol;
    }

    static_assert(isCloseTo(.5, .500000001) == true);
    static_assert(isCloseTo(.1, .999989) == false);
    static_assert(isCloseTo(.1, .10002) == false);

    [[nodiscard]] constexpr double scaleOutput(double inputMin, double inputMax, double outputMin, double outputMax, double input)
    {
        return ((input - inputMin) / (inputMax - inputMin)) * ((outputMax - outputMin)) + outputMin;
    }

    static_assert(isCloseTo(scaleOutput(0, 1, -1, 1, 0), -1));
    static_assert(isCloseTo(scaleOutput(0, 1, -1, 1, 1), 1));
    static_assert(isCloseTo(scaleOutput(0, 1, -1, 1, .5), 0));

    // bad, but good enough implimentation of std::midpoint from C++20
    // remove this if upgraded to C++20
    template <typename A, typename B>
    constexpr std::common_type_t<A, B> midpoint(A const &a, B const &b)
    {
        return (a + b) / 2;
    }

    template <class ForwardIt>
    [[nodiscard]] constexpr bool isSorted(ForwardIt const &first, ForwardIt const &last, bool const &descending = false)
    {
        auto current = first;
        auto next = first;
        next++;

        while (next < last)
        {
            if (descending)
            {
                if (*(next++) > *(current++))
                    return false;
            }
            else
            {
                if (*(next++) < *(current++))
                    return false;
            }
        }
        return true;
    }

    static_assert([]()
                  {
                      constexpr std : array<double, 3> ascending_array{-.09, 1, 10000};
                      constexpr std::array<double, 3> descending_array{1, -0.8, -33000};

                      return (
                          isSorted(ascending_array.begin(), ascending_array.end()) 
                      &&
                      isSorted(descending_array.begin(), descending_array.end(), true)); }());

} // namespace ngr