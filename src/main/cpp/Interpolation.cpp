#include "Interpolation.hpp"
#include "ngr.hpp"

#include <array>

struct table_row
{
    double y_val;
    double hood_val;
    double fly_speed;
};

struct point
{
    double x;
    double y;
};

/******************************************************************/
/*                       Private Constants                        */
/******************************************************************/
constexpr std::array<table_row, 6> lookup_table{
    table_row{16.1, -17.023779, .5},
    table_row{10.8, -18.190428, .6},
    table_row{5.6, -19.476120, .7},
    table_row{1.95, -20.190395, .8},
    table_row{-0.9, -20.809433, .9},
    table_row{-3.6, -21.118952, 1}};

/******************************************************************/
/*                  Private Function Definitions                  */
/******************************************************************/

constexpr table_row const *findValueInTable(double const &yval)
{
    return ngr::findIf(std::next(std::begin(lookup_table)), std::end(lookup_table), [=](auto const &val)
                       { return yval >= val.y_val; });
}

constexpr double interpolate(double const &x_value, point const &lower_bound, point const &upper_bound)
{
    double const slope = (upper_bound.y - lower_bound.y) / (upper_bound.x - lower_bound.x);
    double const adjusted_x_value = x_value - lower_bound.x;
    double const y_intercept = lower_bound.y;

    // This is just y = ax + b;
    return slope * adjusted_x_value + y_intercept;
}

/******************************************************************/
/*                   Public Function Definitions                  */
/******************************************************************/

constexpr double Interpolation::getHoodValue(double const &y_val)
{
    auto const lower = findValueInTable(y_val);
    // Previous value since the interpolation table is descending
    auto const upper = std::prev(lower);

    // Constructs a point (x,y) for the region below and above the y_val, then turns that into a linear equation.
    // Plugs in y_val as x. Returns hood value as y.
    return interpolate(y_val, {lower->y_val, lower->hood_val}, {upper->y_val, upper->hood_val});
}

constexpr double Interpolation::getFlySpeed(double const &y_val)
{
    auto const lower = findValueInTable(y_val);
    // Previous value since the interpolation table is descending
    auto const upper = std::prev(lower);

    // Constructs a point (x,y) for the region below and above the y_val, then turns that into a linear equation.
    // Plugs in y_val as x. Returns fly speed as y.
    return interpolate(y_val, {lower->y_val, lower->fly_speed}, {upper->y_val, upper->fly_speed});
}

/******************************************************************/
/*                           Logic Tests                          */
/******************************************************************/
static_assert(std::end(lookup_table) - std::begin(lookup_table) >= 2, "Lookup table too small!");

static_assert(
    findValueInTable(lookup_table[3].y_val)->y_val == lookup_table[3].y_val,
    "Error with findValueInTable()");

static_assert(ngr::isSorted(std::begin(lookup_table), std::end(lookup_table),
                            [](auto const &lhs, auto const &rhs)
                            { return lhs.y_val > rhs.y_val; }),
              "Lookup table not sorted");

static_assert(ngr::isCloseTo(Interpolation::getHoodValue(16), -17.0, .2), "Error with getHoodValue()");
static_assert(ngr::isCloseTo(Interpolation::getHoodValue(-1), -20.8, .2), "Error with getHoodValue()");
static_assert(ngr::isCloseTo(Interpolation::getFlySpeed(11), .6, .05), "Error with getFlySpeed()");
static_assert(ngr::isCloseTo(Interpolation::getFlySpeed(-3.5), 1, .05), "Error with getFlySpeed()");
