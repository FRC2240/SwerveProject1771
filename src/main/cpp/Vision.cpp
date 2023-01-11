#include "Vision.h"

Vision::Vision()
{
  m_table->PutNumber("ledMode", 1); // Disable lights on boot
}


std::vector<units::meters> get_field_pos_by_tag()
  {
    /*
     *Gets the position of the robot on the field.
     *Used to reset odometry and for auto placement.
     *Use the position from a known apriltag to get position.
     *Return a vector of (Meters X, Meters Y)
     *
     **/

  }


std::vector<double> get_xy_offset()
  {

  }

units::radians get_rotation_by_tag();
