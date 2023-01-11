#ifndef VISION_H_
#define VISION_H_

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <vector>
#include <units/angle.h>
#include <units/length.h>

class Vision
{
private:

  std::shared_ptr<nt::NetworkTable> m_table =
    nt::NetworkTableInstance::GetDefault().GetTable("limelight-brute");

public:
Vision(/* args */);
~Vision();

std::vector<units::meters> get_field_pos_by_tag();
std::vector<double> get_xy_offset();
units::radians get_rotation_by_tag();

};
#endif
