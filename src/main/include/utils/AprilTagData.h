#pragma once

#include <string>
#include <map>
#include <units/length.h>

// Define a struct to represent data associated with an April Tag ID
class AprilTagData {
  public:
    AprilTagData(); //constructor

    // Member function to add data associated with an ID
    void addAprilTagData(int id, units::length::meter_t height, const std::string& targetType, const std::string& redOrBlueAlliance);
    units::length::meter_t returnAprilTagDataHeight(int id);
    const std::string returnAprilTagDataTargetType(int id);
    const std::string returnAprilTagDataTargetAlliance(int id);
    
private:
struct AprilTagInfo {
    int id;
    units::length::meter_t height;
    std::string targetType;
    std::string redOrBlueAlliance;
};
std::map<int, AprilTagInfo> idAprilTagDataMap;
};