
#include "utils/AprilTagData.h"
#include <units/length.h>

//Constructor
AprilTagData::AprilTagData(){}

//void addAprilTagData(int id, double height, const std::string& targetType, const std::string& redOrBlueAlliance);

void AprilTagData::addAprilTagData(int id, units::length::meter_t height, const std::string& targetType, const std::string& redOrBlueAlliance) {
    idAprilTagDataMap[id] = {id, height, targetType, redOrBlueAlliance};
}

units::length::meter_t AprilTagData::returnAprilTagDataHeight(int id){
  return idAprilTagDataMap[id].height;
}
const std::string AprilTagData::returnAprilTagDataTargetType(int id){
  return idAprilTagDataMap[id].targetType;
}
const std::string AprilTagData::returnAprilTagDataTargetAlliance(int id){
  return idAprilTagDataMap[id].redOrBlueAlliance;
}


