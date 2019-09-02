#ifndef FLIGHTCONTROLLER_HPP
#define FLIGHTCONTROLLER_HPP


#include "FlightControllerBase.hpp"

namespace msp
{

template <FirmwareVariant firmware>
class FlightController : public FlightControllerBase {
public:
    /**
     * @brief FlightController Constructor
     */
    FlightController() :
        FlightControllerBase()
    {}

    /**
     * @brief ~FlightController Destructor
     */
    ~FlightController()
    { 
        stop(); 
    }
    
    virtual FirmwareVariant getInterfaceFwVariant() const override {
        return firmware;
    }
    
    
    virtual ControlLevel getMspControlCapability() const override {
        return ControlLevel::NONE;
    }
    
    virtual ControlLevel getMspControlState() const override {
        return ControlLevel::NONE;
    }
    
    virtual bool setMspControlState(ControlLevel /*level*/) override {
        return false;
    }
    
};

template <>
ControlLevel FlightController<FirmwareVariant::INAV>::getMspControlCapability() const {
    if (hasMode("MSP RC OVERRIDE")) {
        return ControlLevel::SHARED_OR_COMPLETE;
    }
    return ControlLevel::COMPLETE;
}

template <>
ControlLevel FlightController<FirmwareVariant::INAV>::getMspControlState() const {
    if (isModeActive("MSP RC OVERRIDE")) return ControlLevel::SHARED;
    if (getRadioControlType() == RadioControlType::MSP) return ControlLevel::COMPLETE;
    return ControlLevel::NONE;
}

template <>
bool FlightController<FirmwareVariant::INAV>::setMspControlState(ControlLevel level) {
    //dont do this if there is any chance we are off the ground
    if (isArmed()) return false;
    
    ControlLevel current_level = getMspControlState();
    
    //user has asked for automatic selection
    if (level == ControlLevel::SHARED_OR_COMPLETE) {
        //is shared even an option in this build?
        if (getMspControlCapability() == ControlLevel::SHARED_OR_COMPLETE) {
            //choose shared, unless we are already in complete control
            if (current_level == ControlLevel::COMPLETE) level = ControlLevel::COMPLETE;
            else level = ControlLevel::SHARED;
        } else {
            level = ControlLevel::COMPLETE;
        }
    }
    
    //automatic success if we are already in the correct state
    if (current_level == level) return true;
    
    if (level == ControlLevel::COMPLETE) {
        if (current_level == ControlLevel::SHARED) {
            if (!updateMspModes(std::set<std::string>(),std::set<std::string>{"MSP RC OVERRIDE"})) return false;
        }
        //this transition requires reboot
        if (!setRadioControlType(RadioControlType::MSP)) return false;
        if (!saveSettings()) return false;
        if (!reboot()) return false;
    } else if (level == ControlLevel::SHARED) {
        if (getMspControlCapability() != ControlLevel::SHARED_OR_COMPLETE) return false;
        
        //shared only works if the primary radio is not MSP
        if (!setRadioControlType(RadioControlType::SERIAL)) return false;
        if (!saveSettings()) return false;
        if (!reboot()) return false;
        if (!updateMspModes(std::set<std::string>{"MSP RC OVERRIDE"})) return false;
    } else if (level == ControlLevel::NONE) {
        if (current_level == ControlLevel::SHARED) {
            if (!updateMspModes(std::set<std::string>(),std::set<std::string>{"MSP RC OVERRIDE"})) return false;
        } else {
            //TODO: be more intelligent here...
            if (!setRadioControlTypeToCachedDefault()) return false;
            if (!saveSettings()) return false;
            if (!reboot()) return false;
        }
    } 
    
    return true;
}


}  // namespace msp

#endif  // FLIGHTCONTROLLER_HPP
