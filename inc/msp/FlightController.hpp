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
    
    virtual bool setMspControlState(const ControlLevel& /*level*/) override {
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
bool FlightController<FirmwareVariant::INAV>::setMspControlState(const ControlLevel& level) {
    if (level == ControlLevel::NONE) {
        return setRadioControlTypeToCachedDefault();
    } else if (level == ControlLevel::COMPLETE) {
        return setRadioControlType(RadioControlType::MSP);
    }
    //we cant actually set shared control via this interface
    return false;
}


}  // namespace msp

#endif  // FLIGHTCONTROLLER_HPP
