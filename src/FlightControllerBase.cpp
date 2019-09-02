#include "FlightControllerBase.hpp"

namespace msp {

FlightControllerBase::FlightControllerBase() : 
    Client(),
    config_cache_valid_(false),
    default_rx(RadioControlType::NONE),
    msp_timer_(std::bind(&FlightControllerBase::generateMSP, this), 0.1) 
{}
   
FlightControllerBase::~FlightControllerBase()
{
    msp_timer_.stop();
    stop();
}

bool FlightControllerBase::start(const std::string& device, const size_t baudrate)
{
    device_name_ = device;
    baudrate_ = baudrate;
    if (!Client::start(device, baudrate)) return false;
    if (!initConfigurationCache()) return false;
    return msp_timer_.start();
}

bool FlightControllerBase::stop()
{
    msp_timer_.stop();
    config_cache_valid_ = false;
    return Client::stop();
}

bool FlightControllerBase::setRadioControlType(const RadioControlType& source)
{
    if (source == RadioControlType::NONE) return false;
    
    //we need to set MSP as the rc source
    if(source == getRadioControlType()) return true;

    msp::msg::RxConfig<> rxConfig;
    if(!sendMessage(rxConfig)) return false;

    msp::msg::SetRxConfig<> setRxConfig;
    static_cast<msp::msg::RxConfigSettings &>(setRxConfig) =
        static_cast<msp::msg::RxConfigSettings &>(rxConfig);

    setRxConfig.receiverType = uint8_t(source);

    return sendMessage(setRxConfig);
}

bool FlightControllerBase::setRadioControlTypeToCachedDefault()
{
    return setRadioControlType(default_rx);
}

void FlightControllerBase::setDefaultRadioControlType()
{
    default_rx = getRadioControlType();
}
    
void FlightControllerBase::setDefaultRadioControlType(const RadioControlType& source)
{
    default_rx = source;
}


RadioControlType FlightControllerBase::getRadioControlType() const
{
    msp::msg::RxConfig<> rxConfig;
    sendMessage(rxConfig);
    return RadioControlType(rxConfig.receiverType());
}


void FlightControllerBase::setRPYT(std::array<double, 4> &rpyt) 
{
    {
        std::lock_guard<std::mutex> lock(msp_updates_mutex);
        rpyt_.swap(rpyt);
    }
    generateMSP();
}


void FlightControllerBase::generateMSP()
{
    std::vector<uint16_t> cmds(4, 1000);
    {
        std::lock_guard<std::mutex> lock(msp_updates_mutex);
        cmds[channel_map_[0]] = uint16_t(rpyt_[0] * 500 + 1500);
        cmds[channel_map_[1]] = uint16_t(rpyt_[1] * 500 + 1500);
        cmds[channel_map_[2]] = uint16_t(rpyt_[2] * 500 + 1500);
        cmds[channel_map_[3]] = uint16_t(rpyt_[3] * 500 + 1500);
    }
    setRc(cmds);
}


bool FlightControllerBase::saveSettings()
{
    msp::msg::WriteEEPROM<> writeEEPROM;
    return sendMessage(writeEEPROM);
}


bool FlightControllerBase::reboot()
{
    config_cache_valid_ = false;
    msp::msg::Reboot<> reboot;
    if (!sendMessage(reboot)) return false;
    if (!stop()) return false;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    return start(device_name_, baudrate_);
}


FirmwareVariant FlightControllerBase::getFwVariant() const
{
    msp::msg::FcVariant<> fcvar;
    if(sendMessage(fcvar)) {
        return msp::variant_map.at(fcvar.identifier());

    }
    return FirmwareVariant::NONE;
}

std::string FlightControllerBase::getBoardName() const
{
    msp::msg::BoardInfo<> boardinfo;
    if(sendMessage(boardinfo)) {
        return boardinfo.name();
    }
    
    return std::string(); 
}


bool FlightControllerBase::setRc(const uint16_t& roll, const uint16_t& pitch, 
           const uint16_t& yaw, const uint16_t& throttle, const uint16_t& aux1,
           const uint16_t& aux2, const uint16_t& aux3, const uint16_t& aux4,
           const std::vector<uint16_t>& auxs)
{
    msp::msg::SetRawRc<> rc;
    // insert mappable channels
    rc.channels.resize(msp::msg::MAX_MAPPABLE_RX_INPUTS);
    rc.channels[channel_map_[0]] = roll;
    rc.channels[channel_map_[1]] = pitch;
    rc.channels[channel_map_[2]] = yaw;
    rc.channels[channel_map_[3]] = throttle;

    rc.channels.emplace_back(aux1);
    rc.channels.emplace_back(aux2);
    rc.channels.emplace_back(aux3);
    rc.channels.emplace_back(aux4);

    // insert remaining aux channels
    rc.channels.insert(std::end(rc.channels), std::begin(auxs), std::end(auxs));

    // send MSP_SET_RAW_RC without waiting for ACK
    return sendMessageNoWait(rc);
}


bool FlightControllerBase::setRc(const std::vector<uint16_t>& channels)
{
    msp::msg::SetRawRc<> rc;
    rc.channels = channels;
    return sendMessageNoWait(rc);
}


bool FlightControllerBase::hasCapability(const msp::msg::Capability &cap) const 
{
    return capabilities_.count(cap);
}


bool FlightControllerBase::hasBind() const 
{
    return hasCapability(msp::msg::Capability::BIND); 
}


bool FlightControllerBase::hasDynBal() const 
{
    return hasCapability(msp::msg::Capability::DYNBAL);
}


bool FlightControllerBase::hasFlap() const 
{
    return hasCapability(msp::msg::Capability::FLAP); 
}


bool FlightControllerBase::hasSensor(const msp::msg::Sensor &sensor) const 
{
    return sensors_.count(sensor);
}


bool FlightControllerBase::hasAccelerometer() const 
{
    return hasSensor(msp::msg::Sensor::Accelerometer);
}



bool FlightControllerBase::hasBarometer() const 
{
    return hasSensor(msp::msg::Sensor::Barometer); 
}


bool FlightControllerBase::hasMagnetometer() const 
{
    return hasSensor(msp::msg::Sensor::Magnetometer);
}


bool FlightControllerBase::hasGPS() const 
{
    return hasSensor(msp::msg::Sensor::GPS); 
}


bool FlightControllerBase::hasSonar() const
{
    return hasSensor(msp::msg::Sensor::Sonar); 
}

const std::map<std::string, const uint8_t>& FlightControllerBase::getModeNames() const 
{
    return box_name_to_perm_id_;
}

bool FlightControllerBase::setMspModes(const std::set<std::string> &modes)
{
     if (!config_cache_valid_) return false;
     
    msp_mode_cache_.clear();
    for (unsigned char i = 0; i < box_names_.size(); ++i) {
        if (modes.count(box_names_[i])) msp_mode_cache_.emplace(i);
    }
    
    msp::msg::SetBox<> boxes_out;
    boxes_out.requested_box_ids = msp_mode_cache_;
    if(!sendMessage(boxes_out)) return false;
    return true;
}

bool FlightControllerBase::updateMspModes (
    const std::set<std::string> &add, const std::set<std::string> &remove)
{
    if (!config_cache_valid_) return false;
    
    for (unsigned char i = 0; i < box_names_.size(); ++i) {
        if (add.count(box_names_[i])) msp_mode_cache_.emplace(i);
        if (remove.count(box_names_[i])) msp_mode_cache_.erase(i);
    }
    
    msp::msg::SetBox<> boxes_out;
    boxes_out.requested_box_ids = msp_mode_cache_;

    if(!sendMessage(boxes_out)) return false;
    
    return true;
}

bool FlightControllerBase::hasMode(const std::string& name) const
{
    return getModeNames().count(name);
}

bool FlightControllerBase::hasMode(uint8_t permanent_id) const
{
    for (const auto& entry : box_name_to_perm_id_) {
        if (entry.second == permanent_id) return true;
    }
    return false;
}


bool FlightControllerBase::isModeActive(const std::string &status_name) const
{
    if (!config_cache_valid_) return false;
    if (!hasMode(status_name)) return false;
    msp::msg::Status<> status;
    sendMessage(status);

    // check if box id is amongst active box IDs
    return status.box_mode_flags.count(box_name_to_perm_id_.at(status_name));
}


bool FlightControllerBase::isArmed() const
{
    return isModeActive("ARM"); 
}


bool FlightControllerBase::isFailsafe() const
{
    return isModeActive("FAILSAFE"); 
}

bool FlightControllerBase::arm()
{
    return updateMspModes(std::set<std::string>{"ARM"});
}

bool FlightControllerBase::disarm()
{
    return updateMspModes(std::set<std::string>(), std::set<std::string>{"ARM"});
}

void FlightControllerBase::printActiveModes() const
{
    if (!config_cache_valid_) return;
    
    msp::msg::ActiveBoxes<> boxes;
    sendMessage(boxes);
    std::cout << "status count: " << boxes.active_ids.size() << std::endl;
    for (const auto& indx : boxes.active_ids) {
        std::cout << " " << box_names_[indx];
    }
    std::cout << std::endl;
    
}


bool FlightControllerBase::setMotors(const std::array<uint16_t, msp::msg::N_MOTOR> &motor_values)
{
    msp::msg::SetMotor<> motor;
    motor.motor = motor_values;
    return sendMessage(motor);
}


int FlightControllerBase::updateFeatures(
    const std::set<std::string> &add, const std::set<std::string> &remove)
{
    // get original feature configuration
    msp::msg::Feature<> feature_in;
    if(!sendMessage(feature_in)) return -1;

    // update feature configuration
    msp::msg::SetFeature<> feature_out;
    feature_out.features = feature_in.features;
    // enable features
    for(const std::string &a : add) {
        feature_out.features.insert(a);
    }
    // disable features
    for(const std::string &rem : remove) {
        feature_out.features.erase(rem);
    }

    // check if feature configuration changed
    if(feature_out.features == feature_in.features) return 0;

    if(!sendMessage(feature_out)) return -1;

    // make settings permanent and reboot
    if(!saveSettings()) return -1;
    if(!reboot()) return -1;

    return 1;
}

bool FlightControllerBase::initConfigurationCache(const double &timeout, const bool print_info) 
{
    if(getFwVariant() != FirmwareVariant::MWII) {
        msp::msg::ApiVersion<> api_version;
        if(sendMessage(api_version, timeout)) {
            if(print_info) std::cout << api_version;
            setMspVersion(api_version.major());
        }
    }

    msp::msg::Status<> status;
    if (sendMessage(status, timeout)) {
        if(print_info) std::cout << status;
        sensors_ = status.sensors;
    }

    msp::msg::Ident<> ident;
    if (sendMessage(ident, timeout)) {
        if(print_info) std::cout << ident;
        capabilities_ = ident.capabilities;
    }
    
    // get box names
    msp::msg::BoxNames<> box_names;
    if(!sendMessage(box_names))
        throw std::runtime_error("Cannot get BoxNames!");
    box_names_.swap(box_names.box_names);

    // get box IDs (permanent ID in INAV)
    msp::msg::BoxIds<> box_ids;
    if(!sendMessage(box_ids))
        throw std::runtime_error("Cannot get BoxIds!");
    assert(box_names_.size() == box_ids.box_ids.size());

    box_name_to_perm_id_.clear();
    for(size_t ibox(0); ibox < box_names_.size(); ibox++) {
        box_name_to_perm_id_.emplace(std::make_pair(box_names_[ibox], box_ids.box_ids[ibox]));
    }

    // determine channel mapping
    if(getFwVariant() == msp::FirmwareVariant::MWII) {
        // default mapping
        for(uint8_t i(0); i < msp::msg::MAX_MAPPABLE_RX_INPUTS; ++i) {
            channel_map_[i] = i;
        }
    }
    else {
        // get channel mapping from MSP_RX_MAP
        msp::msg::RxMap<> rx_map;
        sendMessage(rx_map, timeout);
        if(print_info) std::cout << rx_map;
        channel_map_ = rx_map.map;
    }
    
    config_cache_valid_ = true;
    return true;
}

}
