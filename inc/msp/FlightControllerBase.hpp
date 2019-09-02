#ifndef FLIGHTCONTROLLERBASE_HPP
#define FLIGHTCONTROLLERBASE_HPP

#include "Client.hpp"
#include "PeriodicTimer.hpp"
#include "msp_msg.hpp"

namespace msp {

enum class RadioControlType : uint8_t
{ NONE, PWM, PPM, SERIAL, MSP, SPI, UIB };

enum class ControlLevel { NONE, SHARED, COMPLETE, SHARED_OR_COMPLETE };

class FlightControllerBase : public Client {
    
protected:
    FlightControllerBase();
public:    
    virtual ~FlightControllerBase();

    virtual bool start(const std::string& device, const size_t baudrate = 115200) override;
    
    virtual bool stop() override;
    
    virtual FirmwareVariant getInterfaceFwVariant() const = 0;
    
    virtual ControlLevel getMspControlCapability() const = 0;
    
    virtual ControlLevel getMspControlState() const = 0;
    
    virtual bool setMspControlState(ControlLevel level) = 0;

    void setDefaultRadioControlType();
    
    void setDefaultRadioControlType(const RadioControlType& source);
    
    /**
     * @brief Sets which instruction source the flight controller should
     * listen to. Also starts periodic MSP control message if MSP is selected
     * @param source RadioControlType reference
     */
    bool setRadioControlType(const RadioControlType& source);
    
    bool setRadioControlTypeToCachedDefault();

    /**
     * @brief Queries the currently active control source
     * @return RadioControlType object matching current control source
     */
    RadioControlType getRadioControlType() const;

    /**
     * @brief Sets roll, pitch, yaw, and throttle values. They are sent
     * immediately and stored for resending if automatic MSP control is enabled.
     * @param rpyt An array of doubles representing roll, pitch, yaw, and
     * throttle in that order. Values are expected to be in the range -1.0 to
     * +1.0, mapping to 1000us to 2000us pulse widths.
     */
    void setRPYT(std::array<double, 4> &rpyt);
    
    /**
     * @brief Method used to generate the Rc message sent to the flight
     * controller
     */
    void generateMSP();

    /**
     * @brief Sends message to flight controller to save all settings
     */
    bool saveSettings();

    /**
     * @brief Starts message to flight controller signalling that it should
     * reboot. Will result in the serial device disappearing if using a direct
     * USB connection to the board.
     */
    bool reboot();

    /**
     * @brief Queries the currently set firmware variant (Cleanflight,
     * Betaflight, etc.)
     * @return msp::FirmwareVariant enum matching the current firmware
     */
    FirmwareVariant getFwVariant() const;

    /**
     * @brief Queries the currently set board name
     * @return std::String of the board name
     */
    std::string getBoardName() const;

    /**
     * @brief Set RC channels in order: roll, pitch, yaw, throttle by using
     * channel mapping
     * @param roll
     * @param pitch
     * @param yaw
     * @param throttle
     * @param aux1
     * @param aux2
     * @param aux3
     * @param aux4
     * @param auxs
     * @return
     */
    bool setRc(const uint16_t& roll, const uint16_t& pitch, const uint16_t& yaw,
               const uint16_t& throttle, const uint16_t& aux1 = 1000,
               const uint16_t& aux2 = 1000, const uint16_t& aux3 = 1000,
               const uint16_t& aux4 = 1000,
               const std::vector<uint16_t>& auxs = std::vector<uint16_t>());

    /**
     * @brief Set RC channels in raw order as it is interpreted by the FC
     * @param channels list of channel values (1000-2000)
     * @return
     */
    bool setRc(const std::vector<uint16_t>& channels);
    

    /**
     * @brief Queries the cached flight controller information to see
     * if a particular capability is present
     * @return True if the sensor is present
     */
    bool hasCapability(const msp::msg::Capability &cap) const;
    /**
     * @brief Queries for the presence of the BIND capability
     * @return True if the BIND capaibility is present
     */
    bool hasBind() const;

    /**
     * @brief Queries for the presence of the DYNBAL capability
     * @return True if the DYNBAL capaibility is present
     */
    bool hasDynBal() const;
    /**
     * @brief Queries for the presence of the FLAP capability
     * @return True if the FLAP capaibility is present
     */
    bool hasFlap() const;

    /**
     * @brief Queries the cached flight controller information to see
     * if a particular sensor is present
     * @return True if the sensor is present
     */
    bool hasSensor(const msp::msg::Sensor &sensor) const;

    /**
     * @brief Queries for the presence of an accelerometer
     * @return True if there is an accelerometer
     */
    bool hasAccelerometer() const;

    /**
     * @brief Queries for the presence of a barometer
     * @return True if there is a barometer
     */
    bool hasBarometer() const;

    /**
     * @brief Queries for the presence of a magentometer
     * @return True if there is a magentometer
     */
    bool hasMagnetometer() const;

    /**
     * @brief Queries for the presence of a GPS
     * @return True if there is a GPS
     */
    bool hasGPS() const;

    /**
     * @brief Queries for the presence of a sonar
     * @return True if there is a sonar
     */
    bool hasSonar() const;

    /**
     * @brief Gets the information on the box to permanent ID map
     * @return Reference to the internal mapping of strings to mode IDs
     */
    const std::map<std::string, const uint8_t>& getModeNames() const;
    
    bool setMspModes(const std::set<std::string> &modes);
    
    bool updateMspModes (
        const std::set<std::string> &add    = std::set<std::string>(),
        const std::set<std::string> &remove = std::set<std::string>());
    
    bool hasMode(const std::string& name) const;
    
    bool hasMode(uint8_t permanent_id) const;
    
    /**
     * @brief Queries the flight controller to see if a status is active
     * @return True if status if active
     */
    bool isModeActive(const std::string &status_name) const;
    
    void printActiveModes() const;

    /**
     * @brief Queries the flight controller to see if the ARM status is active.
     * Not to be confused with armSet(), which queries whether the flight
     * controller has been instructued to turn on the ARM status.
     * @return True if the ARM status is active
     */
    bool isArmed() const;

    /**
     * @brief Queries the flight controller to see if the FAILSAFE status is
     * active.
     * @return True if the FAILSAFE status is active
     */
    bool isFailsafe() const;

    bool arm();
    
    bool disarm();
    
    /**
     * @brief Directly sets motor values using SetMotor message
     * @return True on successful message delivery
     */
    bool setMotors(const std::array<uint16_t, msp::msg::N_MOTOR> &motor_values);

    /**
     * @brief Enable and disable features on the FC
     * To apply updates, changes will be written to the EEPROM and the FC will
     * reboot.
     * @param add set of features to enable
     * @param remove set of features to disable
     * @return 1 if features have been changed
     * @return 0 if no changes have been applied
     * @return -1 on failure
     */
    int updateFeatures(
        const std::set<std::string> &add    = std::set<std::string>(),
        const std::set<std::string> &remove = std::set<std::string>());

private:
    bool initConfigurationCache(const double &timeout = 0.0, const bool print_info = false);

private:
    // parameters updated by the connect method to cache flight controller info
    bool config_cache_valid_;
    
    std::string device_name_;
    size_t baudrate_;
    
    std::map<std::string, const uint8_t> box_name_to_perm_id_;
    std::vector<std::string> box_names_;
    std::set<uint8_t> msp_mode_cache_;
    std::set<msp::msg::Sensor> sensors_;
    std::array<uint8_t, msp::msg::MAX_MAPPABLE_RX_INPUTS> channel_map_;
    std::set<msp::msg::Capability> capabilities_;
    RadioControlType default_rx;

    // parameters updated by the user, and consumed by MSP control messages
    std::array<double, 4> rpyt_;

    mutable std::mutex msp_updates_mutex;

    msp::PeriodicTimer msp_timer_;

};


}

#endif
