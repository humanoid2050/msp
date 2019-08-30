#ifndef FC_FACTORY_HPP
#define FC_FACTORY_HPP

#include "FlightController.hpp"
#include "Client.hpp"
#include "FirmwareVariants.hpp"

#include <iostream>

namespace msp {


class FlightControllerFactory {
public:

    static std::unique_ptr<FlightControllerBase> create(const std::string& device, const size_t baudrate = 115200)
    {
        Client client;
        client.start(device,baudrate);
        msp::msg::FcVariant<> fcvar;
        std::unique_ptr<FlightControllerBase> fc_ptr;
        if(client.sendMessage(fcvar)) {
            switch(msp::variant_map.at(fcvar.identifier())) {
                case FirmwareVariant::MWII:
                    fc_ptr.reset(new FlightController<FirmwareVariant::MWII>());
                    break;
                case FirmwareVariant::BAFL:
                    fc_ptr.reset(new FlightController<FirmwareVariant::BAFL>());
                    break;
                case FirmwareVariant::BTFL:
                    fc_ptr.reset(new FlightController<FirmwareVariant::BTFL>());
                    break;
                case FirmwareVariant::CLFL:
                    fc_ptr.reset(new FlightController<FirmwareVariant::CLFL>());
                    break;
                case FirmwareVariant::INAV:
                    fc_ptr.reset(new FlightController<FirmwareVariant::INAV>());
                    break;
                case FirmwareVariant::RCFL:
                    fc_ptr.reset(new FlightController<FirmwareVariant::RCFL>());
                    break;
                default:
                    ;
            }
        }
        return fc_ptr;
    }
    
};

}

#endif
