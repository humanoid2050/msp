
#include "FlightController.hpp"
#include "FCFactory.hpp"

int main(int argc, char *argv[]) {
    const std::string device =
        (argc > 1) ? std::string(argv[1]) : "/dev/ttyUSB0";
    const size_t baudrate = (argc > 2) ? std::stoul(argv[2]) : 115200;

    auto fcu = msp::FlightControllerFactory::create(device, baudrate);
    if (!fcu->start(device, baudrate)) {
        std::cout << "fcu start failed" << std::endl;
        return -1;
    }
    
    msp::msg::InavStatus<> istatus;
    fcu->sendMessage(istatus);
    std::cout << istatus;
    //fcu->updateMspModes(std::set<std::string>(), std::set<std::string>{"FAILSAFE"});
    std::cout << "modes at start:" << std::endl;
    fcu->printActiveModes();
    std::cout << "adding modes" << std::endl;
    fcu->updateMspModes(std::set<std::string>{"ANGLE", "NAV ALTHOLD", "SURFACE"});
    std::cout << "arming" << std::endl;
    fcu->arm();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "modes after add and arm:" << std::endl;
    fcu->printActiveModes();
    
    std::cout << "disarming" << std::endl;
    fcu->disarm();
    std::cout << "modes after disarm" << std::endl;
    fcu->printActiveModes();
//    fcu->printActiveModes();
    std::cout << "removing modes" << std::endl;
    fcu->updateMspModes(std::set<std::string>(), std::set<std::string>{"ANGLE", "NAV ALTHOLD", "SURFACE"});
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "modes after remove:" << std::endl;
    fcu->printActiveModes();
    
    
}
