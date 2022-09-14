#include "ell.h"

/*****************************************
 *
 * Utility
 *
 *****************************************/
std::string elliptec::int2addr(uint8_t id) {
    char hexstring[] = {'0', '0'};
    sprintf(hexstring, "%X", id);
    std::string s = std::string(hexstring);
    return s;
}

bool elliptec::devintype(std::string type, uint8_t id) {
    return std::find(devtype[type].begin(), devtype[type].end(), id) != devtype[type].end();
}

bool elliptec::devislinrot(std::string addr) {
    auto dev = devinfo_at_addr(addr);
    if (!dev.has_value()){
        throw std::runtime_error("Device with address " + addr + " not in connected device list");
    }
    return devintype("linrot", dev.value().type);
}

bool elliptec::devislinear(std::string addr) {
    auto dev = devinfo_at_addr(addr);
    if (!dev.has_value()){
        throw std::runtime_error("Device with address " + addr + " not in connected device list");
    }
    return devintype("linear", dev.value().type);
}
bool elliptec::devisrotary(std::string addr) {
    auto dev = devinfo_at_addr(addr);
    if (!dev.has_value()){
        throw std::runtime_error("Device with address " + addr + " not in connected device list");
    }
    return devintype("rotary", dev.value().type);
}
bool elliptec::devispaddle(std::string addr) {
    auto dev = devinfo_at_addr(addr);
    if (!dev.has_value()){
        throw std::runtime_error("Device with address " + addr + " not in connected device list");
    }
    return devintype("paddle", dev.value().type);
}
bool elliptec::devispiezo(std::string addr) {
    auto dev = devinfo_at_addr(addr);
    if (!dev.has_value()){
        throw std::runtime_error("Device with address " + addr + " not in connected device list");
    }
    return devintype("piezo", dev.value().type);
}

int64_t elliptec::deg2step(std::string addr, double deg) {
    auto dev = devinfo_at_addr(addr);
    return std::round(dev.value().pulses * deg / 360);
}

int64_t elliptec::mm2step(std::string addr, double mm){
    auto dev = devinfo_at_addr(addr);
    return std::round(dev.value().pulses * mm);
}

double elliptec::step2deg(std::string addr, int64_t step) {
    auto dev = devinfo_at_addr(addr);
    double deg = 360.0 * step / dev.value().pulses;
    return deg;
}

double elliptec::step2mm(std::string addr, int64_t step){
    auto dev = devinfo_at_addr(addr);
    return 1.0 * step / dev.value().pulses;
}

std::string elliptec::step2hex(int64_t step, uint8_t width) {
    if (step < 0) {
        step += (1ULL << 32);
    }
    std::stringstream ss;
    ss << std::setfill ('0') << std::setw(width) << std::hex << step;
    std::string s = ss.str();
    std::transform(s.begin(), s.end(), s.begin(), ::toupper);

    return s;
}

int64_t elliptec::hex2step(std::string s) {
    int64_t i = stoll(s, nullptr, 16);
    if (i > (1LL << 31)) {
        i -= (1LL << 32);
    }
    return i;
}

std::string elliptec::ll2hex(int64_t i) {
    if (i < 0) {
        i += (1LL<<32);
    }

    std::stringstream ss;
    ss << std::setfill ('0') << std::setw(8) << std::hex << i;
    std::string s = ss.str();

    std::transform(s.begin(), s.end(), s.begin(), ::toupper);
    return s;
}

std::string elliptec::us2hex(uint16_t i) {
    std::stringstream ss;
    ss << std::setfill ('0') << std::setw(4) << std::hex << i;
    std::string s = ss.str();

    std::transform(s.begin(), s.end(), s.begin(), ::toupper);
    return s;
}

std::string elliptec::uc2hex(uint8_t i) {
    std::stringstream ss;
    ss << std::setfill ('0') << std::setw(2) << std::hex << unsigned(i);
    std::string s = ss.str();

    std::transform(s.begin(), s.end(), s.begin(), ::toupper);
    return s;
}

void elliptec::print_dev_info(ell_device dev) {
    std::cout << "motor address: " << dev.address << std::endl;
    std::cout << "device type: " << dev.type << std::endl;
    std::cout << "serial number: " << dev.serial << std::endl;
    std::cout << "manufacturing year: " << dev.year << std::endl;
    std::cout << "fw revision: " << (unsigned)dev.fw << std::endl;
    std::cout << "hw revision: " << (unsigned)dev.hw << std::endl;
    std::cout << "travel: " << dev.travel << std::endl;
    std::cout << "pulses per unit: " << dev.pulses << std::endl;
}

void elliptec::print_addr_info(std::string addr) {
    bool found = false;
    for (auto dev: devices){
        if (dev.address == addr) {
            found = true;
            std::cout << "motor address: " << dev.address << std::endl;
            std::cout << "device type: " << dev.type << std::endl;
            std::cout << "serial number: " << dev.serial << std::endl;
            std::cout << "manufacturing year: " << dev.year << std::endl;
            std::cout << "fw revision: " << (unsigned)dev.fw << std::endl;
            std::cout << "hw revision: " << (unsigned)dev.hw << std::endl;
            std::cout << "travel: " << dev.travel << std::endl;
            std::cout << "pulses per unit: " << dev.pulses << std::endl;
        }
    }
    if (!found) {
        std::cout << "Device with address " << addr << " not found" << std::endl;
    }
}

uint8_t elliptec::parsestatus(std::string msg) {
    uint8_t code = 0;
    if (!msg.substr(1,2).compare(std::string("GS"))) {
        code = hex2step(msg.substr(3,2));
    }
    return code;
}

std::string elliptec::err2string(uint8_t code) {
    std::string errstring;
    if (code > 13) {
        errstring = "error code unknown";
    } else {
        errstring = error_msgs.at(code);
    }
    return errstring;
}
