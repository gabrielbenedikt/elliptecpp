#include "ell.h"

elliptec::elliptec(const std::string devname, const std::vector<uint8_t> inmids, const bool dohome, const bool freqsearch) : _inmids{std::move(inmids)}, _devname(devname)
{
    _dohome = dohome;
    _dofreqsearch = freqsearch;
    
    devtype["rotary"] = {8, 14, 18};
    devtype["linear"] = {7, 10, 17, 20};
    devtype["linrot"] = {7, 8, 10, 14, 17, 18, 20};
    devtype["indexed"] = {6, 9, 12};
    devtype["hasclean"] = {14, 17, 18, 20};
    devtype["paddle"] = {3};
    devtype["piezo"] = {5};

    _ser_timeout = 5;
    
    if (_inmids.size() > 1) {
        std::sort(_inmids.begin(), _inmids.end());
    }
    
    mids.reserve(_inmids.size());
    for (uint8_t id: _inmids) {
        if (id > 15) {
            std::cout << "ERROR: elliptec motor id has to be 0 < id < 15" << std::endl;
        } else {
            mids.push_back(int2addr(id));
        }
    }
    
    bserial = std::unique_ptr<Boost_serial>(new Boost_serial(_devname, 9600,
                               boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
                               boost::asio::serial_port_base::character_size(8),
                               boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none),
                               boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one)));
    bserial->setTimeout(boost::posix_time::seconds(30));

    open(_devname);
    
    for (std::string id : mids) {
        get_info(id);
        if (freqsearch) {
            search_freq(id);
            //save_userdata(id);
        }
        if (dohome) {
            home(id);
        }
        get_position(id);
    }
    bserial->setTimeout(boost::posix_time::seconds(_ser_timeout));
}

elliptec::~elliptec()
{
    bserial->close();
}

std::optional<ell_device> elliptec::devinfo_at_addr(std::string addr) {
    std::cout << "number of connected devices: " << devices.size() << std::endl;
    for (ell_device d : devices) {
        if (d.address == addr) {
            return d;
        }
    }
    return {};
}

void elliptec::handle_devinfo(ell_device dev) {
    bool is_new_dev = true;
    for (ell_device d : devices) {
        if (d.serial == dev.serial) {
            is_new_dev = false;
        }
    }
    if (is_new_dev) {
        devices.push_back(dev);
    }
}

ell_response elliptec::process_response(std::string response) {
    if (!response.compare("")) {
        response = read();
    }
    std::string addstr = response.substr(0,1);
    std::string command = response.substr(1,2);
    std::string data = response.substr(3,response.npos);
    uint8_t addr = std::stoi(addstr.data(),nullptr,16);

    ell_response ret;
    ret.address = addr;
    ret.type = command;
    ret.data = data;
    
    if (!command.compare(std::string("GS"))) {
        uint8_t code = parsestatus(response);
        if (code!=0) {
            std::string errstring = err2string(code);
            std::cout << "Got error" << errstring << std::endl;
        }
    } else if (!command.compare(std::string("PO"))) {
        auto dev = devinfo_at_addr(addstr);
        int64_t step = hex2step(ret.data);
        double pos = 0;
        if (devislinear(addstr)) {
            pos = step2mm(addstr, step);
            std::cout << pos << "mm" << std::endl;
            _current_pos = pos;
        } else if (devisrotary(addstr)) {
            pos = step2deg(addstr, step);
            std::cout << pos << "deg" << std::endl;
            _current_pos = pos;
        } 
    } else if (!command.compare(std::string("GJ"))) {
        auto dev = devinfo_at_addr(addstr);
        int64_t step = hex2step(ret.data);
        double jogsize = 0;
        if (devislinear(addstr)) {
            jogsize = step2mm(addstr, step);
            std::cout << "jogsize: " << jogsize << "mm" << std::endl;
        } else if (devisrotary(addstr)) {
            jogsize = step2deg(addstr, step);
            std::cout << "jogsize: " << jogsize << "deg" << std::endl;
        } 
    } else if (!command.compare(std::string("GV"))) {
        auto dev = devinfo_at_addr(addstr);
        uint64_t percent = hex2step(ret.data);
        std::cout << "speed: " << percent << "%" << std::endl; 
    } else if ((!command.compare(std::string("P1"))) || (!command.compare(std::string("P2"))) || (!command.compare(std::string("P3")))) {
        uint8_t pnum = std::stoi(command.substr(1,1).data(), nullptr, 10);
        
        std::cout << "Paddle " << unsigned(pnum) << " position: " << 3.0*hex2step(ret.data) << "deg" << std::endl;
        return ret;
    } else {
        throw std::runtime_error("Return code not recognized: " + response);
    }

    return ret;
}

/*****************************************
 *
 * Low levels
 *
 *****************************************/
void elliptec::get_info(std::string addr){
    std::string msg = addr + "in";
    write(msg.data());
    
    std::string response = read();
    if (response.substr(1,2).compare(std::string("IN")) == 0) {
        std::string addstr = response.substr(0,1);
        uint16_t type   = std::stoi(response.substr(3,2).data(),  nullptr, 16);
        uint64_t sn     = std::stoi(response.substr(5,8).data(),  nullptr, 10);
        uint16_t year   = std::stoi(response.substr(13,4).data(), nullptr, 10);
        uint8_t fwrel   = std::stoi(response.substr(17,2).data(), nullptr, 10);
        uint8_t hwrel   = std::stoi(response.substr(19,2).data(), nullptr, 10);
        uint16_t travel = std::stoi(response.substr(21,4).data(), nullptr, 16);
        uint64_t pulses = std::stoi(response.substr(25,8).data(), nullptr, 16);

        ell_device dev;
        dev.address = addstr;
        dev.type = type;
        dev.serial = sn;
        dev.year = year;
        dev.fw = fwrel;
        dev.hw = hwrel;
        dev.travel = travel;
        dev.pulses = pulses;
        handle_devinfo(dev);
    }
    //reply with info response
}

void elliptec::get_motor_info(std::string addr, uint8_t motor_num){
    if ((motor_num > 3) || (motor_num < 1)) {
        throw std::invalid_argument("motor_num has to be 1, 2 or 3");
    } 
    std::string msg = addr + "i" + std::to_string(motor_num);
    write(msg.data());
    
    std::string response = read();
    if ((response.substr(1,2).compare(std::string("I1"))) || (response.substr(1,2).compare(std::string("I2"))) || (response.substr(1,2).compare(std::string("I3")))) {
        uint8_t ret_mnum    = std::stoi(response.substr(0,1).data(),  nullptr, 16);
        uint8_t loopOn      = std::stoi(response.substr(3,1).data(),  nullptr, 10);
        uint8_t motorOn     = std::stoi(response.substr(4,1).data(),  nullptr, 10);
        uint16_t current    = std::stoi(response.substr(5,4).data(),  nullptr, 16);
        uint16_t rampUp     = std::stoi(response.substr(9,4).data(),  nullptr, 16);
        uint16_t rampDown   = std::stoi(response.substr(13,4).data(), nullptr, 16);
        uint16_t periodFwd  = std::stoi(response.substr(17,4).data(), nullptr, 16);
        uint16_t periodBwd  = std::stoi(response.substr(21,4).data(), nullptr, 16);
        
        std::cout << "Motor " << ret_mnum << " info\n";
        std::cout << "Loop on       : " << unsigned(loopOn) << "\n";
        std::cout << "Motor on      : " << unsigned(motorOn) << "\n";
        std::cout << "Current       : " << 1.0*current/1.866 << "mA\n";
        std::cout << "Ramp up       : " << rampUp << " PWM increase / ms\n";
        std::cout << "Ramp down     : " << rampDown << " PWM decrease / ms\n";
        std::cout << "Fwd frequency : " << 14740000.0/periodFwd << " kHz\n";
        std::cout << "Bwd frequency : " << 14740000.0/periodBwd << " kHz\n";
        std::cout << std::endl;
    } else {
        process_response(response);
    }
}

void elliptec::set_motor_freq(std::string addr, std::string dir, uint8_t motor_num, uint16_t freq_khz, bool factory_reset){
    std::string msg = addr;
    if ((motor_num > 3) || (motor_num < 1)) {
        throw std::invalid_argument("motor_num has to be 1, 2 or 3");
    } 
    
    if (dir == "fwd") {
        msg += "f" + std::to_string(motor_num);
    } else if (dir == "bwd") {
        msg += "b" + std::to_string(motor_num);
    } else {
        throw std::invalid_argument("direction string has to be 'bwd' or 'fwd'.");
    }
    
    if (factory_reset) {
        msg += "8FFF";
    } else {
        if (freq_khz > 78) {
            throw std::invalid_argument("freq has to be < 78");
        } else {
            std::string hexfreq = us2hex(freq_khz);
            hexfreq[0]='8';
            msg += hexfreq;
        }
    }
    write(msg.data());
    process_response();
    //reply with status package
}

void elliptec::search_motor_freq(std::string addr, uint8_t motor_num){
    if ((motor_num > 3) || (motor_num < 1)) {
        throw std::invalid_argument("motor_num has to be 1, 2 or 3");
    } 
    std::string msg = addr + "s" + std::to_string(motor_num);;
    write(msg.data());
    std::string response = read();
    if (response.substr(1,3).compare(std::string("GS0"))){
        save_userdata(addr);
    } else {
        process_response(response);
    }
    //reply with info response
}

void elliptec::scan_motor_current_curve(std::string addr, uint8_t motor_num) {
    if ((motor_num > 3) || (motor_num < 1)) {
        throw std::invalid_argument("motor_num has to be 1, 2 or 3");
    }
    std::string msg = addr + "c" + std::to_string(motor_num);;
    write(msg.data());
    process_response();
    //reply with GS
}

std::vector<std::pair<uint8_t, uint8_t>> elliptec::get_motor_current_curve(std::string addr, uint8_t motor_num) {
    if ((motor_num > 3) || (motor_num < 1)) {
        throw std::invalid_argument("motor_num has to be 1, 2 or 3");
    }
    std::string msg = addr + "C" + std::to_string(motor_num);
    write(msg.data());
    std::string response = read();
    uint8_t period = 0;
    uint8_t current = 0;
    std::pair<uint8_t, uint8_t> p = {0, 0};
    std::vector<std::pair<uint8_t, uint8_t>> result;
    if (response.substr(1,2) == "C" + std::to_string(motor_num)) {
        std::string payload = response.substr(3,response.npos);
        if (payload.length() < 522) {
            throw std::runtime_error("bad device response:\n"+response);
        }
        for (uint8_t i=0; i<87; ++i) {
            period = std::stoi(payload.substr(0+6*i,2).data(), nullptr, 10);
            current = std::stoi(payload.substr(2+6*i,4).data(), nullptr, 10);;
            p = {period, current};
            result.push_back(p);
        }
    } else {
        process_response(response);
    }
    
    return result;
}

void elliptec::isolate_device(std::string addr, uint8_t minutes){
    std::string msg = addr + "is" + uc2hex(minutes);
    write(msg.data());
    //no response
}

void elliptec::home(std::string addr, std::string dir) {
    std::string msg = addr + "ho" + dir;
    write(msg.data());
    process_response();
    //reply with GS (while moving) or PO
}

void elliptec::paddle_home(std::string addr, uint8_t paddle_num) {
    if ((paddle_num < 1) || (paddle_num > 7)) {
        throw std::invalid_argument("Paddle specifyer has to be 1...7");
    } else {
        
    }
    std::string msg = addr + "ho" + std::to_string(paddle_num);
    write(msg.data());
    process_response();
    //reply with GS (while moving) or PO
}

// TODO: This assumes that, after trying to move to position, 
// ell_response ret = process_response() is returning a position. 
// Harden.
void elliptec::move_absolute(std::string addr, double pos) {
    std::string msg = addr + "ma";
    std::string hstepstr = "";
    auto dev = devinfo_at_addr(addr);
    if (dev.has_value()) {
        if (devintype("rotary", dev.value().type)) {
            int64_t step = deg2step(addr, pos);
            hstepstr = step2hex(step);
        } else if (devintype("linear", dev.value().type)) {
            int64_t step = mm2step(addr, pos);
            hstepstr = step2hex(step);
        } else {
            std::cout << "device of type" << dev.value().type << " neither linear nor rotary." << std::endl;
        }
    } else {
        std::cout << "device with address " << addr << " not in connected device list" << std::endl;
    }

    if (hstepstr == "") {
        std::cout << "something went wrong in move_relative" << std::endl;
    } else {
        msg += hstepstr;

        uint8_t retcnt = 0;
        uint64_t steps = 0;
        double ERR=0;
        double retpos=0;
        while (retcnt < 5) {
            steps=0;
            retpos=0;
            ERR=0;
            write(msg.data());
            ell_response ret = process_response();
            steps = hex2step(ret.data);
            if (devintype("linear", devinfo_at_addr(addr)->type)) {
                ERR = MMERR;
                retpos = step2mm(addr, steps);
            } else {
                ERR = DEGERR;
                retpos = step2deg(addr, steps);
            }
            if (std::abs(pos-retpos) > ERR) {
                std::cout << "ERROR: rotation failed: moved " << retpos << " while trying to move " << pos << std::endl;
                ++retcnt;
            } else {
                std::cout << "rotation succeeded: moved " << retpos << " while trying to move " << pos << std::endl;
                retcnt = 5;
            }
        }
    }
    //reply with GS (while moving) or PO
}

// TODO: This assumes that, after trying to move to position, 
// ell_response ret = process_response() is returning a position. 
// Harden.
void elliptec::move_relative(std::string addr, double pos) {
    std::string msg = addr + "mr";
    std::string hstepstr = "";
    auto dev = devinfo_at_addr(addr);
    if (dev.has_value()) {
        if (devintype("rotary", dev.value().type)) {
            int64_t step = deg2step(addr, pos);
            hstepstr = step2hex(step);
        } else if (devintype("linear", dev.value().type)) {
            int64_t step = mm2step(addr, pos);
            hstepstr = step2hex(step);
        } else {
            throw std::invalid_argument("Only linear and rotary devices support relative movement");
        }
    } else {
        throw std::runtime_error("Device with address " + addr + " not in connected device list");
    }

    if (hstepstr == "") {
        std::cout << "something went wrong in move_relative" << std::endl;
    } else {
        get_position(addr);
        msg += hstepstr;

        uint8_t retcnt = 0;
        uint64_t steps = 0;
        double ERR = 0;
        double retpos = 0;
        double oldpos = _current_pos;
        while (retcnt < 5) {
            write(msg.data());
            ell_response ret = process_response();
            steps = hex2step(ret.data);
            ERR=0;
            retpos=0;
            if (devintype("linear", devinfo_at_addr(addr)->type)) {
                ERR = MMERR;
                retpos = step2mm(addr, steps);
            } else {
                ERR = DEGERR;
                retpos = step2deg(addr, steps);
            }
            if (std::abs(retpos-oldpos-pos) > ERR) {
                std::cout << "ERROR: rotation failed: moved " << retpos-oldpos << " while trying to move " << pos<< std::endl;
                ++retcnt;
            } else {
                std::cout << "rotation succeeded: moved " << retpos-oldpos << " while trying to move " << pos  << std::endl;
                retcnt = 5;
            }
        }
    }
    //reply with GS (while moving) or PO
}


double elliptec::get_home_offset(std::string addr) {
    auto dev = devinfo_at_addr(addr);
    if (dev.has_value()){
        if (!devintype("linrot", dev.value().type)) {
            throw std::invalid_argument("Only linear and rotary devices support home offset");
        }
    } else {
        throw std::runtime_error("Device with address " + addr + " not in connected device list");
    }
    std::string msg = addr + "go";
    write(msg.data());
    std::string response = read();
    double home_offset = 0;
    if (response.substr(1,2).compare(std::string("HO"))) {
        uint64_t pulses = hex2step(response.substr(2,8));
        if (devintype("linear", devinfo_at_addr(addr)->type)) {
            home_offset = step2mm(addr, pulses);
        } else {
            home_offset = step2deg(addr, pulses);
        }
    } else {
        process_response(response);
    }
    
    return home_offset;
}

void elliptec::set_home_offset(std::string addr, double offset) {
    std::string hexoffset = "";
    if (devislinear(addr)) {
        hexoffset = step2hex(mm2step(addr, offset));
    } else if (devisrotary(addr)) {
        hexoffset = step2hex(deg2step(addr, offset));
    } else {
        throw std::invalid_argument("Only linear and rotary devices support home offset");
    }
    std::string msg = addr + "so" + hexoffset;
    write(msg.data());
    process_response();
    //no response ?
}

double elliptec::get_jogstep_size(std::string addr) {
    if (!devislinrot(addr)) {
        throw std::invalid_argument("Only linear and rotary devices support home offset");
    }
    std::string msg = addr + "gj";
    write(msg.data());
    std::string response = read();
    double jss = 0;
    if (response.substr(1,2).compare(std::string("GJ"))) {
        uint64_t pulses = hex2step(response.substr(2,8));
        if (devintype("linear", devinfo_at_addr(addr)->type)) {
            jss = step2mm(addr, pulses);
        } else {
            jss = step2deg(addr, pulses);
        }
    } else {
        process_response(response);
        return -1;
    }
    
    return jss;
}

void elliptec::set_jogstep_size(std::string addr, double jss) {
    std::string hexjss = "";
    if (devislinear(addr)) {
        hexjss = step2hex(mm2step(addr, jss));
    } else if (devisrotary(addr)) {
        hexjss = step2hex(deg2step(addr, jss));
    } else {
        throw std::invalid_argument("Only linear and rotary devices support jog step size");
    }
    std::string msg = addr + "sj" + hexjss;
    write(msg.data());
    process_response();
    //no response ?
}

void elliptec::move_fwd(std::string addr){
    std::string msg = addr + "fw";
    write(msg.data());
    process_response();
    //reply with GS (while moving) or PO
}

void elliptec::move_bwd(std::string addr){
    std::string msg = addr + "bw";
    write(msg.data());
    process_response();
    //reply with GS (while moving) or PO
}

void elliptec::stop(std::string addr){
    std::string msg = addr + "ms";
    write(msg.data());
    process_response();
    //reply with PO
}

void elliptec::get_position(std::string addr) {
    std::string msg = addr + "gp";
    write(msg.data());
    process_response();
    //reply with GS (while moving) or PO
}

uint8_t elliptec::get_velocity(std::string addr) {
    std::string msg = addr + "gv";
    write(msg.data());
    uint8_t percent = 0;
    std::string response = read();
    if (response.substr(1,2).compare(std::string("GV"))) {
        percent = uint8_t(hex2step(response.substr(3,2)));
    } else {
        process_response(response);
    }
    return percent; 
    //reply with GV
}

void elliptec::set_velocity(std::string addr, uint8_t percent) {
    std::string msg = addr + "sv" + uc2hex(percent);
    write(msg.data());
    process_response();
    //no reply?
}

void elliptec::groupaddress(std::string addr, std::string groupaddr) {
    std::string msg = addr + "ga" + groupaddr;
    write(msg.data());
    process_response();
    //reply with GS
}

void elliptec::paddle_drivetime(std::string addr, uint8_t padnum, uint16_t ms, std::string direction){
    if ((padnum < 1) || (padnum > 3)) {
        throw std::invalid_argument("motor_num has to be 1, 2 or 3");
    }
    if (!(direction.compare(std::string("bwd"))) && !(direction.compare(std::string("fwd")))) {
        throw std::invalid_argument("direction has to be fwd or bwd.");
    }
    if (!devispaddle(addr)) {
        throw std::invalid_argument("only paddles support command -drivetime-");
    }
    std::string time = us2hex(ms);
    if (direction.compare(std::string("bwd"))) {
        time[0] = '8';
    }
    std::string msg = addr + "t" + std::to_string(padnum) + time;
    write(msg.data());
    process_response();
    //reply with P1/P2/P3 (position) or error
}

void elliptec::paddle_moveabsolute(std::string addr, uint8_t padnum, double deg){
    if ((padnum < 1) || (padnum > 3)) {
        throw std::invalid_argument("motor_num has to be 1, 2 or 3");
    }
    if (!devispaddle(addr)) {
        throw std::invalid_argument("only paddles support command -paddle_moveabsolute-");
    }
    uint32_t step = std::lround(deg/0.33);
    std::string msg = addr + "a" + std::to_string(padnum) + step2hex(step, 4);
    write(msg.data());
    process_response();
    //reply with P1/P2/P3 (position) or error
}

void elliptec::paddle_moverelative(std::string addr, uint8_t padnum, double deg){
    if ((padnum < 1) || (padnum > 3)) {
        throw std::invalid_argument("motor_num has to be 1, 2 or 3");
    }
    if (!devispaddle(addr)) {
        throw std::invalid_argument("only paddles support command -paddle_moverelative-");
    }
    int32_t step = std::lround(deg/0.33);
    std::string msg = addr + "r" + std::to_string(padnum) + step2hex(step, 4);
    write(msg.data());
    process_response();
    //reply with P1/P2/P3 (position) or error
}

void elliptec::save_userdata(std::string addr) {
    std::string msg = addr + "us";
    write(msg.data());
    process_response();
}

void elliptec::optimize_motors(std::string addr) {
    std::string msg = addr + "om";
    write(msg.data());
    bserial->setTimeout(boost::posix_time::seconds(0));
    process_response();
    bserial->setTimeout(boost::posix_time::seconds(_ser_timeout));
    //reply with GS (while busy) 0 when done
}

void elliptec::clean_mechanics(std::string addr) {
    std::string msg = addr + "cm";
    write(msg.data());
    bserial->setTimeout(boost::posix_time::seconds(0));
    process_response();
    bserial->setTimeout(boost::posix_time::seconds(_ser_timeout));
    //reply with GS (while busy) 0 when done
}

void elliptec::stop_clean(std::string addr) {
    std::string msg = addr + "st";
    write(msg.data());
    process_response();
    //reply with GS (while busy) 0 when done
}

void elliptec::change_address(std::string addr, std::string newaddr) {
    bool found = false;
    for (auto dev: devices) {
        if (dev.address == newaddr) {
            std::cout << "Error: new address " << newaddr << " already in use" << std::endl;
            found = true;
        }
    }
    if (!found) {
        std::cout << "in change_address" << std::endl;
        std::string msg = addr + "ca" + newaddr;
        write(msg.data());
        process_response();
        save_userdata(newaddr);
        for (auto &dev:devices) { 
            if (dev.address==addr) {
                dev.address=newaddr;
            }
        }
        for (auto &dev: devices) {
            print_dev_info(dev);
        }
        for (size_t i=0; i< mids.size(); ++i) {
            if (mids.at(i) == addr) {
                mids.at(i) = newaddr;
            }
        }
    }
}

void elliptec::get_status(std::string addr) {
    std::string msg = addr + "gs";
    write(msg.data());
    process_response();
    //reply with GS
}

void elliptec::energize_motor(std::string addr, double freq_hz){
    if (!devispiezo(addr)) {
        throw std::invalid_argument("only piezo ELL5 can be energized");
    }
    if ((freq_hz < 230) || (freq_hz > 2000000)) {
        throw std::invalid_argument("Frequency needs to be between 23Hz and 2MHz");
    }
    uint32_t period = std::llround(14740000/freq_hz);
    
    std::string msg = addr + "e1" + step2hex(period,4);
    write(msg.data());
    process_response();
    //reply with GS
}

void elliptec::halt_motor(std::string addr) {
    if (!devispiezo(addr)) {
        throw std::invalid_argument("only piezo ELL5 can halt");
    }
    std::string msg = addr + "h1";
    write(msg.data());
    process_response();
    //reply with GS
}


/*****************************************
 *
 * High levels
 *
 *****************************************/
void elliptec::search_freq(std::string addr) {
    auto dev = devinfo_at_addr(addr);
    if (dev.has_value()){
        if (devintype("indexed", dev.value().type)) {
            search_motor_freq(addr, 1);
        } else if (devintype("linrot", dev.value().type)) {
            search_motor_freq(addr, 1);
            search_motor_freq(addr, 2);
        }
    } else {
        std::cout << "device with address " << addr << " not in connected device list" << std::endl;
    }
}

void elliptec::command_moveboth(int hwp_mnum, int qwp_mnum, double hwpang, double qwpang){
    move_absolute(int2addr(hwp_mnum), std::fmod(hwpang, 360));
    move_absolute(int2addr(qwp_mnum), std::fmod(qwpang, 360));
}

void elliptec::command_movethree(int hwp_mnum, int qwp_mnum, int qwp2_mnum, double hwpang, double qwpang, double qwp2ang){
    move_absolute(int2addr(hwp_mnum), std::fmod(hwpang, 360));
    move_absolute(int2addr(qwp_mnum), std::fmod(qwpang, 360));
    move_absolute(int2addr(qwp2_mnum), std::fmod(qwp2ang, 360));
}

