#ifndef ELLIPTEC_H
#define ELLIPTEC_H

#include "boost_serial.h"

#include <boost/utility.hpp>
#include <boost/asio.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <string>

class timeout_exception;
class Boost_serial;

struct ell_device {
    std::string address;    //address of device on controller
    uint16_t type;          //type of device
    uint64_t serial;        //serial number
    uint16_t year;          //manufacturing year
    uint8_t fw;             //firmware revision
    uint8_t hw;             //firmware revision
    uint32_t travel;        //units of travel range
    uint64_t pulses;        //pulses per unit
};

struct ell_response {
    uint8_t address = 0;
    std::string type = "";
    std::string data = "";
};

enum ell_errors {
    OK = 0,
    COMM_TIMEOUT = 1,
    MECH_TIMEOUT = 2,
    COMMAND_ERR = 3,
    VAL_OUT_OF_RANGE = 4,
    MOD_ISOLATED = 5,
    MOD_OUT_OF_ISOL = 6,
    INIT_ERROR = 7,
    THERMAL_ERROR = 8,
    BUSY = 9,
    SENSOR_ERROR = 10,
    MOTOR_ERROR = 11,
    OUT_OF_RANGE = 12,
    OVER_CURRENT = 13,
    GENERAL_ERROR = 14
};

const std::vector<std::string> error_msgs = {
    "OK, no error",
    "Communication time out",
    "Mechanical time out",
    "Command error or not supported",
    "Value out of range",
    "Module isolated",
    "Module out of isolation",
    "Initializing error",
    "Thermal error",
    "Busy",
    "Sensor Error (May appear during self test. If code persists there is an error)",
    "Motor Error (May appear during self test. If code persists there is an error)",
    "Out of Range (e.g. stage has been instructed to move beyond its travel range)",
    "Over Current error",
    //14-255 Reserved
};

class elliptec {

public:
    elliptec(std::string devname = "", std::vector<uint8_t> inmids = std::vector<uint8_t>(0), bool dohome = true, bool freqsearch = true);
    ~elliptec();

    //serial
    void open(std::string port, bool dohome, bool freqsearch);
    void close();
    bool isopen();

    //low level
    void get_info(std::string addr);
    void get_status(std::string addr);
    void save_userdata(std::string addr);
    void change_address(std::string addr, std::string newaddr);
    void get_motor_info(std::string addr, uint8_t motor_num);
    void set_motor_freq(std::string addr, std::string dir, uint8_t motor_num, uint16_t freq, bool factory_reset=false);
    void scan_motor_current_curve(std::string addr, uint8_t motor_num);
    void search_freq(std::string addr);
    std::vector<std::pair<uint8_t, uint8_t>> get_motor_current_curve(std::string addr, uint8_t motor_num);
    void isolate_device(std::string addr, uint8_t minutes);
    void home(std::string addr, std::string dir = "0");
    void paddle_home(std::string addr, uint8_t paddle_num);
    void move_absolute(std::string addr, double pos);
    void move_relative(std::string addr, double pos);
    double get_home_offset(std::string addr);
    void set_home_offset(std::string addr, double offset);
    double get_jogstep_size(std::string addr);
    void set_jogstep_size(std::string addr, double offset);
    void move_fwd(std::string addr);
    void move_bwd(std::string addr);
    void stop(std::string addr);
    void get_position(std::string addr);
    uint8_t get_velocity(std::string addr);
    void set_velocity(std::string addr, uint8_t percent);
    void groupaddress(std::string addr, std::string groupaddr);
    void paddle_drivetime(std::string addr, uint8_t padnum, uint16_t ms, std::string direction="fwd");
    void paddle_moveabsolute(std::string addr, uint8_t padnum, double deg);
    void paddle_moverelative(std::string addr, uint8_t padnum, double deg);
    void optimize_motors(std::string addr);
    void clean_mechanics(std::string addr);
    void stop_clean(std::string addr);
    void energize_motor(std::string addr, double freq);
    void halt_motor(std::string addr);
    
    void command_moveboth(int hwp_mnum, int qwp_mnum, double hwpang, double qwpang); //!TODO: remove
    void command_movethree(int hwp_mnum, int qwp_mnum, int qwp2_mnum, double hwpang, double qwpang, double qwp2ang); //!TODO: remove
};

#endif // ELLIPTEC_H
