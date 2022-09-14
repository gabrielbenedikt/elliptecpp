#ifndef ELLIPTEC_H
#define ELLIPTEC_H

/*! \file */

//#include "defines.h"
#include "boost_serial.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <string>
#include <vector>

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
    elliptec(const std::string devname, const std::vector<uint8_t> inmids, const bool dohome = true, const bool freqsearch = true);
    ~elliptec();

    //serial
    void open(std::string port);
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
    
    void print_addr_info(std::string addr);
    void cr();
    void lf();
    void command_moveboth(int hwp_mnum, int qwp_mnum, double hwpang, double qwpang); //!TODO: remove
    void command_movethree(int hwp_mnum, int qwp_mnum, int qwp2_mnum, double hwpang, double qwpang, double qwp2ang); //!TODO: remove

private:
    bool _dofreqsearch;
    bool _dohome;
    std::vector<uint8_t> _inmids;
    std::string _devname;
    double _current_pos;

    
    // Direction constants
    static const uint8_t CW = 0;
    static const uint8_t CCW = 1;

    // Acceptable accuracy
    static constexpr double DEGERR = 0.1;
    static constexpr double MMERR = 0.05;

    // serial
    std::string query(const std::string &data);
    std::unique_ptr<Boost_serial> bserial;
    std::string read();
    void write(const std::string &data);
    uint16_t _ser_timeout;

    std::unordered_map<std::string, std::vector<uint8_t>> devtype;

    std::vector<std::string> mids;      //!< motor ids
    std::vector<ell_device> devices;

    ell_response process_response(std::string response = "");
    uint8_t parsestatus(std::string msg);
    std::string err2string(uint8_t code);
    
    void handle_devinfo(ell_device dev);
    void print_dev_info(ell_device dev);
    

    void search_motor_freq(std::string addr, uint8_t motor_num);
    
    bool devintype(std::string type, uint8_t id);
    bool devislinrot(std::string addr);
    bool devislinear(std::string addr);
    bool devisrotary(std::string addr);
    bool devispaddle(std::string addr);
    bool devispiezo(std::string addr);
    std::optional<ell_device> devinfo_at_addr(std::string addr);
    int64_t deg2step(std::string addr, double deg);
    int64_t mm2step(std::string addr, double mm);
    double step2deg(std::string addr, int64_t step);
    double step2mm(std::string addr, int64_t step);
    std::string step2hex(int64_t step, uint8_t width = 8);
    int64_t hex2step(std::string hex);
    std::string ll2hex(int64_t i);
    std::string us2hex(uint16_t i);
    std::string uc2hex(uint8_t i);
    std::string int2addr(uint8_t id);
};

#endif // ELLIPTEC_H
