#include "ell_rot.h"

int main(int argc, char **argv) {
    std::string devname = "";
    uint mnum = 0;
    float angle = 0;
    
    /*
     * parse arguments
     */
    try {
        bpo::options_description args("Arguments");
        args.add_options()
            ("help,h", "prints this message")
            ("device-path,d", bpo::value<std::string>(), "elliptec controller device path")
            ("motor-id,i", bpo::value<uint>()->default_value(0), "motor idto rotate")
            ("angle,a", bpo::value<float>()->default_value(0), "angle to rotate to")
            ;
        
        bpo::options_description cmdline_options;
        cmdline_options.add(args);
        
        bpo::variables_map vm;
        store(bpo::command_line_parser(argc, argv).
              options(cmdline_options).run(), vm);
        notify(vm);
        
        if (vm.count("help")) {
            std::cout << "Usage: ./ell_rot -d devicepath -m motorid -a angle\n";
            std::cout << "Rotates elliptec motors.\n";
            std::cout << args << "\n";
            return 0;
        }

        if (vm.count("device-path")) {
            devname = vm["device-path"].as< std::string >();
        } else {
            std::cout << "no device specified.\n";
            return 1;
        }
        if (vm.count("motor-id")) {
            mnum = (uint)vm["motor-id"].as< uint >();
        } else {
            std::cout << "no motor id specified.\n";
            return 1;
        }
        
        if (vm.count("angle")) {
            angle = vm["angle"].as< float >();
        } else {
            std::cout << "no angle specified.\n";
            return 1;
        }
        
    }
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
        return 1;
    }
    std::vector<uint8_t> mnumvec(1,mnum);
    
    elliptec dev = elliptec(devname, mnumvec);
    
    dev.move_absolute(std::to_string(mnum), angle);
        
    dev.close();
    
    return 0;
}

