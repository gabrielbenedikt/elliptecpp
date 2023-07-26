#include "ell_interactive.h"

int main(int argc, char **argv) {
    std::string devname = "";
    std::vector<uint> mnum = std::vector<uint>(0);
    
    /*
     * parse arguments
     */
    try {
        bpo::options_description args("Arguments");
        args.add_options()
            ("help,h", "prints this message")
            ("device-path,d", bpo::value<std::string>(), "elliptec controller device path")
            ("motor-id,i", bpo::value<std::vector<uint>>()->multitoken(), "motor ids connected to controller")
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
            mnum = (std::vector<uint>)vm["motor-id"].as< std::vector<uint >>();
        } else {
            std::cout << "no motor id specified.\n";
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
    std::vector<uint8_t> mnumvec;
    for (auto mn: mnum) {
        mnumvec.push_back(mn);
    }
    
    elliptec dev = elliptec(devname, mnumvec, true, true);
    
    /*
     * command prompt
     */
    const std::string prompt_history_file = "promt_history.txt";
    linenoise::SetCompletionCallback([](const char* editBuffer, std::vector<std::string>& completions) {
    if (editBuffer[0] == 'm') {
        completions.push_back("moveabsolute");
        completions.push_back("moverelative");
        completions.push_back("moveabsolute");
        completions.push_back("mr");
    }});
    linenoise::SetHistoryMaxLen(256);
    linenoise::LoadHistory(prompt_history_file.c_str());

    while (true) {
        // Read line
        std::string line;
        std::vector<std::string> linevec;
        std::string cmd;
        std::string id;
        std::vector<std::string> args;
        auto quit = linenoise::Readline("> ", line);

        if (quit) {
            break;
        }

        linevec = split(line);
        
        if (linevec.empty()==true) {
            continue;
        } 
        if (linevec.size()>0) {
            cmd = linevec.at(0);
            if (linevec.size()>1) {
                id = linevec.at(1);
                if (linevec.size()>2) {
                    args = {linevec.begin()+2, linevec.end()};
                }
            }
        }
    
        try {
            if (!cmd.compare("help")) {
                std::cout << "known commands: \n";
                std::cout << "|short|command       paramters       \n";
                std::cout << "|-----|------------------------------\n";
                std::cout << "| ma  |moveabsolute  <id> <angle/mm> \n";
                std::cout << "| mr  |moverelative  <id> <angle/mm> \n";
                std::cout << "| mf  |moveforwards  <id>            \n";
                std::cout << "| mb  |movebackwards <id>            \n";
                std::cout << "|     |stop          <id>            \n";
                std::cout << "| gv  |getvelocity   <id>            \n";
                std::cout << "| sv  |setvelocity   <id> <percent>  \n";
                std::cout << "| gj  |getjogsize    <id>            \n";
                std::cout << "| sj  |setjogsize    <id> <angle/mm> \n";
                std::cout << "| po  |getpos        <id>            \n";
                std::cout << "| i   |info          <id>            \n";
                std::cout << "|     |status        <id>            \n";
                std::cout << "|     |save          <id>            \n";
                std::cout << "|     |change_id     <oldid> <newid> \n";
                std::cout << "|     |search_freq   <id>            \n";
                std::cout << "| ho  |home          <id>            \n";
                std::cout << "|     |clean         <id>            \n";
                std::cout << "|     |optimize      <id>            \n";
                std::cout << "|  q  |quit                          \n";
            } else if ((!cmd.compare("moveabsolute")) || (!cmd.compare("ma"))) {
                dev.move_absolute(id, std::stod(args.at(0)));
            } else if ((!cmd.compare("moverelative")) || (!cmd.compare("mr"))) {
                dev.move_relative(id, std::stod(args.at(0)));
            } else if ((!cmd.compare("info")) || (!cmd.compare("i"))) {
                dev.get_info(id);
                dev.print_addr_info(id);
            } else if (!cmd.compare("status")) {
                dev.get_status(id);
            } else if (!cmd.compare("save")) {
                dev.save_userdata(id);
            } else if ((!cmd.compare("home")) || (!cmd.compare("ho"))) {
                dev.home(id);
            } else if ((!cmd.compare("getpos")) || (!cmd.compare("po"))) {
                dev.get_position(id);
            } else if (!cmd.compare("stop")) {
                dev.stop(id);
            } else if ((!cmd.compare("getvelocity")) || (!cmd.compare("gv"))) {
                dev.get_velocity(id);
            } else if ((!cmd.compare("setvelocity")) || (!cmd.compare("sv"))) {
                unsigned long arg = std::stoul(args.at(0));
                uint8_t percent = 0;
                if (arg<100) {
                    percent = arg;
                } else {
                    percent=100;
                }
                dev.set_velocity(id,percent);
            } else if ((!cmd.compare("moveforwards")) || (!cmd.compare("mf"))) {
                dev.move_fwd(id);
            } else if ((!cmd.compare("movebackwards")) || (!cmd.compare("mb"))) {
                dev.move_bwd(id);
            } else if ((!cmd.compare("getjogsize")) || (!cmd.compare("gj"))) {
                dev.get_jogstep_size(id);
            } else if ((!cmd.compare("setjogsize")) || (!cmd.compare("sj"))) {
                dev.set_jogstep_size(id, std::stod(args.at(0)));
            } else if (!cmd.compare("change_id")) {
                if (args.empty()) {
                    std::cout << "need to specify new address" << std::endl;
                    continue;
                } else {
                    dev.change_address(id, args.at(0));
                }
            } else if (!cmd.compare("clean")) {
                dev.clean_mechanics(id);
            }else if (!cmd.compare("stopclean")) {
                dev.stop_clean(id);
            } else if (!cmd.compare("search_freq")) {
                dev.search_freq(id);
            } else if (!cmd.compare("optimize")) {
                dev.optimize_motors(id);
            } else if ((!cmd.compare("quit")) || (!cmd.compare("q"))) {
                break;
            } 
        } catch (const std::exception& e) {
            std::cout << e.what() << std::endl;
            continue;
        }
        linenoise::AddHistory(line.c_str());
        linenoise::SaveHistory(prompt_history_file.c_str());
    }
    
    dev.close();
    
    return 0;
}

std::vector<std::string> split(const std::string s) {
    std::stringstream ss(s);
    std::istream_iterator<std::string> begin(ss);
    std::istream_iterator<std::string> end;
    std::vector<std::string> ret(begin, end);
    return ret;
}
