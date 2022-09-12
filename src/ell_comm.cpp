#include "ell.h"

/*****************************************
 *
 * Serial
 *
 *****************************************/
std::string elliptec::read()
{
    std::cout << "in read" << std::endl;
    std::string response = bserial->readStringUntil("\r\n");
    std::cout << "got response " << response << std::endl;
    return response;
}

void elliptec::write(const std::string &data)
{
    std::cout << "in write" << std::endl;
    std::cout << "send " << data << std::endl;
    bserial->writeString(data);
}

std::string elliptec::query(const std::string &data) {
    std::cout << "in query" << std::endl;
    std::cout << "send " << data << std::endl;
    bserial->writeString(data);
    std::string response = bserial->readStringUntil("\r\n");
    std::cout << "got response " << response << std::endl;
    return response;
}

void elliptec::close()
{
    std::cout << "in close" << std::endl;
    if (bserial->isOpen())
        bserial->close();
}

bool elliptec::isopen()
{
    std::cout << "in isopen" << std::endl;
    return bserial->isOpen();
}

void elliptec::open(std::string port, bool dohome, bool freqsearch) {
    std::cout << "in open" << std::endl;
    if (!bserial->isOpen()) {
        try {
             bserial->open(port, 9600,
                           boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none),
                           boost::asio::serial_port_base::character_size(8),
                           boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none),
                           boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
         }  catch (std::exception & ex) {
             std::cout << ex.what() << std::endl;
         }

        for (std::string id : mids) {
            get_info(id);
            if (freqsearch) {
                search_freq(id);
                save_userdata(id);
            }
            if (dohome) {
                home(id);
            }
        }
    }
}
