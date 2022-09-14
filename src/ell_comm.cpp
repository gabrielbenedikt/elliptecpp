#include "ell.h"

/*****************************************
 *
 * Serial
 *
 *****************************************/
std::string elliptec::read()
{
    std::string response = bserial->readStringUntil("\r\n");
    std::cout << "got response " << response << std::endl;
    return response;
}

void elliptec::write(const std::string &data)
{
    bserial->writeString(data);
}

std::string elliptec::query(const std::string &data) {
    bserial->writeString(data);
    std::string response = bserial->readStringUntil("\r\n");
    std::cout << "got response " << response << std::endl;
    return response;
}

void elliptec::close()
{
    if (bserial->isOpen())
        bserial->close();
}

bool elliptec::isopen()
{
    return bserial->isOpen();
}

void elliptec::open(std::string port) {
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
    }
}
