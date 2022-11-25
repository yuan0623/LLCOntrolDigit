//
// Created by yuan on 6/3/21.
//

#ifndef DIGIT_KF_DIGIT_JSON_HELPER_H
#define DIGIT_KF_DIGIT_JSON_HELPER_H
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <nlohmann/json.hpp>

class JsonMessageCommunication{
public:

    JsonMessageCommunication(const std::string host, const std::string port);

    std::vector<std::pair<int,bool>> contact_detection();
    //std::vector<s11n_example::stock> get_stocks();
    double simulation_rate();
private:
    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> *ws_;
    std::string host_;
    std::string port_;
};

#endif //DIGIT_KF_DIGIT_JSON_HELPER_H
