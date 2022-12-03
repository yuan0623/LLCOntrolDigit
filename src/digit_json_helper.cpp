//
// Created by yuan on 6/3/21.
//
#include "digit_json_helper.h"

JsonMessageCommunication::JsonMessageCommunication(std::string host, const std::string port) {

    host_ = host;
    port_ = port;

    auto const results = resolver.resolve(host, port);

    // Make the connection on the IP address we get from a lookup
    auto ep = boost::asio::connect(ws.next_layer(), results);

    host += ':' + std::to_string(ep.port());

    // Set a decorator to change the User-Agent of the handshake
    ws.set_option(boost::beast::websocket::stream_base::decorator(
            [](boost::beast::websocket::request_type& req)
            {
                req.set(boost::beast::http::field::user_agent,
                        std::string(BOOST_BEAST_VERSION_STRING) +
                        " websocket-client-coro");
//                    req.set("protocol", "json-v1-agility");

                req.set(boost::beast::http::field::sec_websocket_protocol, "json-v1-agility");
            }));


    ws.handshake(host_, "/");

}

std::vector<std::pair<int,bool>> JsonMessageCommunication::contact_detection(){

    std::string text = "[\"get-robot-info\",{}]";
    ws.write(boost::asio::buffer(std::string(text)));

    // This buffer will hold the incoming message
    boost::beast::flat_buffer buffer;

    // Read a message into our buffer
    ws.read(buffer);

    std::string data_str = boost::beast::buffers_to_string(buffer.cdata());
    data_str.replace(0,16,"");
    data_str.erase(data_str.size() - 1);
    std::cout<<"here is the data from robot"<<std::endl;
    std::cout<<data_str<<std::endl;

    nlohmann::json data_json = nlohmann::json::parse(data_str);

    std::vector<std::pair<int,bool> > contacts;
    contacts.push_back(std::pair<int,bool> (0, data_json["left-foot-in-contact"]));
    contacts.push_back(std::pair<int,bool> (1, data_json["right-foot-in-contact"]));
    return contacts;

}
