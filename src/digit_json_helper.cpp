//
// Created by yuan on 6/3/21.
//
#include "digit_json_helper.h"

JsonMessageCommunication::JsonMessageCommunication(std::string host, const std::string port) {

    host_ = host;
    port_ = port;
    // The io_context is required for all I/O
    boost::asio::io_context ioc;

    // These objects perform our I/O
    boost::asio::ip::tcp::resolver resolver{ioc};

    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws{ioc};

    //ws_ = ws;
    // Look up the domain name
    auto const results = resolver.resolve(host, port);

    // Make the connection on the IP address we get from a lookup
    auto ep = boost::asio::connect(ws.next_layer(), results);

    // Update the host_ string. This will provide the value of the
    // Host HTTP header during the WebSocket handshake.
    // See https://tools.ietf.org/html/rfc7230#section-5.4
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

    // Perform the websocket handshake
    ws.handshake(host, "/");

    ws_  = &ws;

}

std::vector<std::pair<int,bool>> JsonMessageCommunication::contact_detection(){
    std::string host = host_;
    std::string port = port_;
    // Send the message
    boost::asio::io_context ioc;

    // These objects perform our I/O
    boost::asio::ip::tcp::resolver resolver{ioc};

    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws{ioc};

    //ws_ = ws;
    // Look up the domain name
    auto const results = resolver.resolve(host, port);

    // Make the connection on the IP address we get from a lookup
    auto ep = boost::asio::connect(ws.next_layer(), results);

    // Update the host_ string. This will provide the value of the
    // Host HTTP header during the WebSocket handshake.
    // See https://tools.ietf.org/html/rfc7230#section-5.4
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

    // Perform the websocket handshake
    ws.handshake(host, "/");
    std::string text = "[\"get-robot-status\",{}]";
    ws.write(boost::asio::buffer(std::string(text)));

    // This buffer will hold the incoming message
    boost::beast::flat_buffer buffer;

    // Read a message into our buffer
    ws.read(buffer);

    std::string data_str = boost::beast::buffers_to_string(buffer.cdata());
    data_str.replace(0,16,"");
    data_str.erase(data_str.size() - 1);
    //std::cout<<data_str<<std::endl;
    nlohmann::json data_json = nlohmann::json::parse(data_str);

    std::vector<std::pair<int,bool> > contacts;
    contacts.push_back(std::pair<int,bool> (0, data_json["left-foot-in-contact"]));
    contacts.push_back(std::pair<int,bool> (1, data_json["right-foot-in-contact"]));
    return contacts;

}

double JsonMessageCommunication::simulation_rate(){
    std::string host = host_;
    std::string port = port_;
    // Send the message
    boost::asio::io_context ioc;

    // These objects perform our I/O
    boost::asio::ip::tcp::resolver resolver{ioc};

    boost::beast::websocket::stream<boost::asio::ip::tcp::socket> ws{ioc};

    //ws_ = ws;
    // Look up the domain name
    auto const results = resolver.resolve(host, port);

    // Make the connection on the IP address we get from a lookup
    auto ep = boost::asio::connect(ws.next_layer(), results);

    // Update the host_ string. This will provide the value of the
    // Host HTTP header during the WebSocket handshake.
    // See https://tools.ietf.org/html/rfc7230#section-5.4
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

    // Perform the websocket handshake
    ws.handshake(host, "/");

    std::string text = "[\"get-robot-status\",{}]";
    ws.write(boost::asio::buffer(std::string(text)));

    // This buffer will hold the incoming message
    boost::beast::flat_buffer buffer;

    // Read a message into our buffer
    ws.read(buffer);

    std::string data_str = boost::beast::buffers_to_string(buffer.cdata());
    data_str.replace(0,16,"");
    data_str.erase(data_str.size() - 1);
    nlohmann::json data_json = nlohmann::json::parse(data_str);
    double simulation_rate = data_json["simulation-rate"];
    return simulation_rate;
}
