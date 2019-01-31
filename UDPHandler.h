#pragma once
#include <boost/asio.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <thread>

using namespace boost::asio;

class UDPHandler {
private:
	std::string ip;
	int port;

	io_service service;
	ip::udp::socket socket{ service };
	ip::udp::endpoint remote_endpoint;
	boost::system::error_code error;

	static const int bufferSize{ 128 };

	char receivedMessage[bufferSize];
	std::thread receiveThread;

	void receive();

public:
	UDPHandler(std::string ip, int port);
	void send(std::string message);
	void close();
	std::string getMessage();
	std::string getIP();
	int getPort();
};
