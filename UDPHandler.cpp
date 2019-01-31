#include "UDPHandler.h"

UDPHandler::UDPHandler(std::string ip, int port)
	: ip{ip}, port{port}
{
	socket.open(ip::udp::v4());
	receive_endpoint = ip::udp::endpoint(ip::udp::v4(), 9001);
	send_endpoint = ip::udp::endpoint(ip::address::from_string(ip), 9000);
	socket.bind(receive_endpoint, error);
	service.run(error);

	receiveThread = std::thread(&UDPHandler::receive, this);
}

void UDPHandler::send(std::string message)
{
	//socket.send(buffer(message.c_str(), bufferSize), 0, error);
	socket.send_to(buffer(message.c_str(), bufferSize), send_endpoint, 0, error);
}

void UDPHandler::receive()
{
	while (true)
	{
		socket.receive(buffer(receivedMessage, bufferSize), 0, error);
	}
}

void UDPHandler::close()
{
	socket.close();
}

std::string UDPHandler::getMessage()
{
	return std::string(receivedMessage);
}

std::string UDPHandler::getIP()
{
	return ip;
}

int UDPHandler::getPort()
{
	return port;
}
