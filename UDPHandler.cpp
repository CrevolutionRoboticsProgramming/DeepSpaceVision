#include "UDPHandler.h"

UDPHandler::UDPHandler(std::string ip, int sendPort, int receivePort)
	: ip{ip}, sendPort{sendPort}
{
	socket.open(ip::udp::v4(), error);
	send_endpoint = ip::udp::endpoint(ip::address::from_string(ip), sendPort);
	receive_endpoint = ip::udp::endpoint(ip::udp::v4(), receivePort);
	socket.bind(receive_endpoint, error);
	service.run(error);

	receiveThread = std::thread(&UDPHandler::receive, this);
}

void UDPHandler::send(std::string message)
{
	//socket.send(buffer(message.c_str(), bufferSize), 0, error);
	socket.send_to(buffer(message, bufferSize), send_endpoint, 0, error);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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

void UDPHandler::clearMessage()
{
	std::fill_n(receivedMessage, bufferSize, ' ');
}

std::string UDPHandler::getIP()
{
	return ip;
}

int UDPHandler::getSendPort()
{
	return sendPort;
}

int UDPHandler::getReceivePort()
{
	return receivePort;
}
