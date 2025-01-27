#include "udp_server.hpp"

using namespace server;

UDPServer::UDPServer(std::string server_ip, unsigned long server_port, std::string client_ip, unsigned long client_port):
server_ip_(server_ip),
server_port_(server_port),
client_ip_(client_ip),
client_port_(client_port)
{
    
}

UDPServer::~UDPServer()
{
	stop();
}

void UDPServer::start()
{	
	server_started_ = true;

	if ( (sockfd_ = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
		std::cerr << "Ошибка при создании сокета" << std::endl; 
		return; 
	} 
	
	memset(&servaddr_, 0, sizeof(servaddr_)); 
	memset(&cliaddr_, 0, sizeof(cliaddr_)); 
	
	servaddr_.sin_family = AF_INET; // IPv4 
	// servaddr.sin_addr.s_addr = INADDR_ANY; 
	inet_pton(AF_INET, server_ip_.c_str(), &servaddr_.sin_addr);
	servaddr_.sin_port = htons(server_port_); 

	cliaddr_.sin_family = AF_INET; // IPv4 
	// servaddr.sin_addr.s_addr = INADDR_ANY; 
	inet_pton(AF_INET, client_ip_.c_str(), &cliaddr_.sin_addr);
	cliaddr_.sin_port = htons(client_port_); 
	
	// Bind the socket with the server address 
	if ( bind(sockfd_, (const struct sockaddr *)&servaddr_, sizeof(servaddr_)) < 0 ) 
	{ 
		std::cerr << "Ошибка привязки сокета" << std::endl;
		close(sockfd_);
		sockfd_ = -1; 
		return;
	} 

	len_ = sizeof(cliaddr_); //len is value/result

    receiv_ = std::jthread(&UDPServer::run_receive, this);
    transmit_ = std::jthread(&UDPServer::run_transmit, this);
}

void UDPServer::stop()
{	
	if (server_started_) 
	{
		server_started_ = false;
		if ((receiv_.joinable())&&(transmit_.joinable())) {
			receiv_.join();
			transmit_.join();
		}
		closeSocket();
	}
}

void UDPServer::run_receive()
{
    while (server_started_)
	{
		n_ = recvfrom(sockfd_, (char *)buffer_, 1024, 
					MSG_WAITALL, ( struct sockaddr *) &cliaddr_, 
					&len_); 
		buffer_[n_] = '\0';

		std::cout << buffer_ << std::endl;

		// -----------------------------------------------------------------------
        // ---------------------------------------------- processing received data
		// -----------------------------------------------------------------------

		// std::string str(buffer_);

		// try {
        //     double d =  std::stod(str);
		// 	angle_ = int(d*10000);

		// 	// mtx_.lock();
		// 	// messageQueue_.push(d);
		// 	// mtx_.unlock();

		// } catch (const std::invalid_argument&) {
		// 	std::cerr << "Некорректное сообщение: не число: " << str << std::endl;
		// } catch (const std::out_of_range&) {
		// 	std::cerr << "Число вне допустимого диапазона: " << str << std::endl;
        // } 
	}
}

void UDPServer::run_transmit()
{
    const char* ch = "A";
    while (server_started_)
	{
		if (msg_ready_)
		{
			sendto(sockfd_, ch, strlen(ch), MSG_CONFIRM, (const struct sockaddr *) &cliaddr_, sizeof(cliaddr_)); 
			msg_ready_ = false;
		}
	}
}

void UDPServer::closeSocket() 
{
	if (sockfd_ >= 0) {
		close(sockfd_);
		sockfd_ = -1;
	}
}

bool UDPServer::getMsg()
{

	// mtx_.lock();
	// if (messageQueue_.empty()) {
	// 	mtx_.unlock();
	// 	return false;
	// }
	// number = messageQueue_.front();
	// messageQueue_.pop();
	// mtx_.unlock();

	// number = double(angle_)/10000;

	return true;
}

bool UDPServer::setMsg()
{
	msg_ready_ = true;

	return true;
}