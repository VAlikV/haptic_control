#ifndef UDP_SERVER
#define UDP_SERVER

#include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

#include <thread>
#include <mutex>
#include <functional>
#include <atomic>

namespace server
{
    class UDPServer
    {
    private:
        std::string server_ip_;         // own server ip
        unsigned long server_port_;     // own server port

        std::string client_ip_;         // client ip
        unsigned long client_port_;     // client port

        std::atomic<bool> server_started_ = false;   // is receive started
        std::atomic<bool> msg_ready_ = false;

        int sockfd_; 
        char buffer_[1024]; 

        struct sockaddr_in servaddr_;
        struct sockaddr_in cliaddr_;

        std::jthread receiv_;           // thread for receiving 
        std::jthread transmit_;         // thread for transmit

        socklen_t len_;
	    int n_; 
        
		// -----------------------------------------------------------------------
        // ---------------------------------------------------- type received data
		// -----------------------------------------------------------------------

        // std::queue<double> messageQueue_;
        // std::mutex mtx_; 

        // std::atomic<int> angle_;

        // -----------------------------------------------------------------------
        // ------------------------------------------------- type transmitted data
		// -----------------------------------------------------------------------

        
        void closeSocket();
        void run_receive();
        void run_transmit();

    public: 
        UDPServer(std::string server_ip, unsigned long server_port, std::string client_ip = INADDR_ANY, unsigned long client_port = 8080);
        ~UDPServer();

        void start();
        void stop();

        bool getMsg();
        bool setMsg();
    };
}

#endif