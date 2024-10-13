#ifndef TCPSERVER_HPP
#define TCPSERVER_HPP

#include <netinet/in.h>
#include "message.pb.h"
#include "BearingEKF.hpp"
#include <Eigen/Dense>
#include <cmath>


class TCPServer {
public:
    TCPServer(int port);
    ~TCPServer();
    
    bool registerClient();
    void handleCommunication();
    int client_socket;
    std::shared_ptr<BearingEKF> ekf;

private:
    
    int server_fd;
    int port;
    struct sockaddr_in address;
    int addrlen;

    void initServer();
    void cleanup();
};





#endif  // TCPSERVER_HPP