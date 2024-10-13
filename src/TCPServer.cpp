#include "TCPServer.hpp"

#include "google/protobuf/timestamp.pb.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <iostream>
#include <cstring>



TCPServer::TCPServer(int port) : port(port), server_fd(-1), addrlen(sizeof(address)) {
    ekf = std::make_shared<BearingEKF>();
    initServer();
}

TCPServer::~TCPServer() {
    cleanup();
}

void TCPServer::initServer() {
    // Create socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("Socket failed");
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    // Bind the socket to the port
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("Bind failed");
        exit(EXIT_FAILURE);
    }

    std::cout << "Server is listening on port " << port << std::endl;

    // Listen for incoming connections
    if (listen(server_fd, 3) < 0) {
        perror("Listen failed");
        exit(EXIT_FAILURE);
    }
}

bool TCPServer::registerClient() {
    std::cout << "Waiting for a client to connect..." << std::endl;

    // Accept incoming connection
    if ((client_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        perror("Accept failed");
        return false;
    }

    std::cout << "Client connected!" << std::endl;
    return true;
}

// This method handles the communication (reading/writing) with the connected client
void TCPServer::handleCommunication() {
    char buffer[1024] = {0};
    int valread;

    // Read data from the client
    bool freshBearing = false;
    bool freshPosition = false;
    while((valread = read(client_socket, buffer, sizeof(buffer))) > 0) {
        
        SensorMessage sensor_msg;
        if (sensor_msg.ParseFromArray(buffer, valread)) {
            switch (sensor_msg.message_type_case()) {
                case SensorMessage::kBearing: {
                    const BearingMessage& bearing_msg = sensor_msg.bearing();
                    ekf->setBearing(bearing_msg.bearing());
                    freshBearing = true;
                    // std::cout << "Timestamp (sec): " << bearing_msg.timestamp().seconds() << std::endl;
                    // std::cout << "Timestamp (nsec): " << bearing_msg.timestamp().nanos() << std::endl;
                    // std::cout << "Bearing: " << bearing_msg.bearing() << std::endl;
                    break;
                }
                case SensorMessage::kPosition: {
                    const PositionMessage& position_msg = sensor_msg.position();
                    ekf->setSensorPosition(Eigen::Vector2d(position_msg.x(), position_msg.y()));
                    freshPosition = true;
                    // std::cout << "Timestamp (sec): " << position_msg.timestamp().seconds() << std::endl;
                    // std::cout << "Timestamp (nsec): " << position_msg.timestamp().nanos() << std::endl;
                    // std::cout << "x: " << position_msg.x() << std::endl;
                    // std::cout << "y: " << position_msg.y() << std::endl;
                    break;
                }
                case SensorMessage::kPose: {
                    std::cout << "Pose message received!" << std::endl;
                    const PoseMessage& pose_msg = sensor_msg.pose();
                    break;
                }
                default:
                    std::cout << "Unknown message type!" << std::endl;
            }
            
        } else {
            std::cout << "Failed to parse bearing message!" << std::endl;
        }
        if (freshBearing && freshPosition) {
            ekf->predict();
            ekf->update();
            // Send a response to the client
            Eigen::Vector2d targetPosition = ekf->getTargetPosition();
            std::cout << "Target position: " << targetPosition.transpose() << std::endl << std::endl;
            PositionMessage targetPositionMsg;
            targetPositionMsg.set_x(targetPosition(0));
            targetPositionMsg.set_y(targetPosition(1));
            SensorMessage response_msg;
            response_msg.set_allocated_position(new PositionMessage(targetPositionMsg));
            std::string response;
            response_msg.SerializeToString(&response);
            send(client_socket, response.c_str(), response.size(), 0);
            freshBearing = false;
            freshPosition = false;
        }
        

        // Clear the buffer
        memset(buffer, 0, sizeof(buffer));
    }

    // Close the client socket after communication
    close(client_socket);
}

void TCPServer::cleanup() {
    if (server_fd != -1) {
        close(server_fd);
    }
}



