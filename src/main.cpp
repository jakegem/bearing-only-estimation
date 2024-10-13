#include "TCPServer.hpp"

int main() {
    int port = 8080;
    
    // Create and initialize the server
    TCPServer server(port);

    // Server loop
    while (true) {
        // Register (accept) a new client
        int client_socket = server.registerClient();
        
        // If client successfully connected, handle communication
        if (server.client_socket != -1) {
            server.handleCommunication();
        }
    }

    return 0;
}
