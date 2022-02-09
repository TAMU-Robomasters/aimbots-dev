/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED

#include "tcp_server.hpp"

#ifdef __linux__
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif  // __linux__

#include <atomic>
#include <iostream>
#include <stdexcept>

#include "json_messages.hpp"
using std::cerr;
/**
 * TCP Server class to allow MCB simulator to communicate with stuff.
 * I actually have no idea what's happening ~ Tenzin
 */
namespace tap
{
namespace communication
{
/**
 * TCPServer constructor. Runs a server on the given portnumber.
 * Also takes a function pointer. This function is left to the implementer
 * of a specific TCP server to define (it should take an int16_t parameter
 * for client file descriptor). Ideally mainLoop should use a non-blocking
 * loop as currently since threads detach its possible that if a client never
 * responds there could be threads that never close.
 */

TCPServer::TCPServer(int targetPortNumber)
#ifdef __linux__
    : socketOpened(false),
      clientConnected(false),
      mainClientDescriptor(-1),
      serverAddress(),
      portNumber(-1)
#endif  // __linux__
{
#ifdef __linux__
    // Do sockety stuff.
    listenFileDescriptor = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFileDescriptor < 0)
    {
        perror("TCPServer failed to open socket");
        throw std::runtime_error("SocketError");
    }

    int yes = 1;
    if (setsockopt(listenFileDescriptor, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)))
    {
        perror("TCPSever failed to set socket options");
        throw std::runtime_error("SocketOptionsError");
    }

    socketOpened = true;
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(targetPortNumber);
    serverAddress.sin_addr.s_addr = INADDR_ANY;

    int n = bind(
        listenFileDescriptor,
        reinterpret_cast<sockaddr*>(&serverAddress),
        sizeof(serverAddress));

    if (n < 0)
    {
        perror("TCPServer failed to bind socket");
        // Good reference for why this error occurs and what it means:
        // https://hea-www.harvard.edu/~fine/Tech/addrinuse.html
        std::cerr << "If you're seeing this error it means that the TCPServer "
                     "has failed to bind to a port too many times. If you're seeing this "
                     "message, something is probably really messed up."
                  << std::endl;
        throw std::runtime_error("BindError");
    }

    portNumber = targetPortNumber;

    listen(listenFileDescriptor, LISTEN_QUEUE_SIZE);
    std::cout << "TCPServer initialized on port: " << targetPortNumber << std::endl;
    std::cout << "call getConnection() to accept client" << std::endl;
#else
    UNUSED(targetPortNumber);
#endif  // __linux__
}

TCPServer::~TCPServer()
{
#ifdef __linux__
    close(listenFileDescriptor);
    close(mainClientDescriptor);
#endif  // __linux__
}

TCPServer* TCPServer::MainServer()
{
#if defined(ENV_UNIT_TESTS) || !defined(__linux__)
    return nullptr;
#else
    return &mainServer;
#endif
}

void TCPServer::getConnection()
{
#ifdef __linux__
    sockaddr_in clientAddress;
    socklen_t clientAddressLength = sizeof(clientAddress);
    mainClientDescriptor = accept(
        listenFileDescriptor,
        reinterpret_cast<sockaddr*>(&clientAddress),
        &clientAddressLength);
    cerr << "TCPServer: connection accepted" << std::endl;
#endif  // __linux__
}

void TCPServer::closeConnection()
{
#ifdef __linux__
    close(mainClientDescriptor);
    mainClientDescriptor = -1;
    std::cout << "TCPServer: closed connection with client, "
                 "use getConnection() to connect to a new one";
#endif  // __linux__
}

/**
 * Post: Returns the port number of this server.
 */
uint16_t TCPServer::getPortNumber()
{
#ifdef __linux__
    return this->portNumber;
#else
    return 0;
#endif  // __linux__
}

/**
 * Writes "messageLength" bytes of "message" to the mainClient
 */
void TCPServer::writeToClient(const char* message, int32_t messageLength)
{
#ifdef __linux__
    if (mainClientDescriptor < 0)
    {
        // Not necessarily an error if fileDescriptor still hasn't been opened
        // so we just don't write to anything and early return.
        cerr << "TCPServer: mainClientDescriptor not connected yet" << std::endl;
        return;
    }
    try
    {
        writeMessage(mainClientDescriptor, message, messageLength);
    }
    catch (std::runtime_error& e)
    {
        std::cerr << e.what() << std::endl;
    }
#else
    UNUSED(message);
    UNUSED(messageLength);
#endif  // __linux__
}

#ifdef __linux__
void readMessage(int16_t fileDescriptor, char* readBuffer, uint16_t messageLength)
{
    readBuffer[messageLength] = '\0';  // Null terminate the message
    uint16_t bytesRead = read(fileDescriptor, readBuffer, messageLength);
    while (bytesRead < messageLength)
    {
        int32_t n = read(fileDescriptor, readBuffer + bytesRead, messageLength - bytesRead);
        if (n < 0)
        {
            if (errno == EAGAIN or errno == EINTR)
            {
                continue;
            }
            else
            {
                perror("TCPServer failed to read from client");
                throw std::runtime_error("ReadingError");
            }
        }
        bytesRead += n;
    }
}

/**
 * Write to connected connectionFileDescriptor, ensures that all bytes are sent.
 * Throws runtime_error if irrecoverable error occurs.
 */
void writeMessage(int16_t fileDescriptor, const char* message, uint16_t bytes)
{
    uint32_t bytesWritten = 0;
    while (bytesWritten < bytes)
    {
        int32_t n = write(fileDescriptor, message + bytesWritten, bytes - bytesWritten);
        if (n < 0)
        {
            if (errno == EAGAIN or errno == EINTR)
            {
                continue;
            }
            else
            {
                perror("TCPServer failed to write");
                throw std::runtime_error("WriteError");
            }
        }
        bytesWritten += n;
    }
}

/**
 * Read the next 4 bytes as a big endian int32_t
 */
int32_t readInt32(int16_t fileDescriptor)
{
    char buffer[5];
    readMessage(fileDescriptor, buffer, 4);
    int32_t answer = 0;
    for (size_t i = 0; i < 4; i++)
    {
        answer = answer | buffer[i];
        answer <<= 8;
    }
    return answer;
}
#endif  // __linux__

// Only construct static singleton in actual sim, not in unit tests.
#ifndef ENV_UNIT_TESTS
// Definition of static variable. mainServer never created otherwise.
TCPServer TCPServer::mainServer(2001);
#endif

}  // namespace communication

}  // namespace tap

#endif  // PLATFORM_HOSTED
