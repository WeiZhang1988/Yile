// =============================================================================
// PROJECT YILE - 
//
// Copyright (c) 2023
// All rights reserved.
//
// Use of this source code is governed by a GPL-3.0 license that can be found
// in the LICENSE file
//
// Author of this file	Wei ZHANG wei_zhang_1988@outlook.com
//
// =============================================================================
#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <arpa/inet.h>    
#include <errno.h>    
#include <stdlib.h> 
#include "yile.hpp"
using namespace Yile;

class UDP_Server {
public:
    UDP_Server(uint16_t port) : m_port(port) {
        std::cout<<"my port is "<<port<<std::endl;
        m_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if(m_sock_fd < 0) {  
            perror("socket");  
            exit(1);  
        } 
        memset(&m_addr_serv, 0, sizeof(struct sockaddr_in));
        m_addr_serv.sin_family = AF_INET;
        m_addr_serv.sin_port = htons(m_port);
        m_addr_serv.sin_addr.s_addr = htonl(INADDR_ANY); 
        m_len = sizeof(m_addr_serv); 
        if(bind(m_sock_fd, (struct sockaddr *)&m_addr_serv, sizeof(m_addr_serv)) < 0) {  
            perror("bind error:");  
            exit(1);  
        } 
    };
    ~UDP_Server() {
        close(m_sock_fd); 
    };

    void get_request (double *recv_buf, std::size_t sz);
    void respond (double *send_buf, std::size_t sz);


private:
    int m_sock_fd;
    struct sockaddr_in m_addr_serv; 
    struct sockaddr_in m_addr_client;
    uint16_t m_port;
    int m_len;
    double *m_send_buf;
    double *m_recv_buf;
    int m_recved_num;
	int m_sent_num;
};



/////////////////////////////////////////////////////////////////////////////////////
class UDP_Client {
public:
    UDP_Client(std::string ip, uint16_t port) : m_ip(ip), m_port(port) {
        std::cout<<"destination ip is "<<ip<<std::endl;
        std::cout<<"destination port is "<<port<<std::endl;
        m_sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if(m_sock_fd < 0) {  
            perror("socket");  
            exit(1);  
        } 
        memset(&m_addr_serv, 0, sizeof(m_addr_serv));
        m_addr_serv.sin_family = AF_INET;
        m_addr_serv.sin_port = htons(m_port);
        m_addr_serv.sin_addr.s_addr = inet_addr(m_ip.c_str()); 
        m_len = sizeof(m_addr_serv); 

    };
    ~UDP_Client() {
        close(m_sock_fd); 
    };

    void request (double *send_buf, std::size_t sz);
    void get_response(double *recv_buf, std::size_t sz);


private:
    int m_sock_fd;
    struct sockaddr_in m_addr_serv; 
    std::string m_ip;
    uint16_t m_port;
    int m_len;
    double *m_send_buf;
    double *m_recv_buf;
    int m_recved_num;
	int m_sent_num;
};