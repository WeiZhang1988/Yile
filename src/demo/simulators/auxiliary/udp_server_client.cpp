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
#include "udp_server_client.hpp"  

void UDP_Server::get_request(double *recv_buf, std::size_t sz) {
    m_recved_num = recvfrom(m_sock_fd, recv_buf, sz, 0, (struct sockaddr *)&m_addr_client, (socklen_t *)&m_len);
    if(m_recved_num < 0) {  
		perror("recvfrom error: ...");  
		//exit(1);  
	} 
}

void UDP_Server::respond(double *send_buf, std::size_t sz) {
    std::cout<<"sending response ... "<<std::endl;
    m_sent_num = sendto(m_sock_fd, send_buf, sz, 0, (struct sockaddr *)&m_addr_client, m_len);  	
    if(m_sent_num < 0)  {  
        perror("respond error: ...");  
        //exit(1);  
    } 
}

void UDP_Client::request(double *send_buf, std::size_t sz) {
    std::cout<<"sending request ... "<<std::endl;
    m_sent_num = sendto(m_sock_fd, send_buf, sz, 0, (struct sockaddr *)&m_addr_serv, m_len);  	
    if(m_sent_num < 0)  {  
        perror("respond error: ...");  
        //exit(1);  
    } 
}

void UDP_Client::get_response(double *recv_buf, std::size_t sz) {
    std::cout<<"getting response ... "<<std::endl;
    m_recved_num = recvfrom(m_sock_fd, recv_buf, sz, 0, (struct sockaddr *)&m_addr_serv, (socklen_t *)&m_len);
    if(m_recved_num < 0) {  
		perror("recvfrom error: ...");  
		//exit(1);  
	} 
}
