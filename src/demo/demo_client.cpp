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
#include <stdio.h>   
#include <string.h>   
#include <errno.h>   
#include <stdlib.h>   
#include <unistd.h>   
#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>   
#include <arpa/inet.h>  
#include <iostream>
#include "interfaces/interface_chassis_2ind_disk_fiala.hpp"
#include "components/common.hpp"
#include "simulators/auxiliary/udp_server_client.hpp"
#include <chrono>
#include <unistd.h>
#include "simulators/csv.hpp"

using namespace Yile;
   
#define DFLT_DEST_PORT 9000
#define DFLT_DEST_IP "127.0.0.1"
   
  
int main()  
{  
    std::string input_file = "data/inputs/pass14dof_reversedriving.csv";    

    std::shared_ptr<UDP_Client> udp_sptr_client = std::make_shared<UDP_Client>(DFLT_DEST_IP,DFLT_DEST_PORT);  
        
    int send_num;  
    int recv_num;  
    double send_buf[Int_Chassis_2Ind_Disk_Fiala::m_inputs_nums] = {
        eps,eps,eps,eps,  //1-StrgAng 0000
        eps,eps,eps,eps,  //2-AxlTrq 0000
        eps,eps,eps,eps,  //3-BrkPrs 0000
        eps,eps,eps,	  //4-WindXYZ 000
        eps,eps,eps,      //5-Grade 000
        eps,eps,eps,eps,  //6-Ground 0000
        1.0,1.0,1.0,1.0,  //7-Friction 111
        2.2e5,2.2e5,2.2e5,2.2e5, //Tire Pressure 220000
        273.0,// Air temperature Constant: Tair=273
        eps,eps,eps,eps,// Tire temperature Constant: Tamb=0
        eps,eps,eps,//Extern Fx, Fy, Fz
        eps,eps,eps
    };  
    double recv_buf[6];  
        

    io::CSVReader<41> m_inputs(input_file); //need modification
    while(true) {

        m_inputs.read_row(
			//1-WhlAng 0000
			send_buf[0], 
			send_buf[1],
            send_buf[2],
            send_buf[3],
			
			//2-AxlTrq 0000
			send_buf[4], 
			send_buf[5],
            send_buf[6],
            send_buf[7], \
			
			//3-BrkPrs 0000
			send_buf[8], 
			send_buf[9],
            send_buf[10],
            send_buf[11], \
			
			//4-WindXYZ 000
			send_buf[12], 
			send_buf[13],
            send_buf[14], \

            //5-Grade 0000
            send_buf[15], 
			send_buf[16],
            send_buf[17], \
			
			//5-Ground 0000
			send_buf[18], 
			send_buf[19],
            send_buf[20],
            send_buf[21], \
			
			//6-Friction 1
			send_buf[22], 
			send_buf[23],
            send_buf[24],
            send_buf[25], \
			
			//Other-parameters 220000
			send_buf[26], 
			send_buf[27],
            send_buf[28],
            send_buf[29], \
		
			// Air temperature Constant: Tair=273
			send_buf[30],
			
			// Tire temperature Constant: Tamb=0
			send_buf[31], 
			send_buf[32],
            send_buf[33],
            send_buf[34], \
			
			//000
			send_buf[35], 
			send_buf[36],
            send_buf[37],
			//000
			send_buf[38], 
			send_buf[39],
            send_buf[40]
		);
        udp_sptr_client->request(send_buf, sizeof(send_buf));
        usleep(500);
            
        udp_sptr_client->get_response(recv_buf, sizeof(recv_buf));  
            
        for (auto item : recv_buf) {
            std::cout<< item << ", ";
        }
        std::cout<<std::endl;
    } 
        
    return 0;  
}