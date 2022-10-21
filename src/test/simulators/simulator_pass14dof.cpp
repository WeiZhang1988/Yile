// =============================================================================
// PROJECT YILE - 
//
// Copyright (c) 2023
// All rights reserved.
//
// Use of this source code is governed by a GPL-3.0 license that can be found
// in the LICENSE file
//
// Author of this file	Wei ZHANG wei_zhang_1988@outlook.com,ChangMeng Hou 945881625@qq.com
//
// =============================================================================
#include "simulator_pass14dof.hpp"
//upd required
#include <stdio.h>   
#include <sys/types.h>   
#include <sys/socket.h>   
#include <netinet/in.h>    
#include <errno.h>    
#include <stdlib.h>  
#define SERV_PORT   9000 
//upd required
//multi thread required
#include <mutex>
#include <condition_variable>
#include <thread>
//multi thread required

Simulator_Pass14DOF::Simulator_Pass14DOF(real_Y t_start, real_Y t_end, real_Y t_step) {
	m_steps         = 0;
	m_t_start       = t_start;
	m_t_end         = t_end;
	m_t_step        = t_step;
	m_t_step_micros = m_t_step/1e-6;
	m_t_current     = m_t_start;

	m_sptr_sys          = make_shared<Sys_Chassis_2Ind_Disk_Fiala>();
	m_sptr_store        = make_shared<d_v_vec>();
	m_sptr_interface    = make_shared<Int_Chassis_2Ind_Disk_Fiala>();
    shared_ptr<Vehicle_Body> sptr_vhl_bdy               = make_shared<Vehicle_Body>();
    shared_ptr<Subsys_Wheel_4Disk> sptr_sub_whl_4disk   = make_shared<Subsys_Wheel_4Disk>();
	shared_ptr<Subsys_Sus_2Ind> sptr_sub_sus_2ind       = make_shared<Subsys_Sus_2Ind>();
    shared_ptr<Subsys_Tire_4Fiala> sptr_sub_tir_4fiala  = make_shared<Subsys_Tire_4Fiala>();
 
    m_sptr_sys->add_vhl_bdy(sptr_vhl_bdy);
	m_sptr_sys->add_subsys_whl_4disk(sptr_sub_whl_4disk);
	m_sptr_sys->add_subsys_sus_2ind(sptr_sub_sus_2ind);
	m_sptr_sys->add_subsys_tir_4fiala(sptr_sub_tir_4fiala);

    m_sptr_sys->add_interface(m_sptr_interface);
    m_sptr_sys->add_store(m_sptr_store);
}

void Simulator_Pass14DOF::step(const std::vector<real_Y>& inputs){
	//1-StreeringAng
	m_sptr_interface->m_Strg_str_fl = inputs[0];
	m_sptr_interface->m_Strg_str_fr = inputs[1];
	m_sptr_interface->m_Strg_str_rl = inputs[2];
	m_sptr_interface->m_Strg_str_rr = inputs[3];
	
	//2-AxlTrq 0000
	m_sptr_interface->m_Axl_Trq_fl = inputs[4];
	m_sptr_interface->m_Axl_Trq_fr = inputs[5];
	m_sptr_interface->m_Axl_Trq_rl = inputs[6];
	m_sptr_interface->m_Axl_Trq_rr = inputs[7];
	
	//3-BrkPrs 0000
	m_sptr_interface->m_Brk_Prs_fl = inputs[8];
	m_sptr_interface->m_Brk_Prs_fr = inputs[9];
	m_sptr_interface->m_Brk_Prs_rl = inputs[10];
	m_sptr_interface->m_Brk_Prs_rr = inputs[11];
	
	//4-WindXYZ 000
	m_sptr_interface->m_Air_Wx = inputs[12];
	m_sptr_interface->m_Air_Wy = inputs[13];
	m_sptr_interface->m_Air_Wz = inputs[14];
	
	//5-Ground 0000
	m_sptr_interface->m_Gnd_Pz_fl = inputs[15];
	m_sptr_interface->m_Gnd_Pz_fr = inputs[16];
	m_sptr_interface->m_Gnd_Pz_rl = inputs[17];
	m_sptr_interface->m_Gnd_Pz_rr = inputs[18];
	
	//6-Friction 1111
	m_sptr_interface->m_Gnd_scale_fl = inputs[19];
	m_sptr_interface->m_Gnd_scale_fr = inputs[20];
	m_sptr_interface->m_Gnd_scale_rl = inputs[21];
	m_sptr_interface->m_Gnd_scale_rr = inputs[22];
	
	//Other-parameters 220000
	m_sptr_interface->m_Tir_Prs_fl = inputs[23];
	m_sptr_interface->m_Tir_Prs_fr = inputs[24];
	m_sptr_interface->m_Tir_Prs_rl = inputs[25];
	m_sptr_interface->m_Tir_Prs_rr = inputs[26];
	
	// Air temperature Constant: Tair=273
	m_sptr_interface->m_Air_Tair   = inputs[27];
	
	// Tire temperature Constant: Tamb=0
	m_sptr_interface->m_Air_Tamb_fl = inputs[28];
	m_sptr_interface->m_Air_Tamb_fr = inputs[29];
	m_sptr_interface->m_Air_Tamb_rl = inputs[30];
	m_sptr_interface->m_Air_Tamb_rr = inputs[31];
	
	//000
	m_sptr_interface->m_Ext_Fx_ext = inputs[32];
	m_sptr_interface->m_Ext_Fy_ext = inputs[33];
	m_sptr_interface->m_Ext_Fz_ext = inputs[34];
	//000
	m_sptr_interface->m_Ext_Mx_ext = inputs[35]; 
	m_sptr_interface->m_Ext_My_ext = inputs[36];
	m_sptr_interface->m_Ext_Mz_ext = inputs[37];

	m_sptr_sys->push_con_states_veh_whl(m_sptr_sys->m_con_states);
	//m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
	m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_con_states,m_t_current,m_t_step);
	//m_sptr_sys->pull_con_states(m_sptr_sys->m_con_states);
	m_sptr_sys->pull_con_states_veh_whl(m_sptr_sys->m_con_states);
	m_t_current += m_t_step;
}

void Simulator_Pass14DOF::run () {
	/*//1-StreeringAng
	m_sptr_interface->m_Strg_str_fl = 0;
	m_sptr_interface->m_Strg_str_fr = 0;
	m_sptr_interface->m_Strg_str_rl = 0;
	m_sptr_interface->m_Strg_str_rr = 0;
	
	//2-AxlTrq 0000
	m_sptr_interface->m_Axl_Trq_fl = 0;
	m_sptr_interface->m_Axl_Trq_fr = 0;
	m_sptr_interface->m_Axl_Trq_rl = 0;
	m_sptr_interface->m_Axl_Trq_rr = 0;
	
;	//3-BrkPrs 0000
;	m_sptr_interface->m_Brk_Prs_fl = 0;
	m_sptr_interface->m_Brk_Prs_fr = 0;
	m_sptr_interface->m_Brk_Prs_rl = 0;
	m_sptr_interface->m_Brk_Prs_rr = 0;
	
	//4-WindXYZ 000
	m_sptr_interface->m_Air_Wx = 0;
	m_sptr_interface->m_Air_Wy = 0;
	m_sptr_interface->m_Air_Wz = 0;
	
	//5-Ground 0000
	m_sptr_interface->m_Gnd_Pz_fl = 0;
	m_sptr_interface->m_Gnd_Pz_fr = 0;
	m_sptr_interface->m_Gnd_Pz_rl = 0;
	m_sptr_interface->m_Gnd_Pz_rr = 0;
	
	//6-Friction 1111
	m_sptr_interface->m_Gnd_scale_fl = 1;
	m_sptr_interface->m_Gnd_scale_fr = 1;
	m_sptr_interface->m_Gnd_scale_rl = 1;
	m_sptr_interface->m_Gnd_scale_rr = 1;
	
	//Other-parameters 220000
	m_sptr_interface->m_Tir_Prs_fl = 220000;
	m_sptr_interface->m_Tir_Prs_fr = 220000;
	m_sptr_interface->m_Tir_Prs_rl = 220000;
	m_sptr_interface->m_Tir_Prs_rr = 220000;
	
	// Air temperature Constant: Tair=273
	m_sptr_interface->m_Air_Tair   = 273;
	
	// Tire temperature Constant: Tamb=0
	m_sptr_interface->m_Air_Tamb_fl = 0;
	m_sptr_interface->m_Air_Tamb_fr = 0;
	m_sptr_interface->m_Air_Tamb_rl = 0;
	m_sptr_interface->m_Air_Tamb_rr = 0;
	
	//000
	m_sptr_interface->m_Ext_Fx_ext = 0;
	m_sptr_interface->m_Ext_Fy_ext = 0;
	m_sptr_interface->m_Ext_Fz_ext = 0;
	//000
	m_sptr_interface->m_Ext_Mx_ext = 0; 
	m_sptr_interface->m_Ext_My_ext = 0;
	m_sptr_interface->m_Ext_Mz_ext = 0;*/

	std::thread t1(&Simulator_Pass14DOF::udp_pthread,this);
	std::thread t2(&Simulator_Pass14DOF::do_simulation,this);
	t1.join();
	t2.join();
}

void Simulator_Pass14DOF::spin (const int &steps) {
	m_tp_end = steady_clock::now();
	microseconds dur_micros = duration_cast<microseconds>(m_tp_end - m_tp_start);
	int dur_micros_cnt = dur_micros.count();
	int dur_micros_cnt_trgt = steps * static_cast<int>(m_t_step_micros);
	if (dur_micros_cnt<dur_micros_cnt_trgt) {
	this_thread::sleep_for(microseconds(dur_micros_cnt_trgt - dur_micros_cnt));
	}
}

void Simulator_Pass14DOF::udp_pthread() {
	/* sock_fd --- socket file descriptor create upd socket*/  
	std::cout<< "udp thread start:"<<std::endl;
	int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if(sock_fd < 0)  
	{  
		perror("socket");  
		exit(1);  
	} 

	/* bind socket to IP address and port number */  
	struct sockaddr_in addr_serv;  
	int len;  
	memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //set every byte to zero
	addr_serv.sin_family = AF_INET;                     //use IPV4
	addr_serv.sin_port = htons(SERV_PORT);              //set port number
	/* INADDR_ANY indicates that no matter which net card receives data
	，as lont as port number is SERV_PORT，the program will receive it */  
	addr_serv.sin_addr.s_addr = htonl(INADDR_ANY);  	//automatically have IP address
	len = sizeof(addr_serv); 

	/* bind socket */  
	if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)  
	{  
		perror("bind error:");  
		exit(1);  
	}  

	int recved_num;
	int sent_num;

	double send_buf[6] = {0.0,0.0,0.0,0.0,0.0,0.0};  
  	double recv_buf[Int_Chassis_2Ind_Disk_Fiala::m_inputs_nums] = {
		eps,eps,eps,eps,  //1-StrgAng 0000
		eps,eps,eps,eps,  //2-AxlTrq 0000
		eps,eps,eps,eps,  //3-BrkPrs 0000
		eps,eps,eps,	  //4-WindXYZ 000
		eps,eps,eps,eps,  //5-Ground 0000
		1.0,1.0,1.0,1.0,  //6-Friction 111
		2.2e5,2.2e5,2.2e5,2.2e5, //Tire Pressure 220000
		273.0,// Air temperature Constant: Tair=273
		eps,eps,eps,eps,// Tire temperature Constant: Tamb=0
		eps,eps,eps,//Extern Fx, Fy, Fz
		eps,eps,eps //Extern Mx, My, Mz
	};
	struct sockaddr_in addr_client;
	
	std::cout<< "udp thread..."<<std::endl;
	
	while(!m_simulation_done)  
	{  
		recved_num = recvfrom(sock_fd, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *)&addr_client, (socklen_t *)&len);  
		
		if(recved_num < 0)  {  
		perror("recvfrom error:");  
		exit(1);  
		}  

		std::unique_lock<std::mutex> locker(m_mu); 
		//1-StreeringAng
		m_sptr_interface->m_Strg_str_fl = recv_buf[0];
		m_sptr_interface->m_Strg_str_fr = recv_buf[1];
		m_sptr_interface->m_Strg_str_rl = recv_buf[2];
		m_sptr_interface->m_Strg_str_rr = recv_buf[3];
		
		//2-AxlTrq 0000
		m_sptr_interface->m_Axl_Trq_fl = recv_buf[4];
		m_sptr_interface->m_Axl_Trq_fr = recv_buf[5];
		m_sptr_interface->m_Axl_Trq_rl = recv_buf[6];
		m_sptr_interface->m_Axl_Trq_rr = recv_buf[7];

		//3-BrkPrs 000
		m_sptr_interface->m_Brk_Prs_fl = recv_buf[8];
		m_sptr_interface->m_Brk_Prs_fr = recv_buf[9];
		m_sptr_interface->m_Brk_Prs_rl = recv_buf[10];
		m_sptr_interface->m_Brk_Prs_rr = recv_buf[11];
		
		//4-WindXYZ 000
		m_sptr_interface->m_Air_Wx = recv_buf[12];
		m_sptr_interface->m_Air_Wy = recv_buf[13];
		m_sptr_interface->m_Air_Wz = recv_buf[14];
		
		//5-Ground 0000
		m_sptr_interface->m_Gnd_Pz_fl = recv_buf[15];
		m_sptr_interface->m_Gnd_Pz_fr = recv_buf[16];
		m_sptr_interface->m_Gnd_Pz_rl = recv_buf[17];
		m_sptr_interface->m_Gnd_Pz_rr = recv_buf[18];
		
		//6-Friction 1111
		m_sptr_interface->m_Gnd_scale_fl = recv_buf[19];
		m_sptr_interface->m_Gnd_scale_fr = recv_buf[20];
		m_sptr_interface->m_Gnd_scale_rl = recv_buf[21];
		m_sptr_interface->m_Gnd_scale_rr = recv_buf[22];
		
		//Other-parameters 220000
		m_sptr_interface->m_Tir_Prs_fl = recv_buf[23];
		m_sptr_interface->m_Tir_Prs_fr = recv_buf[24];
		m_sptr_interface->m_Tir_Prs_rl = recv_buf[25];
		m_sptr_interface->m_Tir_Prs_rr = recv_buf[26];
		
		// Air temperature Constant: Tair=273
		m_sptr_interface->m_Air_Tair   = recv_buf[27];
		
		// Tire temperature Constant: Tamb=0
		m_sptr_interface->m_Air_Tamb_fl = recv_buf[28];
		m_sptr_interface->m_Air_Tamb_fr = recv_buf[29];
		m_sptr_interface->m_Air_Tamb_rl = recv_buf[30];
		m_sptr_interface->m_Air_Tamb_rr = recv_buf[31];
		
		//000
		m_sptr_interface->m_Ext_Fx_ext = recv_buf[32];
		m_sptr_interface->m_Ext_Fy_ext = recv_buf[33];
		m_sptr_interface->m_Ext_Fz_ext = recv_buf[34];
		//000
		m_sptr_interface->m_Ext_Mx_ext = recv_buf[35]; 
		m_sptr_interface->m_Ext_My_ext = recv_buf[36];
		m_sptr_interface->m_Ext_Mz_ext = recv_buf[37];
		locker.unlock();

		m_enable_do_sim = true;
		
		send_buf[0] = m_sptr_interface->m_xe_x_c;
		send_buf[1] = m_sptr_interface->m_xe_y_c;
		send_buf[2] = m_sptr_interface->m_xe_z_c;
		send_buf[3] = m_sptr_interface->m_phai_c;
		send_buf[4] = m_sptr_interface->m_theta_c;
		send_buf[5] = m_sptr_interface->m_psi_c;
		sent_num = sendto(sock_fd, send_buf, sizeof(send_buf), 0, (struct sockaddr *)&addr_client, len);  
		
		if(sent_num < 0)  {  
		perror("sendto error:");  
		exit(1);  
		} 
	}  
    
  	close(sock_fd);  
}

void Simulator_Pass14DOF::do_simulation() {
	
	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	real_Y t = m_t_start;
	while(!m_enable_do_sim);
	m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
	m_tp_start = steady_clock::now();
	for (int i=0; i<steps_num; i++) {
		
		m_steps++;	
		m_times.push_back(t);
		std::unique_lock<std::mutex> locker(m_mu); 
		m_sptr_sys->push_con_states_veh_whl(m_sptr_sys->m_con_states);
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_con_states,t,m_t_step);
		m_sptr_sys->pull_con_states_veh_whl(m_sptr_sys->m_con_states);
		locker.unlock();
		t += m_t_step;
		spin(m_steps);
		std::cout<< i <<std::endl;
	}
	m_times.push_back(t);
	m_sptr_sys->pull_con_states(m_sptr_sys->m_con_states);
	m_sptr_sys->update_pv();
	m_sptr_sys->update_fm();
	m_sptr_sys->store_data();
	m_simulation_done = true;
	//exit(0);
}