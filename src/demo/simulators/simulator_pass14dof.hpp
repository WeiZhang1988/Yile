// =============================================================================
// PROJECT YILE - 
//
// Copyright (c) 2023
// All rights reserved.
//
// Use of this source code is governed by a GPL-3.0 license that can be found
// in the LICENSE file
//
// Authors of this file	Wei ZHANG wei_zhang_1988@outlook.com,
//						ChangMeng Hou 945881625@qq.com
//
// =============================================================================
#ifndef SIMULATOR_PASS14DOF_FIALA_HPP
#define SIMULATOR_PASS14DOF_FIALA_HPP

#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <string>
#include <filesystem>
#include <boost/property_tree/json_parser.hpp>
#include <atomic>
#include <mutex>
#include <thread>
#include "csv.hpp"
#include "communicators/udp_server.hpp"
#include "yile.hpp"

namespace pt = boost::property_tree;
using namespace boost::numeric::odeint;
using namespace std::chrono;
using namespace std;
using namespace Yile;

class Simulator_Pass14DOF {
public:
	Simulator_Pass14DOF(string par_file, real_Y t_start, real_Y t_end, real_Y t_step);
    void run ();
    
    shared_ptr<Sys_Chassis_2Ind_Disk_Fiala>     m_sptr_sys;
    shared_ptr<d_v_vec>                         m_sptr_store;
    shared_ptr<Int_Chassis_2Ind_Disk_Fiala>     m_sptr_interface;

    int m_steps;
    d_v_vec m_states, m_outputs;
    d_vec m_times;
    real_Y m_t_current;
	
private:
    void udp_pull();
    void udp_push();
    void do_simulation();
	void spin(const int &steps);
    d_vec read_json_list(const pt::ptree &in);

    std::atomic<bool> m_enable_do_sim = false;
    std::atomic<bool> m_simulation_done = false;
    std::atomic<bool> m_enable_output = false;
    std::mutex m_mu;

    double m_udp_pull_send_buf[6] = {0.0,0.0,0.0,0.0,0.0,0.0};  
  	double m_udp_pull_recv_buf[Int_Chassis_2Ind_Disk_Fiala::m_inputs_nums] = {
		0.0,0.0,0.0,0.0,  //1-StrgAng 0000
		0.0,0.0,0.0,0.0,  //2-AxlTrq 0000
		0.0,0.0,0.0,0.0,  //3-BrkPrs 0000
		0.0,0.0,0.0,	  //4-WindXYZ 000
		0.0,0.0,0.0,0.0,  //5-Ground 0000
		1.0,1.0,1.0,1.0,  //6-Friction 111
		2.2e5,2.2e5,2.2e5,2.2e5, //Tire Pressure 220000
		273.0,// Air temperature Constant: Tair=273
		0.0,0.0,0.0,0.0,// Tire temperature Constant: Tamb=0
		0.0,0.0,0.0,//Extern Fx, Fy, Fz
		0.0,0.0,0.0 //Extern Mx, My, Mz
	};
    UDP_Server m_udp_pull_server = UDP_Server(9000);
    double m_udp_push_send_buf[6] = {0.0,0.0,0.0,0.0,0.0,0.0};  
  	double m_udp_push_recv_buf[1] = {0.0};
    UDP_Server m_udp_push_server = UDP_Server(9001);

    string m_par_file;
	real_Y m_t_start, m_t_end, m_t_step, m_t_step_micros;
    runge_kutta4<d_vec> m_stepper;
    //euler<d_vec> m_stepper;
    steady_clock::time_point m_tp_start = steady_clock::now();
    steady_clock::time_point m_tp_end = steady_clock::now();

};

#endif //SIMULATOR_PASS14DOF_FIALA_HPP
