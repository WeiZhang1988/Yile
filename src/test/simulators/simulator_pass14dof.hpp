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
#ifndef SIMULATOR_PASS14DOF_FIALA_HPP
#define SIMULATOR_PASS14DOF_FIALA_HPP

#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include "systems/system_chassis_2ind_disk_fiala.hpp"
#include "csv.hpp"

using namespace boost::numeric::odeint;
using namespace std::chrono;
using namespace std;
using namespace Yile;

class Simulator_Pass14DOF {
public:
	Simulator_Pass14DOF(real_Y t_start, real_Y t_end, real_Y t_step);
	
    void step(const std::vector<double>& inputs);
    void udp_pthread();
    void do_simulation();
    void run ();
    
    shared_ptr<Sys_Chassis_2Ind_Disk_Fiala>     m_sptr_sys;
    shared_ptr<d_v_vec>                         m_sptr_store;
    shared_ptr<Int_Chassis_2Ind_Disk_Fiala>     m_sptr_interface;

    int m_steps;
    d_v_vec m_states, m_outputs;
    d_vec m_times;
    real_Y m_t_current;
	
private:
    bool m_enable_do_sim = false;
    bool m_simulation_done = false;
	void spin(const int &steps);
    std::mutex m_mu;

	real_Y m_t_start, m_t_end, m_t_step, m_t_step_micros;
    runge_kutta4<d_vec> m_stepper;
    //euler<d_vec> m_stepper;
    steady_clock::time_point m_tp_start = steady_clock::now();
    steady_clock::time_point m_tp_end = steady_clock::now();

};

#endif //SIMULATOR_PASS14DOF_FIALA_HPP
