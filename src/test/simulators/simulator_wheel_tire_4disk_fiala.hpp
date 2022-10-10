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
#ifndef SIMULATOR_WHEEL_TIRE_4DISK_FIALA_HPP
#define SIMULATOR_WHEEL_TIRE_4DISK_FIALA_HPP

#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include "systems/system_wheel_tire_4disk_fiala.hpp"
#include "csv.hpp"

using namespace boost::numeric::odeint;
using namespace std::chrono;
using namespace std;
using namespace Yile;

class Simulator_Wheel_Tire_4Disk_Fiala {
public:
	Simulator_Wheel_Tire_4Disk_Fiala(double t_start, double t_end, double t_step);
	void run ();

    shared_ptr<Sys_Wheel_Tire_4Disk_Fiala>      m_sptr_sys;
    shared_ptr<d_v_vec>                         m_sptr_store;
    shared_ptr<Int_Chassis_2Ind_Disk_Fiala>     m_sptr_interface;

    int m_steps;
    d_v_vec m_states, m_outputs;
    d_vec m_times;
	
private:
	void spin(const int &steps);

	double m_t_start, m_t_end, m_t_step, m_t_step_micros;
    runge_kutta4<d_vec> m_stepper;
    steady_clock::time_point m_tp_start = steady_clock::now();
    steady_clock::time_point m_tp_end = steady_clock::now();

};

#endif //SIMULATOR_WHEEL_TIRE_4DISK_FIALA_HPP
