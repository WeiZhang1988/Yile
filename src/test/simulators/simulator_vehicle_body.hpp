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
#ifndef SIMULATOR_VEHICLE_BODY_HPP
#define SIMULATOR_VEHICLE_BODY_HPP

#include <boost/numeric/odeint.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include "systems/system_vehicle_body.hpp"

using namespace boost::numeric::odeint;
using namespace std::chrono;
using namespace std;
using namespace Yile;

class Simulator_Vehicle_Body {
public:
	Simulator_Vehicle_Body(double t_start, double t_end, double t_step);
	void run ();

    shared_ptr<Sys_Vehicle_Body> m_sptr_sys;
    
    d_vec m_external_inputs = d_vec(Sys_Vehicle_Body::m_external_inputs_num,NaN);

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

#endif //SIMULATOR_VEHICLE_BODY_HPP
