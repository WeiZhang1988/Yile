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
#include "simulator_vehicle_body.hpp"

Simulator_Vehicle_Body::Simulator_Vehicle_Body(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	m_t_step_micros = m_t_step/1e-6;

	m_sptr_sys = make_shared<Sys_Vehicle_Body>();
	m_sptr_interface = make_shared<Int_Vehicle_Body>(int(Sys_Vehicle_Body::m_external_inputs_num));
	shared_ptr<Vehicle_Body> sptr_vhl_bdy = make_shared<Vehicle_Body>();
 
	m_sptr_sys->add_vhl_bdy(sptr_vhl_bdy);
	m_sptr_sys->add_interface(m_sptr_interface);
    
    m_external_inputs = d_vec(Sys_Vehicle_Body::m_external_inputs_num,10.0);
}

void Simulator_Vehicle_Body::run() {
	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	double t = m_t_start;
	
	m_tp_start = steady_clock::now();
	m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);

		m_sptr_sys->pull_external_inputs (m_external_inputs);
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_con_states,t,m_t_step);
		m_outputs.push_back(m_sptr_interface->m_vhl_bdy_pv_outputs);
	
		t += m_t_step;
		//spin(m_steps);
	}
	
	m_times.push_back(t);
	m_sptr_sys->pull_con_states(m_sptr_sys->m_con_states);
	m_sptr_sys->update_pv();
	m_sptr_sys->update_fm();
	m_outputs.push_back(m_sptr_interface->m_vhl_bdy_pv_outputs);
}

void Simulator_Vehicle_Body::spin (const int &steps) {
	m_tp_end = steady_clock::now();
	microseconds dur_micros = duration_cast<microseconds>(m_tp_end - m_tp_start);
	int dur_micros_cnt = dur_micros.count();
	int dur_micros_cnt_trgt = steps * static_cast<int>(m_t_step_micros);
	if (dur_micros_cnt<dur_micros_cnt_trgt) {
		this_thread::sleep_for(microseconds(dur_micros_cnt_trgt - dur_micros_cnt));
	}
}
