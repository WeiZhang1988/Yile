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
#include "simulator_sus_2ind.hpp"

Simulator_Sus_2Ind::Simulator_Sus_2Ind(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	m_t_step_micros = m_t_step/1e-6;

	m_sptr_sys = make_shared<Sys_Sus_2Ind>();
	shared_ptr<Subsys_Sus_2Ind> sptr_sub_sus_2ind = make_shared<Subsys_Sus_2Ind>();
    shared_ptr<Sus_Ind_2Tracks> sptr_sus_f = make_shared<Sus_Ind_2Tracks>();
	shared_ptr<Sus_Ind_2Tracks> sptr_sus_r = make_shared<Sus_Ind_2Tracks>();
 
    sptr_sub_sus_2ind->add_suses(sptr_sus_f, sptr_sus_r);
	m_sptr_sys->add_subsys_sus_2ind(sptr_sub_sus_2ind);
    
    m_external_inputs = d_vec(Sys_Sus_2Ind::m_external_inputs_num,0.0);
}

void Simulator_Sus_2Ind::run () {

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	double t = m_t_start;

	m_tp_start = steady_clock::now();
	for (int i=0; i<steps_num; i++) {
		m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
		m_steps++;	
		m_times.push_back(t);
		m_outputs.push_back(m_sptr_sys->m_con_states);

		m_sptr_sys->pull_external_inputs (m_external_inputs);
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_con_states,t,m_t_step);
		
		t += m_t_step;
		spin(m_steps);
	}
	m_times.push_back(t);
	m_outputs.push_back(m_sptr_sys->m_con_states);
}

void Simulator_Sus_2Ind::spin (const int &steps) {
	m_tp_end = steady_clock::now();
	microseconds dur_micros = duration_cast<microseconds>(m_tp_end - m_tp_start);
	int dur_micros_cnt = dur_micros.count();
	int dur_micros_cnt_trgt = steps * static_cast<int>(m_t_step_micros);
	if (dur_micros_cnt<dur_micros_cnt_trgt) {
		this_thread::sleep_for(microseconds(dur_micros_cnt_trgt - dur_micros_cnt));
	}
}