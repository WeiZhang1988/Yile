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
#include "simulator_tir_4fiala.hpp"

Simulator_Tir_4Fiala::Simulator_Tir_4Fiala(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	m_t_step_micros = m_t_step/1e-6;

	m_sptr_sys = make_shared<Sys_Tir_4Fiala>();
	shared_ptr<Subsys_Tire_4Fiala> sptr_sub_tir_4fiala = make_shared<Subsys_Tire_4Fiala>();
    shared_ptr<Tire_Fiala> sptr_tir_fl = make_shared<Tire_Fiala>();
	shared_ptr<Tire_Fiala> sptr_tir_fr = make_shared<Tire_Fiala>();
	shared_ptr<Tire_Fiala> sptr_tir_rl = make_shared<Tire_Fiala>();
	shared_ptr<Tire_Fiala> sptr_tir_rr = make_shared<Tire_Fiala>();
 
    sptr_sub_tir_4fiala->add_tirs(sptr_tir_fl,sptr_tir_fr,sptr_tir_rl,sptr_tir_rr);
	m_sptr_sys->add_subsys_tir_4fiala(sptr_sub_tir_4fiala);
    
    m_external_inputs = d_vec(Sys_Tir_4Fiala::m_external_inputs_num,0.0);
}

void Simulator_Tir_4Fiala::run () {

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

void Simulator_Tir_4Fiala::spin (const int &steps) {
	m_tp_end = steady_clock::now();
	microseconds dur_micros = duration_cast<microseconds>(m_tp_end - m_tp_start);
	int dur_micros_cnt = dur_micros.count();
	int dur_micros_cnt_trgt = steps * static_cast<int>(m_t_step_micros);
	if (dur_micros_cnt<dur_micros_cnt_trgt) {
		this_thread::sleep_for(microseconds(dur_micros_cnt_trgt - dur_micros_cnt));
	}
}
