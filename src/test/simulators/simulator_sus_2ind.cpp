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
	m_sptr_store = make_shared<d_v_vec>();
	m_sptr_interface = make_shared<Int_Sus_2Ind>();//very strange, link error without int() operation
	shared_ptr<Subsys_Sus_2Ind> sptr_sub_sus_2ind = make_shared<Subsys_Sus_2Ind>();
    shared_ptr<Sus_Ind_2Tracks> sptr_sus_f = make_shared<Sus_Ind_2Tracks>();
	shared_ptr<Sus_Ind_2Tracks> sptr_sus_r = make_shared<Sus_Ind_2Tracks>();
 
    sptr_sub_sus_2ind->add_suses(sptr_sus_f, sptr_sus_r);
	m_sptr_sys->add_interface(m_sptr_interface);
	m_sptr_sys->add_subsys_sus_2ind(sptr_sub_sus_2ind);
	m_sptr_sys->add_store(m_sptr_store);
}

void Simulator_Sus_2Ind::run () {
	io::CSVReader<15> m_inputs("data/inputs/sus_2ind_inputs.csv");

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	double t = m_t_start;

	m_tp_start = steady_clock::now();
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		m_inputs.read_row(m_sptr_interface->m_Veh_hgt_cg, m_sptr_interface->m_Veh_r, \
    	m_sptr_interface->m_Strg_str_fl, m_sptr_interface->m_Tir_Pz_fl, m_sptr_interface->m_Tir_vz_fl,	m_sptr_interface->m_Tir_Re_fl, \
		m_sptr_interface->m_Veh_Pz_fl, m_sptr_interface->m_Veh_vx_fl, m_sptr_interface->m_Veh_vy_fl, m_sptr_interface->m_Veh_vz_fl, \
		m_sptr_interface->m_Sus_TirFx_fl, m_sptr_interface->m_Sus_TirFy_fl, m_sptr_interface->m_Tir_Mx_fl, m_sptr_interface->m_Tir_My_fl, m_sptr_interface->m_Tir_Mz_fl); 
		m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_con_states,t,m_t_step);

		t += m_t_step;
		//spin(m_steps);
	}
	m_times.push_back(t);
	m_sptr_sys->pull_con_states(m_sptr_sys->m_con_states);
	m_sptr_sys->update_pv();
	m_sptr_sys->update_fm();
	m_sptr_sys->store_data();
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