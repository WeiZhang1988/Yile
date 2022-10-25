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
#include "simulator_sus_2ind.hpp"

Simulator_Sus_2Ind::Simulator_Sus_2Ind(real_Y t_start, real_Y t_end, real_Y t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	m_t_step_micros = m_t_step/1e-6;

	m_sptr_sys = make_shared<Sys_Sus_2Ind>();
	m_sptr_store = make_shared<d_v_vec>();
	m_sptr_interface = make_shared<Int_Sus_2Ind>();
	shared_ptr<Subsys_Sus_2Ind> sptr_sub_sus_2ind = make_shared<Subsys_Sus_2Ind>();
    
	m_sptr_sys->add_interface(m_sptr_interface);
	m_sptr_sys->add_subsys_sus_2ind(sptr_sub_sus_2ind);
	m_sptr_sys->add_store(m_sptr_store);
}

void Simulator_Sus_2Ind::run () {
	io::CSVReader<58> m_inputs("sus_test_result/sus_2ind_inputs10141457.csv");

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	real_Y t = m_t_start;
	m_tp_start = steady_clock::now();
	
	m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		m_inputs.read_row(\
		m_sptr_interface->m_Veh_hgt_cg, 
		m_sptr_interface->m_Veh_r, \

    	m_sptr_interface->m_Strg_str_fl, 
		m_sptr_interface->m_Strg_str_fr,
		m_sptr_interface->m_Strg_str_rl,
		m_sptr_interface->m_Strg_str_rr,

		m_sptr_interface->m_Sus_TirPz_fl, 
		m_sptr_interface->m_Sus_TirPz_fr, 
		m_sptr_interface->m_Sus_TirPz_rl, 
		m_sptr_interface->m_Sus_TirPz_rr, 

		m_sptr_interface->m_Sus_Tirvz_fl,
		m_sptr_interface->m_Sus_Tirvz_fr,
		m_sptr_interface->m_Sus_Tirvz_rl,
		m_sptr_interface->m_Sus_Tirvz_rr,

		m_sptr_interface->m_Tir_Re_fl, 
		m_sptr_interface->m_Tir_Re_fr, 
		m_sptr_interface->m_Tir_Re_rl, 
		m_sptr_interface->m_Tir_Re_rr, 
		
		m_sptr_interface->m_Int_Pz_fl, 
		m_sptr_interface->m_Int_Pz_fr,
		m_sptr_interface->m_Int_Pz_rl,
		m_sptr_interface->m_Int_Pz_rr,

		m_sptr_interface->m_Int_Vz_fl, 
		m_sptr_interface->m_Int_Vz_fr,
		m_sptr_interface->m_Int_Vz_rl,
		m_sptr_interface->m_Int_Vz_rr,

		m_sptr_interface->m_Veh_vx_fl, 
		m_sptr_interface->m_Veh_vx_fr, 
		m_sptr_interface->m_Veh_vx_rl, 
		m_sptr_interface->m_Veh_vx_rr, 
		
		m_sptr_interface->m_Veh_vy_fl,
		m_sptr_interface->m_Veh_vy_fr,
		m_sptr_interface->m_Veh_vy_rl,
		m_sptr_interface->m_Veh_vy_rr,

		m_sptr_interface->m_Veh_vz_fl, 
		m_sptr_interface->m_Veh_vz_fr, 
		m_sptr_interface->m_Veh_vz_rl, 
		m_sptr_interface->m_Veh_vz_rr, 

		m_sptr_interface->m_Sus_TirFx_fl, 
		m_sptr_interface->m_Sus_TirFx_fr, 
		m_sptr_interface->m_Sus_TirFx_rl, 
		m_sptr_interface->m_Sus_TirFx_rr, 

		m_sptr_interface->m_Sus_TirFy_fl,
		m_sptr_interface->m_Sus_TirFy_fr,
		m_sptr_interface->m_Sus_TirFy_rl,
		m_sptr_interface->m_Sus_TirFy_rr,
		
		m_sptr_interface->m_Tir_Mx_fl, 
		m_sptr_interface->m_Tir_Mx_fr,
		m_sptr_interface->m_Tir_Mx_rl,
		m_sptr_interface->m_Tir_Mx_rr,

		m_sptr_interface->m_Tir_My_fl, 
		m_sptr_interface->m_Tir_My_fr, 
		m_sptr_interface->m_Tir_My_rl, 
		m_sptr_interface->m_Tir_My_rr, 

		m_sptr_interface->m_Tir_Mz_fl,
		m_sptr_interface->m_Tir_Mz_fr,
		m_sptr_interface->m_Tir_Mz_rl,
		m_sptr_interface->m_Tir_Mz_rr
	); 
		
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