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
#include "simulator_whl_4disk.hpp"

Simulator_Whl_4Disk::Simulator_Whl_4Disk(real_Y t_start, real_Y t_end, real_Y t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	m_t_step_micros = m_t_step/1e-6;

	m_sptr_sys = make_shared<Sys_Whl_4Disk>();
	m_sptr_store = make_shared<d_v_vec>();
	m_sptr_interface = make_shared<Int_Whl_4Disk>();
	shared_ptr<Subsys_Wheel_4Disk> sptr_sub_whl_4disk = make_shared<Subsys_Wheel_4Disk>();

	m_sptr_sys->add_subsys_whl_4disk(sptr_sub_whl_4disk);
	m_sptr_sys->add_interface(m_sptr_interface);
	m_sptr_sys->add_store(m_sptr_store);

}

void Simulator_Whl_4Disk::run () {
	io::CSVReader<28> m_inputs("wheel_test_result/whl_inputs_10121430.csv");

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	real_Y t = m_t_start;
	m_tp_start = steady_clock::now();
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		m_inputs.read_row(
		m_sptr_interface->m_Brk_Prs_fl,
		m_sptr_interface->m_Brk_Prs_fr, 
		m_sptr_interface->m_Brk_Prs_rl,
		m_sptr_interface->m_Brk_Prs_rr, 

		m_sptr_interface->m_Axl_Trq_fl, 
		m_sptr_interface->m_Axl_Trq_fr,
		m_sptr_interface->m_Axl_Trq_rl,
		m_sptr_interface->m_Axl_Trq_rr, 

		m_sptr_interface->m_Tir_Fx_fl, 
		m_sptr_interface->m_Tir_Fx_fr, 
		m_sptr_interface->m_Tir_Fx_rl,
		m_sptr_interface->m_Tir_Fx_rr, 

		m_sptr_interface->m_Tir_My_fl, 
		m_sptr_interface->m_Tir_My_fr, 
		m_sptr_interface->m_Tir_My_rl, 
		m_sptr_interface->m_Tir_My_rr, 

		m_sptr_interface->m_Gnd_Pz_fl, 
		m_sptr_interface->m_Gnd_Pz_fr,
		m_sptr_interface->m_Gnd_Pz_rl, 
		m_sptr_interface->m_Gnd_Pz_rr, 

		m_sptr_interface->m_Sus_Fz_fl, 
		m_sptr_interface->m_Sus_Fz_fr, 
		m_sptr_interface->m_Sus_Fz_rl, 
		m_sptr_interface->m_Sus_Fz_rr, 

		m_sptr_interface->m_Tir_Fz_fl,
		m_sptr_interface->m_Tir_Fz_fr,
		m_sptr_interface->m_Tir_Fz_rl,
		m_sptr_interface->m_Tir_Fz_rr
		); 
		
		m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_con_states,t,m_t_step);
		m_sptr_sys->pull_con_states(m_sptr_sys->m_con_states);	
			
		t += m_t_step;
		//spin(m_steps);
	}
	m_times.push_back(t);
	m_sptr_sys->pull_con_states(m_sptr_sys->m_con_states);
	m_sptr_sys->update_pv();
	m_sptr_sys->update_fm();
	m_sptr_sys->store_data();
}


void Simulator_Whl_4Disk::spin (const int &steps) {
	m_tp_end = steady_clock::now();
	microseconds dur_micros = duration_cast<microseconds>(m_tp_end - m_tp_start);
	int dur_micros_cnt = dur_micros.count();
	int dur_micros_cnt_trgt = steps * static_cast<int>(m_t_step_micros);
	if (dur_micros_cnt<dur_micros_cnt_trgt) {
		this_thread::sleep_for(microseconds(dur_micros_cnt_trgt - dur_micros_cnt));
	}
}