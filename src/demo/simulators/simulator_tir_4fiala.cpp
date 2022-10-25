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
#include "simulator_tir_4fiala.hpp"

Simulator_Tir_4Fiala::Simulator_Tir_4Fiala(real_Y t_start, real_Y t_end, real_Y t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	m_t_step_micros = m_t_step/1e-6;

	m_sptr_sys = make_shared<Sys_Tir_4Fiala>();
	m_sptr_store = make_shared<d_v_vec>();
	m_sptr_interface = make_shared<Int_Tir_4Fiala>();
	shared_ptr<Subsys_Tire_4Fiala> sptr_sub_tir_4fiala = make_shared<Subsys_Tire_4Fiala>();
 
	m_sptr_sys->add_subsys_tir_4fiala(sptr_sub_tir_4fiala);
	m_sptr_sys->add_interface(m_sptr_interface);
	m_sptr_sys->add_store(m_sptr_store);
}

void Simulator_Tir_4Fiala::run () {

	io::CSVReader<52> m_inputs("tir_test_result/tirfiala_inputs9280920.csv");
	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	real_Y t = m_t_start;

	m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
	m_tp_start = steady_clock::now();
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		m_inputs.read_row(
		m_sptr_interface->m_Tir_omega_fl, 
		m_sptr_interface->m_Tir_omega_fr, 
		m_sptr_interface->m_Tir_omega_rl, 
		m_sptr_interface->m_Tir_omega_rr, 

		m_sptr_interface->m_Tir_rhoz_fl,
		m_sptr_interface->m_Tir_rhoz_fr,
		m_sptr_interface->m_Tir_rhoz_rl, 
		m_sptr_interface->m_Tir_rhoz_rr,

		m_sptr_interface->m_Tir_Re_fl, 
		m_sptr_interface->m_Tir_Re_fr, 
		m_sptr_interface->m_Tir_Re_rl,
		m_sptr_interface->m_Tir_Re_rr,

		m_sptr_interface->m_Sus_vx_fl, 
		m_sptr_interface->m_Sus_vx_fr, 
		m_sptr_interface->m_Sus_vx_rl, 
		m_sptr_interface->m_Sus_vx_rr,


		m_sptr_interface->m_Sus_vy_fl, 
		m_sptr_interface->m_Sus_vy_fr, 
		m_sptr_interface->m_Sus_vy_rl, 
		m_sptr_interface->m_Sus_vy_rr,

		m_sptr_interface->m_Sus_vz_fl, 
		m_sptr_interface->m_Sus_vz_fr, 
		m_sptr_interface->m_Sus_vz_rl, 
		m_sptr_interface->m_Sus_vz_rr, 
		
		m_sptr_interface->m_Sus_gamma_fl, 
		m_sptr_interface->m_Sus_gamma_fr, 
		m_sptr_interface->m_Sus_gamma_rl, 
		m_sptr_interface->m_Sus_gamma_rr, 

		m_sptr_interface->m_Sus_str_fl, 
		m_sptr_interface->m_Sus_str_fr, 
		m_sptr_interface->m_Sus_str_rl, 
		m_sptr_interface->m_Sus_str_rr, 

		m_sptr_interface->m_Sus_r_fl, 
		m_sptr_interface->m_Sus_r_fr,
		m_sptr_interface->m_Sus_r_rl,
		m_sptr_interface->m_Sus_r_rr,

		m_sptr_interface->m_Sus_Fz_fl, 
		m_sptr_interface->m_Sus_Fz_fr,
		m_sptr_interface->m_Sus_Fz_rl,
		m_sptr_interface->m_Sus_Fz_rr,

		m_sptr_interface->m_Gnd_scale_fl, 
		m_sptr_interface->m_Gnd_scale_fr, 
		m_sptr_interface->m_Gnd_scale_rl, 
		m_sptr_interface->m_Gnd_scale_rr, 

		m_sptr_interface->m_Tir_Prs_fl, 
		m_sptr_interface->m_Tir_Prs_fr, 
		m_sptr_interface->m_Tir_Prs_rl, 
		m_sptr_interface->m_Tir_Prs_rr, 

		m_sptr_interface->m_Air_Tamb_fl,
		m_sptr_interface->m_Air_Tamb_fr,
		m_sptr_interface->m_Air_Tamb_rl,
		m_sptr_interface->m_Air_Tamb_rr
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

void Simulator_Tir_4Fiala::spin (const int &steps) {
	m_tp_end = steady_clock::now();
	microseconds dur_micros = duration_cast<microseconds>(m_tp_end - m_tp_start);
	int dur_micros_cnt = dur_micros.count();
	int dur_micros_cnt_trgt = steps * static_cast<int>(m_t_step_micros);
	if (dur_micros_cnt<dur_micros_cnt_trgt) {
		this_thread::sleep_for(microseconds(dur_micros_cnt_trgt - dur_micros_cnt));
	}
}