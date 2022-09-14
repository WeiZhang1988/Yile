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
#include "simulator_whl_4disk.hpp"


Simulator_Whl_4Disk::Simulator_Whl_4Disk(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	m_t_step_micros = m_t_step/1e-6;

	m_sptr_sys = make_shared<Sys_Whl_4Disk>();
	m_sptr_interface = make_shared<Int_Whl_4Disk>();
	shared_ptr<Subsys_Wheel_4Disk> sptr_sub_whl_4disk = make_shared<Subsys_Wheel_4Disk>();
    shared_ptr<Wheel_Disk> sptr_whl_fl = make_shared<Wheel_Disk>();
	shared_ptr<Wheel_Disk> sptr_whl_fr = make_shared<Wheel_Disk>();
	shared_ptr<Wheel_Disk> sptr_whl_rl = make_shared<Wheel_Disk>();
	shared_ptr<Wheel_Disk> sptr_whl_rr = make_shared<Wheel_Disk>();
 
    sptr_sub_whl_4disk->add_whls(sptr_whl_fl,sptr_whl_fr,sptr_whl_rl,sptr_whl_rr);
	m_sptr_sys->add_subsys_whl_4disk(sptr_sub_whl_4disk);
	m_sptr_sys->add_interface(m_sptr_interface);
}

void Simulator_Whl_4Disk::run () {
	io::CSVReader<7> m_inputs("data/inputs/whl_4disk_inputs.csv");

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	double t = m_t_start;

	m_tp_start = steady_clock::now();
	m_sptr_sys->update_pv();
	m_sptr_sys->update_fm();
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		d_vec tmp = {m_sptr_interface->m_Tir_omega_fl, m_sptr_interface->m_Tir_Pz_fl, m_sptr_interface->m_Tir_vz_fl, m_sptr_interface->m_Tir_rhoz_fl, m_sptr_interface->m_Tir_rhoz_fl, m_sptr_interface->m_Tir_Re_fl, m_sptr_interface->m_Brk_Trq_fl};
		m_outputs.push_back(tmp);
		m_inputs.read_row(m_sptr_interface->m_Brk_Prs_fl, m_sptr_interface->m_Axl_Trq_fl, m_sptr_interface->m_Tir_Fx_fl, m_sptr_interface->m_Tir_My_fl, m_sptr_interface->m_Gnd_Pz_fl, m_sptr_interface->m_Sus_Fz_fl, m_sptr_interface->m_Tir_Fz_fl); 	//*
		m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_con_states,t,m_t_step);
		
		
		t += m_t_step;
		//spin(m_steps);
	}
	m_times.push_back(t);
	m_sptr_sys->pull_con_states(m_sptr_sys->m_con_states);
	m_sptr_sys->update_pv();
	m_sptr_sys->update_fm();
	d_vec tmp1 = {m_sptr_interface->m_Tir_omega_fl, m_sptr_interface->m_Tir_Pz_fl, m_sptr_interface->m_Tir_vz_fl, m_sptr_interface->m_Tir_rhoz_fl, m_sptr_interface->m_Tir_rhoz_fl, m_sptr_interface->m_Tir_Re_fl, m_sptr_interface->m_Brk_Trq_fl};
	m_outputs.push_back(tmp1);
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