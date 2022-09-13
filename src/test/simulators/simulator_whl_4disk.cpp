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
	m_sptr_interface = make_shared<Int_Whl_4Disk>(int(Sys_Whl_4Disk::m_external_inputs_num));
	shared_ptr<Subsys_Wheel_4Disk> sptr_sub_whl_4disk = make_shared<Subsys_Wheel_4Disk>();
    shared_ptr<Wheel_Disk> sptr_whl_fl = make_shared<Wheel_Disk>("data/params/whl_disk_par_0.json");
	shared_ptr<Wheel_Disk> sptr_whl_fr = make_shared<Wheel_Disk>("data/params/whl_disk_par_0.json");
	shared_ptr<Wheel_Disk> sptr_whl_rl = make_shared<Wheel_Disk>("data/params/whl_disk_par_0.json");
	shared_ptr<Wheel_Disk> sptr_whl_rr = make_shared<Wheel_Disk>("data/params/whl_disk_par_0.json");
 
    sptr_sub_whl_4disk->add_whls(sptr_whl_fl,sptr_whl_fr,sptr_whl_rl,sptr_whl_rr);
	m_sptr_sys->add_subsys_whl_4disk(sptr_sub_whl_4disk);
	m_sptr_sys->add_interface(m_sptr_interface);
    
    m_external_inputs = d_vec(Sys_Whl_4Disk::m_external_inputs_num,0.0);
}

void Simulator_Whl_4Disk::run () {
	io::CSVReader<2> m_inputs("data/inputs/whl_4disk_inputs.csv");

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	double t = m_t_start;

	m_tp_start = steady_clock::now();
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		m_inputs.read_row(m_external_inputs[2],m_external_inputs[1]); 
		m_sptr_sys->pull_external_inputs (m_external_inputs);
		m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_con_states,t,m_t_step);
		m_outputs.push_back(m_sptr_interface->m_sub_whl_4disk_pv_outputs);
		
		t += m_t_step;
		//spin(m_steps);
	}
	m_times.push_back(t);
	m_sptr_sys->pull_con_states(m_sptr_sys->m_con_states);
	m_sptr_sys->update_pv();
	m_sptr_sys->update_fm();
	m_outputs.push_back(m_sptr_interface->m_sub_whl_4disk_pv_outputs);
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