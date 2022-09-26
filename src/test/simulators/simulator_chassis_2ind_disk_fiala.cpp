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
#include "simulator_chassis_2ind_disk_fiala.hpp"

Simulator_Chassis_2Ind_Disk_Fiala::Simulator_Chassis_2Ind_Disk_Fiala(double t_start, double t_end, double t_step) {
	m_steps         = 0;
	m_t_start       = t_start;
	m_t_end         = t_end;
	m_t_step        = t_step;
	m_t_step_micros = m_t_step/1e-6;

	m_sptr_sys          = make_shared<Sys_Chassis_2Ind_Disk_Fiala>();
	m_sptr_store        = make_shared<d_v_vec>();
	m_sptr_interface    = make_shared<Int_Chassis_2Ind_Disk_Fiala>();
    shared_ptr<Vehicle_Body> sptr_vhl_bdy               = make_shared<Vehicle_Body>();
    shared_ptr<Subsys_Wheel_4Disk> sptr_sub_whl_4disk   = make_shared<Subsys_Wheel_4Disk>();
    shared_ptr<Wheel_Disk> sptr_whl_fl                  = make_shared<Wheel_Disk>();
	shared_ptr<Wheel_Disk> sptr_whl_fr                  = make_shared<Wheel_Disk>();
	shared_ptr<Wheel_Disk> sptr_whl_rl                  = make_shared<Wheel_Disk>();
	shared_ptr<Wheel_Disk> sptr_whl_rr                  = make_shared<Wheel_Disk>();
	shared_ptr<Subsys_Sus_2Ind> sptr_sub_sus_2ind       = make_shared<Subsys_Sus_2Ind>();
    shared_ptr<Sus_Ind_2Tracks> sptr_sus_f              = make_shared<Sus_Ind_2Tracks>();
	shared_ptr<Sus_Ind_2Tracks> sptr_sus_r              = make_shared<Sus_Ind_2Tracks>();
    shared_ptr<Subsys_Tire_4Fiala> sptr_sub_tir_4fiala  = make_shared<Subsys_Tire_4Fiala>();
    shared_ptr<Tire_Fiala> sptr_tir_fl                  = make_shared<Tire_Fiala>();
	shared_ptr<Tire_Fiala> sptr_tir_fr                  = make_shared<Tire_Fiala>();
	shared_ptr<Tire_Fiala> sptr_tir_rl                  = make_shared<Tire_Fiala>();
	shared_ptr<Tire_Fiala> sptr_tir_rr                  = make_shared<Tire_Fiala>();
 
    m_sptr_sys->add_vhl_bdy(sptr_vhl_bdy);
    sptr_sub_whl_4disk->add_whls(sptr_whl_fl,sptr_whl_fr,sptr_whl_rl,sptr_whl_rr);
	m_sptr_sys->add_subsys_whl_4disk(sptr_sub_whl_4disk);
    sptr_sub_sus_2ind->add_suses(sptr_sus_f, sptr_sus_r);
	m_sptr_sys->add_subsys_sus_2ind(sptr_sub_sus_2ind);
    sptr_sub_tir_4fiala->add_tirs(sptr_tir_fl,sptr_tir_fr,sptr_tir_rl,sptr_tir_rr);
	m_sptr_sys->add_subsys_tir_4fiala(sptr_sub_tir_4fiala);

    m_sptr_sys->add_interface(m_sptr_interface);
    m_sptr_sys->add_store(m_sptr_store);
}

void Simulator_Chassis_2Ind_Disk_Fiala::run () {
	io::CSVReader<38> m_inputs("sus_test_result/sus_2ind_ramp_inputs.csv"); //need modification

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	double t = m_t_start;

	m_tp_start = steady_clock::now();
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		m_inputs.read_row(m_sptr_interface->m_Gnd_Pz_fl, m_sptr_interface->m_Gnd_Pz_fr, m_sptr_interface->m_Gnd_Pz_rl, m_sptr_interface->m_Gnd_Pz_rr, \
    	m_sptr_interface->m_Gnd_scale_fl, m_sptr_interface->m_Tir_Prs_fl, m_sptr_interface->m_Air_Tamb_fl, \
        m_sptr_interface->m_Gnd_scale_fr, m_sptr_interface->m_Tir_Prs_fr, m_sptr_interface->m_Air_Tamb_fr, \
        m_sptr_interface->m_Gnd_scale_rl, m_sptr_interface->m_Tir_Prs_rl, m_sptr_interface->m_Air_Tamb_rl, \
        m_sptr_interface->m_Gnd_scale_rr, m_sptr_interface->m_Tir_Prs_rr, m_sptr_interface->m_Air_Tamb_rr, \
        m_sptr_interface->m_Air_Wx, m_sptr_interface->m_Air_Wy, m_sptr_interface->m_Air_Wz, \
        m_sptr_interface->m_Air_Tair, \
        m_sptr_interface->m_Ext_Fx_ext, m_sptr_interface->m_Ext_Fy_ext, m_sptr_interface->m_Ext_Fz_ext, \
        m_sptr_interface->m_Ext_Mx_ext, m_sptr_interface->m_Ext_My_ext, m_sptr_interface->m_Ext_Mz_ext, \
        m_sptr_interface->m_Axl_Trq_fl, m_sptr_interface->m_Brk_Prs_fl, m_sptr_interface->m_Axl_Trq_fr, m_sptr_interface->m_Brk_Prs_fr, \
        m_sptr_interface->m_Axl_Trq_rl, m_sptr_interface->m_Brk_Prs_rl, m_sptr_interface->m_Axl_Trq_rr, m_sptr_interface->m_Brk_Prs_rr, \
        m_sptr_interface->m_Strg_str_fl, m_sptr_interface->m_Strg_str_fr, m_sptr_interface->m_Strg_str_rl, m_sptr_interface->m_Strg_str_rr); 
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

void Simulator_Chassis_2Ind_Disk_Fiala::spin (const int &steps) {
	m_tp_end = steady_clock::now();
	microseconds dur_micros = duration_cast<microseconds>(m_tp_end - m_tp_start);
	int dur_micros_cnt = dur_micros.count();
	int dur_micros_cnt_trgt = steps * static_cast<int>(m_t_step_micros);
	if (dur_micros_cnt<dur_micros_cnt_trgt) {
	this_thread::sleep_for(microseconds(dur_micros_cnt_trgt - dur_micros_cnt));
	}
}