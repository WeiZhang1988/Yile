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
#include "simulator_sus_wheel_tire_2ind_disk_fiala.hpp"

Simulator_Sus_Wheel_Tire_2Ind_Disk_Fiala::Simulator_Sus_Wheel_Tire_2Ind_Disk_Fiala(real_Y t_start, real_Y t_end, real_Y t_step) {
	m_steps         = 0;
	m_t_start       = t_start;
	m_t_end         = t_end;
	m_t_step        = t_step;
	m_t_step_micros = m_t_step/1e-6;

	m_sptr_sys          = make_shared<Sys_Sus_Wheel_Tire_2Ind_Disk_Fiala>();
	m_sptr_store        = make_shared<d_v_vec>();
	m_sptr_interface    = make_shared<Int_Chassis_2Ind_Disk_Fiala>();
    shared_ptr<Subsys_Wheel_4Disk> sptr_sub_whl_4disk   = make_shared<Subsys_Wheel_4Disk>();
	shared_ptr<Subsys_Sus_2Ind> sptr_sub_sus_2ind       = make_shared<Subsys_Sus_2Ind>();
    shared_ptr<Subsys_Tire_4Fiala> sptr_sub_tir_4fiala  = make_shared<Subsys_Tire_4Fiala>();
 
	m_sptr_sys->add_subsys_whl_4disk(sptr_sub_whl_4disk);
	m_sptr_sys->add_subsys_sus_2ind(sptr_sub_sus_2ind);
	m_sptr_sys->add_subsys_tir_4fiala(sptr_sub_tir_4fiala);

    m_sptr_sys->add_interface(m_sptr_interface);
    m_sptr_sys->add_store(m_sptr_store);
}

void Simulator_Sus_Wheel_Tire_2Ind_Disk_Fiala::run () {
	io::CSVReader<50> m_inputs("wheel_tire_sus_test_result/Wheel_tire_sus_inputs10141615.csv"); //need modification

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	real_Y t = m_t_start;

	m_tp_start = steady_clock::now();
	m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		m_inputs.read_row( 
			/*Wheel tire inputs*/
			m_sptr_interface->m_Axl_Trq_fl,
			m_sptr_interface->m_Axl_Trq_fr,
			m_sptr_interface->m_Axl_Trq_rl,
			m_sptr_interface->m_Axl_Trq_rr,

			m_sptr_interface->m_Brk_Prs_fl,
			m_sptr_interface->m_Brk_Prs_fr,
			m_sptr_interface->m_Brk_Prs_rl,
			m_sptr_interface->m_Brk_Prs_rr,

			m_sptr_interface->m_Gnd_Pz_fl,
			m_sptr_interface->m_Gnd_Pz_fr,
			m_sptr_interface->m_Gnd_Pz_rl,
			m_sptr_interface->m_Gnd_Pz_rr,

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
			m_sptr_interface->m_Air_Tamb_rr,

			/*suspension*/

			m_sptr_interface->m_Veh_hgt_cg,
			m_sptr_interface->m_Veh_r,


			m_sptr_interface->m_Int_Pz_fl,
			m_sptr_interface->m_Int_Pz_fr,
			m_sptr_interface->m_Int_Pz_rl,
			m_sptr_interface->m_Int_Pz_rr,

			m_sptr_interface->m_Int_Vz_fl,
			m_sptr_interface->m_Int_Vz_fr,
			m_sptr_interface->m_Int_Vz_rl,
			m_sptr_interface->m_Int_Vz_rr,

			m_sptr_interface->m_Strg_str_fl,
			m_sptr_interface->m_Strg_str_fr,
			m_sptr_interface->m_Strg_str_rl,
			m_sptr_interface->m_Strg_str_rr,

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
			m_sptr_interface->m_Veh_vz_rr

		);
		m_sptr_sys->push_con_states_whl_only(m_sptr_sys->m_con_states);
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_con_states,t,m_t_step);
		m_sptr_sys->pull_con_states_whl_only(m_sptr_sys->m_con_states);

		t += m_t_step;
		//spin(m_steps);
	}
	m_times.push_back(t);
	m_sptr_sys->pull_con_states(m_sptr_sys->m_con_states);
	m_sptr_sys->update_pv();
	m_sptr_sys->update_fm();
	m_sptr_sys->store_data();
}

void Simulator_Sus_Wheel_Tire_2Ind_Disk_Fiala::spin (const int &steps) {
	m_tp_end = steady_clock::now();
	microseconds dur_micros = duration_cast<microseconds>(m_tp_end - m_tp_start);
	int dur_micros_cnt = dur_micros.count();
	int dur_micros_cnt_trgt = steps * static_cast<int>(m_t_step_micros);
	if (dur_micros_cnt<dur_micros_cnt_trgt) {
	this_thread::sleep_for(microseconds(dur_micros_cnt_trgt - dur_micros_cnt));
	}
}