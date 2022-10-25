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
#include "simulator_vehicle_body.hpp"

Simulator_Vehicle_Body::Simulator_Vehicle_Body(real_Y t_start, real_Y t_end, real_Y t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	m_t_step_micros = m_t_step/1e-6;

	m_sptr_sys = make_shared<Sys_Vehicle_Body>();
	m_sptr_store = make_shared<d_v_vec>();
	m_sptr_interface = make_shared<Int_Vehicle_Body>();
	shared_ptr<Vehicle_Body> sptr_vhl_bdy = make_shared<Vehicle_Body>();
 
	m_sptr_sys->add_vhl_bdy(sptr_vhl_bdy);
	m_sptr_sys->add_interface(m_sptr_interface);
	m_sptr_sys->add_store(m_sptr_store);
    
}

void Simulator_Vehicle_Body::run() {
	io::CSVReader<34> m_inputs("veh_test_result/veh_vb_inputs10191035.csv");

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	real_Y t = m_t_start;
	
	m_tp_start = steady_clock::now();
	m_sptr_sys->push_con_states(m_sptr_sys->m_con_states); 
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		m_inputs.read_row(
		m_sptr_interface->m_Air_Wx, 
		m_sptr_interface->m_Air_Wy, 
		m_sptr_interface->m_Air_Wz, 
		m_sptr_interface->m_Air_Tair, \

		m_sptr_interface->m_Sus_VehFx_fl, 
		m_sptr_interface->m_Sus_VehFx_fr, 
		m_sptr_interface->m_Sus_VehFx_rl, 
		m_sptr_interface->m_Sus_VehFx_rr, \

		m_sptr_interface->m_Sus_VehFy_fl, 
		m_sptr_interface->m_Sus_VehFy_fr, 
		m_sptr_interface->m_Sus_VehFy_rl, 
		m_sptr_interface->m_Sus_VehFy_rr, \

		m_sptr_interface->m_Sus_VehFz_fl, 
		m_sptr_interface->m_Sus_VehFz_fr, 
		m_sptr_interface->m_Sus_VehFz_rl, 
		m_sptr_interface->m_Sus_VehFz_rr, \

		m_sptr_interface->m_Sus_VehMx_fl, 
		m_sptr_interface->m_Sus_VehMx_fr, 
		m_sptr_interface->m_Sus_VehMx_rl, 
		m_sptr_interface->m_Sus_VehMx_rr, \

		m_sptr_interface->m_Sus_VehMy_fl, 
		m_sptr_interface->m_Sus_VehMy_fr, 
		m_sptr_interface->m_Sus_VehMy_rl, 
		m_sptr_interface->m_Sus_VehMy_rr, \

		m_sptr_interface->m_Sus_VehMz_fl, 
		m_sptr_interface->m_Sus_VehMz_fr, 
		m_sptr_interface->m_Sus_VehMz_rl, 
		m_sptr_interface->m_Sus_VehMz_rr, \

		m_sptr_interface->m_Ext_Fx_ext, 
		m_sptr_interface->m_Ext_Fy_ext, 
		m_sptr_interface->m_Ext_Fz_ext, \
		m_sptr_interface->m_Ext_Mx_ext, 
		m_sptr_interface->m_Ext_My_ext, 
		m_sptr_interface->m_Ext_Mz_ext
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

void Simulator_Vehicle_Body::spin (const int &steps) {
	m_tp_end = steady_clock::now();
	microseconds dur_micros = duration_cast<microseconds>(m_tp_end - m_tp_start);
	int dur_micros_cnt = dur_micros.count();
	int dur_micros_cnt_trgt = steps * static_cast<int>(m_t_step_micros);
	if (dur_micros_cnt<dur_micros_cnt_trgt) {
		this_thread::sleep_for(microseconds(dur_micros_cnt_trgt - dur_micros_cnt));
	}
}
