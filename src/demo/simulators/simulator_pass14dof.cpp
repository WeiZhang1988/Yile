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
#include "simulator_pass14dof.hpp"

Simulator_Pass14DOF::Simulator_Pass14DOF(string par_file, real_Y t_start, real_Y t_end, real_Y t_step) {
	m_par_file = par_file;
	m_steps         = 0;
	m_t_start       = t_start;
	m_t_end         = t_end;
	m_t_step        = t_step;
	m_t_step_micros = m_t_step/1e-6;
	m_t_current     = m_t_start;
	m_sptr_sys          = make_shared<Sys_Chassis_2Ind_Disk_Fiala>();
	m_sptr_store        = make_shared<d_v_vec>();
	m_sptr_interface    = make_shared<Int_Chassis_2Ind_Disk_Fiala>();
    shared_ptr<Vehicle_Body> sptr_vhl_bdy;
    shared_ptr<Subsys_Wheel_4Disk> sptr_sub_whl_4disk;
	shared_ptr<Subsys_Sus_2Ind> sptr_sub_sus_2ind;
    shared_ptr<Subsys_Tire_4Fiala> sptr_sub_tir_4fiala;

	if (std::filesystem::exists(m_par_file)) {
		pt::ptree tree, \
		pt_parameters, \
		pt_vehicle_body, \
		pt_wheels, \
		pt_front_two_wheels, \
		pt_rear_two_wheels, \
		pt_suspensions, \
		pt_front_suspension, \
		pt_rear_suspension, \
		pt_tires, \
		pt_front_two_tires, \
		pt_rear_two_tires;
		pt::read_json(m_par_file,tree);
		pt_parameters = tree.get_child("parameters");
		
		//create vehicle body
		pt_vehicle_body = pt_parameters.get_child("vehicle_body");
		sptr_vhl_bdy = make_shared<Vehicle_Body>(\
		pt_vehicle_body.get<real_Y>("mass"), \
		pt_vehicle_body.get<real_Y>("a"),   pt_vehicle_body.get<real_Y>("b"),   pt_vehicle_body.get<real_Y>("d"), pt_vehicle_body.get<real_Y>("h"), \
		pt_vehicle_body.get<real_Y>("Ixx"), pt_vehicle_body.get<real_Y>("Ixy"), pt_vehicle_body.get<real_Y>("Ixz"), \
		pt_vehicle_body.get<real_Y>("Iyx"), pt_vehicle_body.get<real_Y>("Iyy"), pt_vehicle_body.get<real_Y>("Iyz"), \
		pt_vehicle_body.get<real_Y>("Izx"), pt_vehicle_body.get<real_Y>("Izy"), pt_vehicle_body.get<real_Y>("Izz"), \
		pt_vehicle_body.get<real_Y>("w_f"), pt_vehicle_body.get<real_Y>("w_r"), \
		
		pt_vehicle_body.get<real_Y>("z1m"), \
		pt_vehicle_body.get<real_Y>("z1R_x"),   pt_vehicle_body.get<real_Y>("z1R_y"),  pt_vehicle_body.get<real_Y>("z1R_z"), \
		pt_vehicle_body.get<real_Y>("z1I_xx"),  pt_vehicle_body.get<real_Y>("z1I_xy"), pt_vehicle_body.get<real_Y>("z1I_xz"), \
		pt_vehicle_body.get<real_Y>("z1I_yx"),  pt_vehicle_body.get<real_Y>("z1I_yy"), pt_vehicle_body.get<real_Y>("z1I_yz"), \
		pt_vehicle_body.get<real_Y>("z1I_zx"),  pt_vehicle_body.get<real_Y>("z1I_zy"), pt_vehicle_body.get<real_Y>("z1I_zz"), \
		
		pt_vehicle_body.get<real_Y>("z2m"), \
		pt_vehicle_body.get<real_Y>("z2R_x"),   pt_vehicle_body.get<real_Y>("z2R_y"),  pt_vehicle_body.get<real_Y>("z2R_z"), \
		pt_vehicle_body.get<real_Y>("z2I_xx"),  pt_vehicle_body.get<real_Y>("z2I_xy"), pt_vehicle_body.get<real_Y>("z2I_xz"), \
		pt_vehicle_body.get<real_Y>("z2I_yx"),  pt_vehicle_body.get<real_Y>("z2I_yy"), pt_vehicle_body.get<real_Y>("z2I_yz"), \
		pt_vehicle_body.get<real_Y>("z2I_zx"),  pt_vehicle_body.get<real_Y>("z2I_zy"), pt_vehicle_body.get<real_Y>("z2I_zz"), \
		
		pt_vehicle_body.get<real_Y>("z3m"), \
		pt_vehicle_body.get<real_Y>("z3R_x"),   pt_vehicle_body.get<real_Y>("z3R_y"),  pt_vehicle_body.get<real_Y>("z3R_z"), \
		pt_vehicle_body.get<real_Y>("z3I_xx"),  pt_vehicle_body.get<real_Y>("z3I_xy"), pt_vehicle_body.get<real_Y>("z3I_xz"), \
		pt_vehicle_body.get<real_Y>("z3I_yx"),  pt_vehicle_body.get<real_Y>("z3I_yy"), pt_vehicle_body.get<real_Y>("z3I_yz"), \
		pt_vehicle_body.get<real_Y>("z3I_zx"),  pt_vehicle_body.get<real_Y>("z3I_zy"), pt_vehicle_body.get<real_Y>("z3I_zz"), \
		
		pt_vehicle_body.get<real_Y>("z4m"), \
		pt_vehicle_body.get<real_Y>("z4R_x"),   pt_vehicle_body.get<real_Y>("z4R_y"),  pt_vehicle_body.get<real_Y>("z4R_z"), \
		pt_vehicle_body.get<real_Y>("z4I_xx"),  pt_vehicle_body.get<real_Y>("z4I_xy"), pt_vehicle_body.get<real_Y>("z4I_xz"), \
		pt_vehicle_body.get<real_Y>("z4I_yx"),  pt_vehicle_body.get<real_Y>("z4I_yy"), pt_vehicle_body.get<real_Y>("z4I_yz"), \
		pt_vehicle_body.get<real_Y>("z4I_zx"),  pt_vehicle_body.get<real_Y>("z4I_zy"), pt_vehicle_body.get<real_Y>("z4I_zz"), \
		
		pt_vehicle_body.get<real_Y>("z5m"), \
		pt_vehicle_body.get<real_Y>("z5R_x"),   pt_vehicle_body.get<real_Y>("z5R_y"),  pt_vehicle_body.get<real_Y>("z5R_z"), \
		pt_vehicle_body.get<real_Y>("z5I_xx"),  pt_vehicle_body.get<real_Y>("z5I_xy"), pt_vehicle_body.get<real_Y>("z5I_xz"), \
		pt_vehicle_body.get<real_Y>("z5I_yx"),  pt_vehicle_body.get<real_Y>("z5I_yy"), pt_vehicle_body.get<real_Y>("z5I_yz"), \
		pt_vehicle_body.get<real_Y>("z5I_zx"),  pt_vehicle_body.get<real_Y>("z5I_zy"), pt_vehicle_body.get<real_Y>("z5I_zz"), \
		
		pt_vehicle_body.get<real_Y>("z6m"), \
		pt_vehicle_body.get<real_Y>("z6R_x"),   pt_vehicle_body.get<real_Y>("z6R_y"),  pt_vehicle_body.get<real_Y>("z6R_z"), \
		pt_vehicle_body.get<real_Y>("z6I_xx"),  pt_vehicle_body.get<real_Y>("z6I_xy"), pt_vehicle_body.get<real_Y>("z6I_xz"), \
		pt_vehicle_body.get<real_Y>("z6I_yx"),  pt_vehicle_body.get<real_Y>("z6I_yy"), pt_vehicle_body.get<real_Y>("z6I_yz"), \
		pt_vehicle_body.get<real_Y>("z6I_zx"),  pt_vehicle_body.get<real_Y>("z6I_zy"), pt_vehicle_body.get<real_Y>("z6I_zz"), \
		
		pt_vehicle_body.get<real_Y>("z7m"), \
		pt_vehicle_body.get<real_Y>("z7R_x"),   pt_vehicle_body.get<real_Y>("z7R_y"),  pt_vehicle_body.get<real_Y>("z7R_z"), \
		pt_vehicle_body.get<real_Y>("z7I_xx"),  pt_vehicle_body.get<real_Y>("z7I_xy"), pt_vehicle_body.get<real_Y>("z7I_xz"), \
		pt_vehicle_body.get<real_Y>("z7I_yx"),  pt_vehicle_body.get<real_Y>("z7I_yy"), pt_vehicle_body.get<real_Y>("z7I_yz"), \
		pt_vehicle_body.get<real_Y>("z7I_zx"),  pt_vehicle_body.get<real_Y>("z7I_zy"), pt_vehicle_body.get<real_Y>("z7I_zz"), \

		pt_vehicle_body.get<real_Y>("Pabs"), pt_vehicle_body.get<real_Y>("Cg"), pt_vehicle_body.get<real_Y>("Af"), \
		pt_vehicle_body.get<real_Y>("Cd"), pt_vehicle_body.get<real_Y>("Cl"), pt_vehicle_body.get<real_Y>("Cpm"), \
		read_json_list(pt_vehicle_body.get_child("beta_w")), \
		read_json_list(pt_vehicle_body.get_child("Cs")), \
		read_json_list(pt_vehicle_body.get_child("Cym")), \
	
		pt_vehicle_body.get<real_Y>("xdot_tol"), \
		pt_vehicle_body.get<real_Y>("longOff"), pt_vehicle_body.get<real_Y>("latOff"), pt_vehicle_body.get<real_Y>("vertOff"), \
	
		pt_vehicle_body.get<real_Y>("init_xe_x"), pt_vehicle_body.get<real_Y>("init_xe_y"), pt_vehicle_body.get<real_Y>("init_xe_z"), \
		pt_vehicle_body.get<real_Y>("init_vb_x"), pt_vehicle_body.get<real_Y>("init_vb_y"), pt_vehicle_body.get<real_Y>("init_vb_z"), \
		pt_vehicle_body.get<real_Y>("init_phai"), pt_vehicle_body.get<real_Y>("init_theta"), pt_vehicle_body.get<real_Y>("init_psi"), \
		pt_vehicle_body.get<real_Y>("init_p"), pt_vehicle_body.get<real_Y>("init_q"), pt_vehicle_body.get<real_Y>("init_r")
		);

		//create wheels
		pt_wheels = pt_parameters.get_child("wheels");
		pt_front_two_wheels = pt_wheels.get_child("front_two_wheels");
		pt_rear_two_wheels = pt_wheels.get_child("rear_two_wheels");
		sptr_sub_whl_4disk = make_shared<Subsys_Wheel_4Disk>(
		pt_front_two_wheels.get<real_Y>("unloaded_radius_f"), pt_front_two_wheels.get<real_Y>("IYY_f"), pt_front_two_wheels.get<real_Y>("mass_f"),\
		pt_front_two_wheels.get<real_Y>("br_f"), pt_front_two_wheels.get<real_Y>("disk_abore_f"), pt_front_two_wheels.get<real_Y>("num_pads_f"), \
		pt_front_two_wheels.get<real_Y>("Rm_f"), pt_front_two_wheels.get<real_Y>("mu_kinetic_f"), pt_front_two_wheels.get<real_Y>("mu_static_f"), \
		pt_front_two_wheels.get<real_Y>("init_omega_f"), pt_front_two_wheels.get<real_Y>("init_Pz_f"), pt_front_two_wheels.get<real_Y>("init_vz_f"), \
		pt_front_two_wheels.get<bool>("init_locked_flag_f"), \
		pt_front_two_wheels.get<bool>("init_locked_state_f"), \
		pt_rear_two_wheels.get<real_Y>("unloaded_radius_r"), pt_rear_two_wheels.get<real_Y>("IYY_r"), pt_rear_two_wheels.get<real_Y>("mass_r"),\
		pt_rear_two_wheels.get<real_Y>("br_r"), pt_rear_two_wheels.get<real_Y>("disk_abore_r"), pt_rear_two_wheels.get<real_Y>("num_pads_r"), \
		pt_rear_two_wheels.get<real_Y>("Rm_r"), pt_rear_two_wheels.get<real_Y>("mu_kinetic_r"), pt_rear_two_wheels.get<real_Y>("mu_static_r"), \
		pt_rear_two_wheels.get<real_Y>("init_omega_r"), pt_rear_two_wheels.get<real_Y>("init_Pz_r"), pt_rear_two_wheels.get<real_Y>("init_vz_r"), \
		pt_rear_two_wheels.get<bool>("init_locked_flag_r"), \
		pt_rear_two_wheels.get<bool>("init_locked_state_r")
		);

		//create suspensions
		pt_suspensions = pt_parameters.get_child("suspensions");
		pt_front_suspension = pt_suspensions.get_child("front_suspension");
		pt_rear_suspension = pt_suspensions.get_child("rear_suspension");
		sptr_sub_sus_2ind = make_shared<Subsys_Sus_2Ind>(
		pt_front_suspension.get<real_Y>("F0z_f"),    	  pt_front_suspension.get<real_Y>("Kz_f"), 				pt_front_suspension.get<real_Y>("Cz_f"), \
		pt_front_suspension.get<real_Y>("Hmax_f"),   	  pt_front_suspension.get<real_Y>("roll_strg_H_slp_f"), \
		pt_front_suspension.get<real_Y>("toe_f"),    	  pt_front_suspension.get<real_Y>("toe_strg_slp_f"), \
		pt_front_suspension.get<real_Y>("caster_f"), 	  pt_front_suspension.get<real_Y>("caster_H_slp_f"), 	pt_front_suspension.get<real_Y>("caster_strg_slp_f"), \
		pt_front_suspension.get<real_Y>("camber_f"), 	  pt_front_suspension.get<real_Y>("camber_H_slp_f"), 	pt_front_suspension.get<real_Y>("camber_strg_slp_f"), \
		pt_front_suspension.get<real_Y>("strg_hgt_slp_f"),\
		pt_front_suspension.get<real_Y>("as_R_f"),   	  pt_front_suspension.get<real_Y>("as_ntrl_ang_f"), 	pt_front_suspension.get<real_Y>("as_trsK_f"), \
		pt_front_suspension.get<bool>("has_anti_sway_f"), pt_front_suspension.get<bool>("is_strg_f"), \
		
		pt_rear_suspension.get<real_Y>("F0z_r"),    	  pt_rear_suspension.get<real_Y>("Kz_r"), 				pt_rear_suspension.get<real_Y>("Cz_r"), \
		pt_rear_suspension.get<real_Y>("Hmax_r"),   	  pt_rear_suspension.get<real_Y>("roll_strg_H_slp_r"), \
		pt_rear_suspension.get<real_Y>("toe_r"),    	  pt_rear_suspension.get<real_Y>("toe_strg_slp_r"), \
		pt_rear_suspension.get<real_Y>("caster_r"), 	  pt_rear_suspension.get<real_Y>("caster_H_slp_r"), 	pt_rear_suspension.get<real_Y>("caster_strg_slp_r"), \
		pt_rear_suspension.get<real_Y>("camber_r"), 	  pt_rear_suspension.get<real_Y>("camber_H_slp_r"), 	pt_rear_suspension.get<real_Y>("camber_strg_slp_r"), \
		pt_rear_suspension.get<real_Y>("strg_hgt_slp_r"),\
		pt_rear_suspension.get<real_Y>("as_R_r"), 		  pt_rear_suspension.get<real_Y>("as_ntrl_ang_r"), 		pt_rear_suspension.get<real_Y>("as_trsK_r"), \
		pt_rear_suspension.get<bool>("has_anti_sway_r"),  pt_rear_suspension.get<bool>("is_strg_r")
		);

		//create tires
		pt_tires = pt_parameters.get_child("tires");
		pt_front_two_tires = pt_tires.get_child("front_two_tires");
		pt_rear_two_tires = pt_tires.get_child("rear_two_tires");
		sptr_sub_tir_4fiala  = make_shared<Subsys_Tire_4Fiala>(
		pt_front_two_tires.get<real_Y>("Lrelx_f"),      pt_front_two_tires.get<real_Y>("Lrely_f"), \
		pt_front_two_tires.get<real_Y>("alpha_min_f"),  pt_front_two_tires.get<real_Y>("alpha_max_f"), \
		pt_front_two_tires.get<real_Y>("mu_min_f"),     pt_front_two_tires.get<real_Y>("mu_max_f"), \
		pt_front_two_tires.get<real_Y>("aMy_f"), 	    pt_front_two_tires.get<real_Y>("bMy_f"), 	   pt_front_two_tires.get<real_Y>("cMy_f"), \
		pt_front_two_tires.get<real_Y>("alphaMy_f"),    pt_front_two_tires.get<real_Y>("betaMy_f"), \
		pt_front_two_tires.get<real_Y>("Fz_min_f"),     pt_front_two_tires.get<real_Y>("Fz_max_f"),    pt_front_two_tires.get<real_Y>("cKappa_f"), \
		pt_front_two_tires.get<real_Y>("cAlpha_f"),     pt_front_two_tires.get<real_Y>("bMz_f"), 	   pt_front_two_tires.get<real_Y>("width_f"), \
		pt_front_two_tires.get<real_Y>("cGamma_f"), \
		pt_front_two_tires.get<real_Y>("init_kappa_f"), pt_front_two_tires.get<real_Y>("init_alpha_f"), \
		pt_front_two_tires.get<real_Y>("init_Mroll_f"), \
		
		pt_rear_two_tires.get<real_Y>("Lrelx_r"),      pt_rear_two_tires.get<real_Y>("Lrely_r"), \
		pt_rear_two_tires.get<real_Y>("alpha_min_r"),  pt_rear_two_tires.get<real_Y>("alpha_max_r"), \
		pt_rear_two_tires.get<real_Y>("mu_min_r"),     pt_rear_two_tires.get<real_Y>("mu_max_r"), \
		pt_rear_two_tires.get<real_Y>("aMy_r"), 	   pt_rear_two_tires.get<real_Y>("bMy_r"), 	   	   pt_rear_two_tires.get<real_Y>("cMy_r"), \
		pt_rear_two_tires.get<real_Y>("alphaMy_r"),    pt_rear_two_tires.get<real_Y>("betaMy_r"), \
		pt_rear_two_tires.get<real_Y>("Fz_min_r"),     pt_rear_two_tires.get<real_Y>("Fz_max_r"),  	   pt_rear_two_tires.get<real_Y>("cKappa_r"), \
		pt_rear_two_tires.get<real_Y>("cAlpha_r"),     pt_rear_two_tires.get<real_Y>("bMz_r"), 	   	   pt_rear_two_tires.get<real_Y>("width_r"), \
		pt_rear_two_tires.get<real_Y>("cGamma_r"), \
		pt_rear_two_tires.get<real_Y>("init_kappa_r"), pt_rear_two_tires.get<real_Y>("init_alpha_r"), \
		pt_rear_two_tires.get<real_Y>("init_Mroll_r")
		);
	} else {
		sptr_vhl_bdy         = make_shared<Vehicle_Body>();
		sptr_sub_whl_4disk   = make_shared<Subsys_Wheel_4Disk>();
		sptr_sub_sus_2ind    = make_shared<Subsys_Sus_2Ind>();
		sptr_sub_tir_4fiala  = make_shared<Subsys_Tire_4Fiala>();
	}
 
    m_sptr_sys->add_vhl_bdy(sptr_vhl_bdy);
	m_sptr_sys->add_subsys_whl_4disk(sptr_sub_whl_4disk);
	m_sptr_sys->add_subsys_sus_2ind(sptr_sub_sus_2ind);
	m_sptr_sys->add_subsys_tir_4fiala(sptr_sub_tir_4fiala);
    m_sptr_sys->add_interface(m_sptr_interface);
    m_sptr_sys->add_store(m_sptr_store);
}

void Simulator_Pass14DOF::spin (const int &steps) {
	m_tp_end = steady_clock::now();
	microseconds dur_micros = duration_cast<microseconds>(m_tp_end - m_tp_start);
	int dur_micros_cnt = dur_micros.count();
	int dur_micros_cnt_trgt = steps * static_cast<int>(m_t_step_micros);
	if (dur_micros_cnt<dur_micros_cnt_trgt) {
		this_thread::sleep_for(microseconds(dur_micros_cnt_trgt - dur_micros_cnt));
	}
}

d_vec Simulator_Pass14DOF::read_json_list(const pt::ptree &in) {
	d_vec out;
	for (auto item : in) {
		out.push_back(item.second.get<real_Y>(""));
	}
	return out;
}

void Simulator_Pass14DOF::udp_pull() {
	
	while(!m_simulation_done)  
	{  
		m_udp_pull_server.get_request(m_udp_pull_recv_buf,sizeof(m_udp_pull_recv_buf)); 
		std::unique_lock<std::mutex> locker(m_mu); 
		m_sptr_interface->m_Strg_str_fl  = m_udp_pull_recv_buf[0];
		m_sptr_interface->m_Strg_str_fr  = m_udp_pull_recv_buf[1];
		m_sptr_interface->m_Strg_str_rl  = m_udp_pull_recv_buf[2];
		m_sptr_interface->m_Strg_str_rr  = m_udp_pull_recv_buf[3];
		m_sptr_interface->m_Axl_Trq_fl   = m_udp_pull_recv_buf[4];
		m_sptr_interface->m_Axl_Trq_fr   = m_udp_pull_recv_buf[5];
		m_sptr_interface->m_Axl_Trq_rl   = m_udp_pull_recv_buf[6];
		m_sptr_interface->m_Axl_Trq_rr   = m_udp_pull_recv_buf[7];
		m_sptr_interface->m_Brk_Prs_fl   = m_udp_pull_recv_buf[8];
		m_sptr_interface->m_Brk_Prs_fr   = m_udp_pull_recv_buf[9];
		m_sptr_interface->m_Brk_Prs_rl   = m_udp_pull_recv_buf[10];
		m_sptr_interface->m_Brk_Prs_rr   = m_udp_pull_recv_buf[11];
		m_sptr_interface->m_Air_Wx 		 = m_udp_pull_recv_buf[12];
		m_sptr_interface->m_Air_Wy 		 = m_udp_pull_recv_buf[13];
		m_sptr_interface->m_Air_Wz 		 = m_udp_pull_recv_buf[14];
		m_sptr_interface->m_Gnd_Pz_fl    = m_udp_pull_recv_buf[15];
		m_sptr_interface->m_Gnd_Pz_fr    = m_udp_pull_recv_buf[16];
		m_sptr_interface->m_Gnd_Pz_rl    = m_udp_pull_recv_buf[17];
		m_sptr_interface->m_Gnd_Pz_rr    = m_udp_pull_recv_buf[18];
		m_sptr_interface->m_Gnd_scale_fl = m_udp_pull_recv_buf[19];
		m_sptr_interface->m_Gnd_scale_fr = m_udp_pull_recv_buf[20];
		m_sptr_interface->m_Gnd_scale_rl = m_udp_pull_recv_buf[21];
		m_sptr_interface->m_Gnd_scale_rr = m_udp_pull_recv_buf[22];
		m_sptr_interface->m_Tir_Prs_fl   = m_udp_pull_recv_buf[23];
		m_sptr_interface->m_Tir_Prs_fr   = m_udp_pull_recv_buf[24];
		m_sptr_interface->m_Tir_Prs_rl   = m_udp_pull_recv_buf[25];
		m_sptr_interface->m_Tir_Prs_rr   = m_udp_pull_recv_buf[26];
		m_sptr_interface->m_Air_Tair     = m_udp_pull_recv_buf[27];
		m_sptr_interface->m_Air_Tamb_fl  = m_udp_pull_recv_buf[28];
		m_sptr_interface->m_Air_Tamb_fr  = m_udp_pull_recv_buf[29];
		m_sptr_interface->m_Air_Tamb_rl  = m_udp_pull_recv_buf[30];
		m_sptr_interface->m_Air_Tamb_rr  = m_udp_pull_recv_buf[31];
		m_sptr_interface->m_Ext_Fx_ext   = m_udp_pull_recv_buf[32];
		m_sptr_interface->m_Ext_Fy_ext   = m_udp_pull_recv_buf[33];
		m_sptr_interface->m_Ext_Fz_ext   = m_udp_pull_recv_buf[34];
		m_sptr_interface->m_Ext_Mx_ext   = m_udp_pull_recv_buf[35]; 
		m_sptr_interface->m_Ext_My_ext   = m_udp_pull_recv_buf[36];
		m_sptr_interface->m_Ext_Mz_ext   = m_udp_pull_recv_buf[37];
		locker.unlock();

		m_enable_do_sim = true;
		if (m_enable_output) {
			m_udp_pull_send_buf[0] = m_sptr_interface->m_xe_x_c;
			m_udp_pull_send_buf[1] = m_sptr_interface->m_xe_y_c;
			m_udp_pull_send_buf[2] = m_sptr_interface->m_xe_z_c;
			m_udp_pull_send_buf[3] = m_sptr_interface->m_phai_c;
			m_udp_pull_send_buf[4] = m_sptr_interface->m_theta_c;
			m_udp_pull_send_buf[5] = m_sptr_interface->m_psi_c;

			m_udp_pull_server.respond(m_udp_pull_send_buf,sizeof(m_udp_pull_send_buf));
		}
		
	}  
    
}

void Simulator_Pass14DOF::udp_push() {
	while(!m_simulation_done)  
	{  
		m_udp_push_server.get_request(m_udp_push_recv_buf,sizeof(m_udp_push_recv_buf));
		
		m_udp_push_send_buf[0] = m_sptr_interface->m_xe_x_c;
		m_udp_push_send_buf[1] = m_sptr_interface->m_xe_y_c;
		m_udp_push_send_buf[2] = m_sptr_interface->m_xe_z_c;
		m_udp_push_send_buf[3] = m_sptr_interface->m_phai_c;
		m_udp_push_send_buf[4] = m_sptr_interface->m_theta_c;
		m_udp_push_send_buf[5] = m_sptr_interface->m_psi_c;

		m_udp_push_server.respond(m_udp_push_send_buf,sizeof(m_udp_push_send_buf));
	}   
}


void Simulator_Pass14DOF::do_simulation() {
	
	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
	real_Y t = m_t_start;
	while(!m_enable_do_sim);
	m_sptr_sys->push_con_states(m_sptr_sys->m_con_states);
	m_tp_start = steady_clock::now();
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		std::unique_lock<std::mutex> locker(m_mu); 
		m_sptr_sys->push_con_states_veh_whl(m_sptr_sys->m_con_states);
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_con_states,t,m_t_step);
		m_sptr_sys->pull_con_states_veh_whl(m_sptr_sys->m_con_states);
		locker.unlock();
		m_enable_output = true;
		t += m_t_step;
		spin(m_steps);
		if (i%1000==0)
			std::cout<<"run "<< i << " steps"<<std::endl;
	}
	m_times.push_back(t);
	m_sptr_sys->pull_con_states(m_sptr_sys->m_con_states);
	m_sptr_sys->update_pv();
	m_sptr_sys->update_fm();
	m_sptr_sys->store_data();
	m_simulation_done = true;
	//exit(0);
}

void Simulator_Pass14DOF::run () {
	std::thread t1(&Simulator_Pass14DOF::udp_pull,this);
	std::thread t2(&Simulator_Pass14DOF::do_simulation,this);
	//std::thread t3(&Simulator_Pass14DOF::udp_push,this);
	t1.join();
	t2.join();
	//t3.join();
}