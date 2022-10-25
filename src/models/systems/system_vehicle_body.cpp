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
#include "system_vehicle_body.hpp"

void NMSPC::Sys_Vehicle_Body::push_con_states (d_vec &con_states) {
	m_sptr_vhl_bdy->push_con_states(m_vhl_bdy_con_states);
	std::copy(m_vhl_bdy_con_states.begin(), m_vhl_bdy_con_states.end(), con_states.begin());
}

void NMSPC::Sys_Vehicle_Body::pull_con_states (const d_vec &con_states) {
	std::copy(con_states.begin(), con_states.begin() + Vehicle_Body::m_con_states_num, m_vhl_bdy_con_states.begin());
	m_sptr_vhl_bdy->pull_con_states(m_vhl_bdy_con_states);
}

void NMSPC::Sys_Vehicle_Body::update_pv() {
	m_sptr_vhl_bdy->pull_pv(m_sptr_interface->m_Air_Wx, m_sptr_interface->m_Air_Wy, m_sptr_interface->m_Air_Wz);
	m_sptr_vhl_bdy->push_pv(m_sptr_interface->m_Veh_hgt_cg, m_sptr_interface->m_xe_x_c, m_sptr_interface->m_xe_y_c, m_sptr_interface->m_xe_z_c, m_sptr_interface->m_ve_x_c, m_sptr_interface->m_ve_y_c, m_sptr_interface->m_ve_z_c, \
	m_sptr_interface->m_vb_x_c,  m_sptr_interface->m_vb_y_c,  m_sptr_interface->m_vb_z_c,    m_sptr_interface->m_phai_c,    m_sptr_interface->m_theta_c,   m_sptr_interface->m_psi_c,    m_sptr_interface->m_p_c, m_sptr_interface->m_q_c, m_sptr_interface->m_Veh_r, m_sptr_interface->m_beta_c, \
	m_sptr_interface->m_xe_x_o,  m_sptr_interface->m_xe_y_o,  m_sptr_interface->m_xe_z_o,    m_sptr_interface->m_ve_x_o,    m_sptr_interface->m_ve_y_o,    m_sptr_interface->m_ve_z_o, \
	m_sptr_interface->m_xb_x_o,  m_sptr_interface->m_xb_y_o,  m_sptr_interface->m_xb_z_o,    m_sptr_interface->m_vb_x_o,    m_sptr_interface->m_vb_y_o,    m_sptr_interface->m_vb_z_o,   m_sptr_interface->m_beta_o, \
	m_sptr_interface->m_xe_x_fl, m_sptr_interface->m_xe_y_fl, m_sptr_interface->m_Int_Pz_fl,   m_sptr_interface->m_ve_x_fl,   m_sptr_interface->m_ve_y_fl,   m_sptr_interface->m_Int_Vz_fl, \
	m_sptr_interface->m_xb_x_fl, m_sptr_interface->m_xb_y_fl, m_sptr_interface->m_xb_z_fl, m_sptr_interface->m_Veh_vx_fl, m_sptr_interface->m_Veh_vy_fl, m_sptr_interface->m_Veh_vz_fl, \
	m_sptr_interface->m_xe_x_fr, m_sptr_interface->m_xe_y_fr, m_sptr_interface->m_Int_Pz_fr,   m_sptr_interface->m_ve_x_fr,   m_sptr_interface->m_ve_y_fr,   m_sptr_interface->m_Int_Vz_fr, \
	m_sptr_interface->m_xb_x_fr, m_sptr_interface->m_xb_y_fr, m_sptr_interface->m_xb_z_fr, m_sptr_interface->m_Veh_vx_fr, m_sptr_interface->m_Veh_vy_fr, m_sptr_interface->m_Veh_vz_fr, \
	m_sptr_interface->m_xe_x_rl, m_sptr_interface->m_xe_y_rl, m_sptr_interface->m_Int_Pz_rl,   m_sptr_interface->m_ve_x_rl,   m_sptr_interface->m_ve_y_rl,   m_sptr_interface->m_Int_Vz_rl, \
	m_sptr_interface->m_xb_x_rl, m_sptr_interface->m_xb_y_rl, m_sptr_interface->m_xb_z_rl, m_sptr_interface->m_Veh_vx_rl, m_sptr_interface->m_Veh_vy_rl, m_sptr_interface->m_Veh_vz_rl, \
	m_sptr_interface->m_xe_x_rr, m_sptr_interface->m_xe_y_rr, m_sptr_interface->m_Int_Pz_rr,   m_sptr_interface->m_ve_x_rr,   m_sptr_interface->m_ve_y_rr,   m_sptr_interface->m_Int_Vz_rr, \
	m_sptr_interface->m_xb_x_rr, m_sptr_interface->m_xb_y_rr, m_sptr_interface->m_xb_z_rr, m_sptr_interface->m_Veh_vx_rr, m_sptr_interface->m_Veh_vy_rr, m_sptr_interface->m_Veh_vz_rr);
}

void NMSPC::Sys_Vehicle_Body::update_fm() {
	m_sptr_vhl_bdy->pull_fm(m_sptr_interface->m_Air_Tair, \
	m_sptr_interface->m_Sus_VehFx_fl, m_sptr_interface->m_Sus_VehFx_fr, m_sptr_interface->m_Sus_VehFx_rl, m_sptr_interface->m_Sus_VehFx_rr, \
	m_sptr_interface->m_Sus_VehFy_fl, m_sptr_interface->m_Sus_VehFy_fr, m_sptr_interface->m_Sus_VehFy_rl, m_sptr_interface->m_Sus_VehFy_rr, \
	m_sptr_interface->m_Sus_VehFz_fl, m_sptr_interface->m_Sus_VehFz_fr, m_sptr_interface->m_Sus_VehFz_rl, m_sptr_interface->m_Sus_VehFz_rr, \
	m_sptr_interface->m_Sus_VehMx_fl, m_sptr_interface->m_Sus_VehMx_fr, m_sptr_interface->m_Sus_VehMx_rl, m_sptr_interface->m_Sus_VehMx_rr ,\
	m_sptr_interface->m_Sus_VehMy_fl, m_sptr_interface->m_Sus_VehMy_fr, m_sptr_interface->m_Sus_VehMy_rl, m_sptr_interface->m_Sus_VehMy_rr, \
	m_sptr_interface->m_Sus_VehMz_fl, m_sptr_interface->m_Sus_VehMz_fr, m_sptr_interface->m_Sus_VehMz_rl, m_sptr_interface->m_Sus_VehMz_rr, \
	m_sptr_interface->m_Ext_Fx_ext, m_sptr_interface->m_Ext_Fy_ext, m_sptr_interface->m_Ext_Fz_ext, \
	m_sptr_interface->m_Ext_Mx_ext, m_sptr_interface->m_Ext_My_ext, m_sptr_interface->m_Ext_Mz_ext);
	m_sptr_vhl_bdy->push_fm();
}

void NMSPC::Sys_Vehicle_Body::update_drv() {
	m_sptr_vhl_bdy->push_drv(m_vhl_bdy_drvs);
	std::copy(m_vhl_bdy_drvs.begin(), m_vhl_bdy_drvs.end(), m_drvs.begin());

}

void NMSPC::Sys_Vehicle_Body::store_data() {
	m_sptr_store->push_back(d_vec{\

		//m_sptr_interface->m_Veh_hgt_cg, \

		m_sptr_interface->m_vb_x_c, \
		m_sptr_interface->m_vb_y_c, \
		m_sptr_interface->m_vb_z_c, \

		m_sptr_interface->m_p_c, 
		m_sptr_interface->m_q_c, 
		m_sptr_interface->m_Veh_r,

		m_sptr_interface->m_phai_c, 
		m_sptr_interface->m_theta_c, 
		m_sptr_interface->m_psi_c, 

		m_sptr_interface->m_xe_x_c, \
		m_sptr_interface->m_xe_y_c, \
		m_sptr_interface->m_xe_z_c, \

		m_sptr_interface->m_ve_x_c, \
		m_sptr_interface->m_ve_y_c, \
		m_sptr_interface->m_ve_z_c, \

		/*m_sptr_interface->m_beta_c, \
		m_sptr_interface->m_xe_x_o, 
		m_sptr_interface->m_xe_y_o, 
		m_sptr_interface->m_xe_z_o, 
		m_sptr_interface->m_ve_x_o, 
		m_sptr_interface->m_ve_y_o, 
		m_sptr_interface->m_ve_z_o, \
		m_sptr_interface->m_xb_x_o, 
		m_sptr_interface->m_xb_y_o,
		m_sptr_interface->m_xb_z_o, 
		m_sptr_interface->m_vb_x_o, 
		m_sptr_interface->m_vb_y_o, 
		m_sptr_interface->m_vb_z_o, 
		m_sptr_interface->m_beta_o, \*/

		//ineritial_frame 31
		m_sptr_interface->m_xe_x_fl, 
		m_sptr_interface->m_xe_y_fl, 
		m_sptr_interface->m_Int_Pz_fl, 
		m_sptr_interface->m_ve_x_fl, 
		m_sptr_interface->m_ve_y_fl, 
		m_sptr_interface->m_Int_Vz_fl, \

		m_sptr_interface->m_xe_x_fr, 
		m_sptr_interface->m_xe_y_fr,
		m_sptr_interface->m_Int_Pz_fr, 
		m_sptr_interface->m_ve_x_fr,
		m_sptr_interface->m_ve_y_fr, 
		m_sptr_interface->m_Int_Vz_fr, \

		m_sptr_interface->m_xe_x_rl, 
		m_sptr_interface->m_xe_y_rl, 
		m_sptr_interface->m_Int_Pz_rl, 
		m_sptr_interface->m_ve_x_rl, 
		m_sptr_interface->m_ve_y_rl, 
		m_sptr_interface->m_Int_Vz_rl, \

		m_sptr_interface->m_xe_x_rr, 
		m_sptr_interface->m_xe_y_rr, 
		m_sptr_interface->m_Int_Pz_rr, 
		m_sptr_interface->m_ve_x_rr, 
		m_sptr_interface->m_ve_y_rr, 
		m_sptr_interface->m_Int_Vz_rr, \

		//body_frame
		m_sptr_interface->m_xb_x_fl,
		m_sptr_interface->m_xb_y_fl, 
		m_sptr_interface->m_xb_z_fl, 
		m_sptr_interface->m_Veh_vx_fl, 
		m_sptr_interface->m_Veh_vy_fl, 
		m_sptr_interface->m_Veh_vz_fl, \

		m_sptr_interface->m_xb_x_fr, 
		m_sptr_interface->m_xb_y_fr, 
		m_sptr_interface->m_xb_z_fr, 
		m_sptr_interface->m_Veh_vx_fr, 
		m_sptr_interface->m_Veh_vy_fr, 
		m_sptr_interface->m_Veh_vz_fr, \

		m_sptr_interface->m_xb_x_rl, 
		m_sptr_interface->m_xb_y_rl, 
		m_sptr_interface->m_xb_z_rl, 
		m_sptr_interface->m_Veh_vx_rl, 
		m_sptr_interface->m_Veh_vy_rl, 
		m_sptr_interface->m_Veh_vz_rl, \

		m_sptr_interface->m_xb_x_rr,
		m_sptr_interface->m_xb_y_rr, 
		m_sptr_interface->m_xb_z_rr, 
		m_sptr_interface->m_Veh_vx_rr, 
		m_sptr_interface->m_Veh_vy_rr, 
		m_sptr_interface->m_Veh_vz_rr,
		
		});
}

void NMSPC::Sys_Vehicle_Body::operator() (const d_vec &x, d_vec &dxdt, const real_Y &t) {
	pull_con_states(x);
	update_pv();
	update_fm();
	update_drv();
	store_data();
	dxdt = m_drvs;

}
