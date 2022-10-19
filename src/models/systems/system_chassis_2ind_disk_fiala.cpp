// =============================================================================
// PROJECT YILE - 
//
// Copyright (c) 2023
// All rights reserved.
//
// Use of this source code is governed by a GPL-3.0 license that can be found
// in the LICENSE file
//
// Author of this file	Wei ZHANG wei_zhang_1988@outlook.com,ChangMeng Hou 945881625@qq.com
//
// =============================================================================
#include "system_chassis_2ind_disk_fiala.hpp"

void NMSPC::Sys_Chassis_2Ind_Disk_Fiala::push_con_states (d_vec &con_states) {
    m_sptr_vhl_bdy->push_con_states(m_vhl_bdy_con_states);
    m_sptr_subsys_whl_4disk->push_con_states(m_subsys_whl_4disk_con_states);
	m_sptr_subsys_sus_2ind->push_con_states(m_subsys_sus_2ind_con_states);
    m_sptr_subsys_tir_4fiala->push_con_states(m_subsys_tir_4fiala_con_states);
	std::copy(m_vhl_bdy_con_states.begin(), m_vhl_bdy_con_states.end(), con_states.begin());
    std::copy(m_subsys_whl_4disk_con_states.begin(), m_subsys_whl_4disk_con_states.end(), con_states.begin() + Vehicle_Body::m_con_states_num);
    std::copy(m_subsys_sus_2ind_con_states.begin(), m_subsys_sus_2ind_con_states.end(), con_states.begin() + Vehicle_Body::m_con_states_num + Subsys_Wheel_4Disk::m_con_states_num);
    std::copy(m_subsys_tir_4fiala_con_states.begin(), m_subsys_tir_4fiala_con_states.end(), con_states.begin() + Vehicle_Body::m_con_states_num + Subsys_Wheel_4Disk::m_con_states_num + Subsys_Sus_2Ind::m_con_states_num);
}

void NMSPC::Sys_Chassis_2Ind_Disk_Fiala::push_con_states_whl_only (d_vec &con_states) {
    m_sptr_subsys_whl_4disk->push_con_states(m_subsys_whl_4disk_con_states);
    std::copy(m_subsys_whl_4disk_con_states.begin(), m_subsys_whl_4disk_con_states.end(), con_states.begin() + Vehicle_Body::m_con_states_num);
}

void NMSPC::Sys_Chassis_2Ind_Disk_Fiala::pull_con_states (const d_vec &con_states) {
	std::copy(con_states.begin(), \
              con_states.begin() + Vehicle_Body::m_con_states_num, \
              m_vhl_bdy_con_states.begin());
    std::copy(con_states.begin() + Vehicle_Body::m_con_states_num, \
              con_states.begin() + Vehicle_Body::m_con_states_num + Subsys_Wheel_4Disk::m_con_states_num, \
              m_subsys_whl_4disk_con_states.begin());
    std::copy(con_states.begin() + Vehicle_Body::m_con_states_num + Subsys_Wheel_4Disk::m_con_states_num, \
              con_states.begin() + Vehicle_Body::m_con_states_num + Subsys_Wheel_4Disk::m_con_states_num + Subsys_Sus_2Ind::m_con_states_num, \
              m_subsys_sus_2ind_con_states.begin());
    std::copy(con_states.begin() + Vehicle_Body::m_con_states_num + Subsys_Wheel_4Disk::m_con_states_num + Subsys_Sus_2Ind::m_con_states_num, \
              con_states.begin() + Vehicle_Body::m_con_states_num + Subsys_Wheel_4Disk::m_con_states_num + Subsys_Sus_2Ind::m_con_states_num + Subsys_Tire_4Fiala::m_con_states_num, \
              m_subsys_tir_4fiala_con_states.begin());
	m_sptr_vhl_bdy->pull_con_states(m_vhl_bdy_con_states);
    m_sptr_subsys_whl_4disk->pull_con_states(m_subsys_whl_4disk_con_states);
    m_sptr_subsys_sus_2ind->pull_con_states(m_subsys_sus_2ind_con_states);
    m_sptr_subsys_tir_4fiala->pull_con_states(m_subsys_tir_4fiala_con_states);
}

void NMSPC::Sys_Chassis_2Ind_Disk_Fiala::pull_con_states_whl_only (const d_vec &con_states) {
    std::copy(con_states.begin() + Vehicle_Body::m_con_states_num, con_states.begin() + Vehicle_Body::m_con_states_num + Subsys_Wheel_4Disk::m_con_states_num, m_subsys_whl_4disk_con_states.begin());
	m_sptr_subsys_whl_4disk->pull_con_states(m_subsys_whl_4disk_con_states);
}

void NMSPC::Sys_Chassis_2Ind_Disk_Fiala::update_pv() {
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

    m_sptr_subsys_whl_4disk->pull_pv(m_sptr_interface->m_Gnd_Pz_fl, m_sptr_interface->m_Gnd_Pz_fr, m_sptr_interface->m_Gnd_Pz_rl, m_sptr_interface->m_Gnd_Pz_rr);
	m_sptr_subsys_whl_4disk->push_pv(m_sptr_interface->m_Tir_omega_fl, m_sptr_interface->m_Sus_TirPz_fl, m_sptr_interface->m_Sus_Tirvz_fl, m_sptr_interface->m_Tir_Pz_fl, m_sptr_interface->m_Tir_vz_fl, m_sptr_interface->m_Tir_rhoz_fl, m_sptr_interface->m_Tir_Re_fl, \
    m_sptr_interface->m_Tir_omega_fr, m_sptr_interface->m_Sus_TirPz_fr, m_sptr_interface->m_Sus_Tirvz_fr, m_sptr_interface->m_Tir_Pz_fr, m_sptr_interface->m_Tir_vz_fr, m_sptr_interface->m_Tir_rhoz_fr, m_sptr_interface->m_Tir_Re_fr, \
    m_sptr_interface->m_Tir_omega_rl, m_sptr_interface->m_Sus_TirPz_rl, m_sptr_interface->m_Sus_Tirvz_rl, m_sptr_interface->m_Tir_Pz_rl, m_sptr_interface->m_Tir_vz_rl, m_sptr_interface->m_Tir_rhoz_rl, m_sptr_interface->m_Tir_Re_rl, \
    m_sptr_interface->m_Tir_omega_rr, m_sptr_interface->m_Sus_TirPz_rr, m_sptr_interface->m_Sus_Tirvz_rr, m_sptr_interface->m_Tir_Pz_rr, m_sptr_interface->m_Tir_vz_rr, m_sptr_interface->m_Tir_rhoz_rr, m_sptr_interface->m_Tir_Re_rr);

	m_sptr_subsys_sus_2ind->pull_pv(m_sptr_interface->m_Veh_hgt_cg, m_sptr_interface->m_Veh_r, \
    m_sptr_interface->m_Strg_str_fl, m_sptr_interface->m_Sus_TirPz_fl, m_sptr_interface->m_Sus_Tirvz_fl, m_sptr_interface->m_Tir_Re_fl, \
	m_sptr_interface->m_Int_Pz_fl, m_sptr_interface->m_Int_Vz_fl, m_sptr_interface->m_Veh_vx_fl, m_sptr_interface->m_Veh_vy_fl, m_sptr_interface->m_Veh_vz_fl, \
	m_sptr_interface->m_Strg_str_fr, m_sptr_interface->m_Sus_TirPz_fr, m_sptr_interface->m_Sus_Tirvz_fr, m_sptr_interface->m_Tir_Re_fr, \
	m_sptr_interface->m_Int_Pz_fr, m_sptr_interface->m_Int_Vz_fr, m_sptr_interface->m_Veh_vx_fr, m_sptr_interface->m_Veh_vy_fr, m_sptr_interface->m_Veh_vz_fr, \
    m_sptr_interface->m_Strg_str_rl, m_sptr_interface->m_Sus_TirPz_rl, m_sptr_interface->m_Sus_Tirvz_rl, m_sptr_interface->m_Tir_Re_rl, \
	m_sptr_interface->m_Int_Pz_rl, m_sptr_interface->m_Int_Vz_rl, m_sptr_interface->m_Veh_vx_rl, m_sptr_interface->m_Veh_vy_rl, m_sptr_interface->m_Veh_vz_rl, \
	m_sptr_interface->m_Strg_str_rr, m_sptr_interface->m_Sus_TirPz_rr, m_sptr_interface->m_Sus_Tirvz_rr, m_sptr_interface->m_Tir_Re_rr, \
	m_sptr_interface->m_Int_Pz_rr, m_sptr_interface->m_Int_Vz_rr, m_sptr_interface->m_Veh_vx_rr, m_sptr_interface->m_Veh_vy_rr, m_sptr_interface->m_Veh_vz_rr);
	m_sptr_subsys_sus_2ind->push_pv(m_sptr_interface->m_Sus_str_fl, m_sptr_interface->m_Sus_gamma_fl, m_sptr_interface->m_Sus_caster_fl, m_sptr_interface->m_Sus_r_fl, m_sptr_interface->m_Sus_vx_fl, m_sptr_interface->m_Sus_vy_fl, m_sptr_interface->m_Sus_vz_fl, \
	m_sptr_interface->m_Sus_str_fr, m_sptr_interface->m_Sus_gamma_fr, m_sptr_interface->m_Sus_caster_fr, m_sptr_interface->m_Sus_r_fr, m_sptr_interface->m_Sus_vx_fr, m_sptr_interface->m_Sus_vy_fr, m_sptr_interface->m_Sus_vz_fr, \
    m_sptr_interface->m_Sus_str_rl, m_sptr_interface->m_Sus_gamma_rl, m_sptr_interface->m_Sus_caster_rl, m_sptr_interface->m_Sus_r_rl, m_sptr_interface->m_Sus_vx_rl, m_sptr_interface->m_Sus_vy_rl, m_sptr_interface->m_Sus_vz_rl, \
	m_sptr_interface->m_Sus_str_rr, m_sptr_interface->m_Sus_gamma_rr, m_sptr_interface->m_Sus_caster_rr, m_sptr_interface->m_Sus_r_rr, m_sptr_interface->m_Sus_vx_rr, m_sptr_interface->m_Sus_vy_rr, m_sptr_interface->m_Sus_vz_rr);

    m_sptr_subsys_tir_4fiala->pull_pv(m_sptr_interface->m_Tir_omega_fl, m_sptr_interface->m_Tir_rhoz_fl, m_sptr_interface->m_Tir_Re_fl, m_sptr_interface->m_Sus_vx_fl, m_sptr_interface->m_Sus_vy_fl, m_sptr_interface->m_Sus_vz_fl,\
	m_sptr_interface->m_Sus_gamma_fl, m_sptr_interface->m_Sus_str_fl, m_sptr_interface->m_Sus_r_fl, \
    m_sptr_interface->m_Tir_omega_fr, m_sptr_interface->m_Tir_rhoz_fr, m_sptr_interface->m_Tir_Re_fr, m_sptr_interface->m_Sus_vx_fr, m_sptr_interface->m_Sus_vy_fr, m_sptr_interface->m_Sus_vz_fr,\
	m_sptr_interface->m_Sus_gamma_fr, m_sptr_interface->m_Sus_str_fr, m_sptr_interface->m_Sus_r_fr, \
    m_sptr_interface->m_Tir_omega_rl, m_sptr_interface->m_Tir_rhoz_rl, m_sptr_interface->m_Tir_Re_rl, m_sptr_interface->m_Sus_vx_rl, m_sptr_interface->m_Sus_vy_rl, m_sptr_interface->m_Sus_vz_rl,\
	m_sptr_interface->m_Sus_gamma_rl, m_sptr_interface->m_Sus_str_rl, m_sptr_interface->m_Sus_r_rl, \
    m_sptr_interface->m_Tir_omega_rr, m_sptr_interface->m_Tir_rhoz_rr, m_sptr_interface->m_Tir_Re_rr, m_sptr_interface->m_Sus_vx_rr, m_sptr_interface->m_Sus_vy_rr, m_sptr_interface->m_Sus_vz_rr,\
	m_sptr_interface->m_Sus_gamma_rr, m_sptr_interface->m_Sus_str_rr, m_sptr_interface->m_Sus_r_rr);
	m_sptr_subsys_tir_4fiala->push_pv();

}

void NMSPC::Sys_Chassis_2Ind_Disk_Fiala::update_fm() {
	m_sptr_subsys_sus_2ind->pull_fm_z();
	m_sptr_subsys_sus_2ind->push_fm_z(m_sptr_interface->m_Sus_VehFz_fl, m_sptr_interface->m_Sus_Fz_fl, \
	m_sptr_interface->m_Sus_VehFz_fr, m_sptr_interface->m_Sus_Fz_fr, \
    m_sptr_interface->m_Sus_VehFz_rl, m_sptr_interface->m_Sus_Fz_rl, \
	m_sptr_interface->m_Sus_VehFz_rr, m_sptr_interface->m_Sus_Fz_rr);

	m_sptr_subsys_tir_4fiala->pull_fm(m_sptr_interface->m_Sus_Fz_fl, m_sptr_interface->m_Gnd_scale_fl, m_sptr_interface->m_Tir_Prs_fl, m_sptr_interface->m_Air_Tamb_fl, \
    m_sptr_interface->m_Sus_Fz_fr, m_sptr_interface->m_Gnd_scale_fr, m_sptr_interface->m_Tir_Prs_fr, m_sptr_interface->m_Air_Tamb_fr, \
    m_sptr_interface->m_Sus_Fz_rl, m_sptr_interface->m_Gnd_scale_rl, m_sptr_interface->m_Tir_Prs_rl, m_sptr_interface->m_Air_Tamb_rl, \
    m_sptr_interface->m_Sus_Fz_rr, m_sptr_interface->m_Gnd_scale_rr, m_sptr_interface->m_Tir_Prs_rr, m_sptr_interface->m_Air_Tamb_rr);
	m_sptr_subsys_tir_4fiala->push_fm(m_sptr_interface->m_Sus_TirFx_fl, m_sptr_interface->m_Sus_TirFy_fl, m_sptr_interface->m_Sus_TirFz_fl, m_sptr_interface->m_Tir_Fx_fl, m_sptr_interface->m_Tir_Fy_fl, m_sptr_interface->m_Tir_Fz_fl, m_sptr_interface->m_Tir_Mx_fl, m_sptr_interface->m_Tir_My_fl, m_sptr_interface->m_Tir_Mz_fl, \
    m_sptr_interface->m_Sus_TirFx_fr, m_sptr_interface->m_Sus_TirFy_fr, m_sptr_interface->m_Sus_TirFz_fr, m_sptr_interface->m_Tir_Fx_fr, m_sptr_interface->m_Tir_Fy_fr, m_sptr_interface->m_Tir_Fz_fr, m_sptr_interface->m_Tir_Mx_fr, m_sptr_interface->m_Tir_My_fr, m_sptr_interface->m_Tir_Mz_fr, \
    m_sptr_interface->m_Sus_TirFx_rl, m_sptr_interface->m_Sus_TirFy_rl, m_sptr_interface->m_Sus_TirFz_rl, m_sptr_interface->m_Tir_Fx_rl, m_sptr_interface->m_Tir_Fy_rl, m_sptr_interface->m_Tir_Fz_rl, m_sptr_interface->m_Tir_Mx_rl, m_sptr_interface->m_Tir_My_rl, m_sptr_interface->m_Tir_Mz_rl, \
    m_sptr_interface->m_Sus_TirFx_rr, m_sptr_interface->m_Sus_TirFy_rr, m_sptr_interface->m_Sus_TirFz_rr, m_sptr_interface->m_Tir_Fx_rr, m_sptr_interface->m_Tir_Fy_rr, m_sptr_interface->m_Tir_Fz_rr, m_sptr_interface->m_Tir_Mx_rr, m_sptr_interface->m_Tir_My_rr, m_sptr_interface->m_Tir_Mz_rr);

	m_sptr_subsys_sus_2ind->pull_fm_o(m_sptr_interface->m_Sus_TirFx_fl, m_sptr_interface->m_Sus_TirFy_fl, m_sptr_interface->m_Tir_Mx_fl, m_sptr_interface->m_Tir_My_fl, m_sptr_interface->m_Tir_Mz_fl, \
	m_sptr_interface->m_Sus_TirFx_fr, m_sptr_interface->m_Sus_TirFy_fr, m_sptr_interface->m_Tir_Mx_fr, m_sptr_interface->m_Tir_My_fr, m_sptr_interface->m_Tir_Mz_fr, \
    m_sptr_interface->m_Sus_TirFx_rl, m_sptr_interface->m_Sus_TirFy_rl, m_sptr_interface->m_Tir_Mx_rl, m_sptr_interface->m_Tir_My_rl, m_sptr_interface->m_Tir_Mz_rl, \
	m_sptr_interface->m_Sus_TirFx_rr, m_sptr_interface->m_Sus_TirFy_rr, m_sptr_interface->m_Tir_Mx_rr, m_sptr_interface->m_Tir_My_rr, m_sptr_interface->m_Tir_Mz_rr);
	m_sptr_subsys_sus_2ind->push_fm_o(m_sptr_interface->m_Sus_VehFx_fl, m_sptr_interface->m_Sus_VehFy_fl, m_sptr_interface->m_Sus_VehMx_fl, m_sptr_interface->m_Sus_VehMy_fl, m_sptr_interface->m_Sus_VehMz_fl, \
	m_sptr_interface->m_Sus_VehFx_fr, m_sptr_interface->m_Sus_VehFy_fr, m_sptr_interface->m_Sus_VehMx_fr, m_sptr_interface->m_Sus_VehMy_fr, m_sptr_interface->m_Sus_VehMz_fr, \
    m_sptr_interface->m_Sus_VehFx_rl, m_sptr_interface->m_Sus_VehFy_rl, m_sptr_interface->m_Sus_VehMx_rl, m_sptr_interface->m_Sus_VehMy_rl, m_sptr_interface->m_Sus_VehMz_rl, \
	m_sptr_interface->m_Sus_VehFx_rr, m_sptr_interface->m_Sus_VehFy_rr, m_sptr_interface->m_Sus_VehMx_rr, m_sptr_interface->m_Sus_VehMy_rr, m_sptr_interface->m_Sus_VehMz_rr);

	m_sptr_subsys_whl_4disk->pull_fm(m_sptr_interface->m_Axl_Trq_fl, m_sptr_interface->m_Brk_Prs_fl, m_sptr_interface->m_Tir_Fx_fl, m_sptr_interface->m_Tir_My_fl, m_sptr_interface->m_Tir_Fz_fl, m_sptr_interface->m_Sus_Fz_fl,\
    m_sptr_interface->m_Axl_Trq_fr, m_sptr_interface->m_Brk_Prs_fr, m_sptr_interface->m_Tir_Fx_fr, m_sptr_interface->m_Tir_My_fr, m_sptr_interface->m_Tir_Fz_fr, m_sptr_interface->m_Sus_Fz_fr,\
    m_sptr_interface->m_Axl_Trq_rl, m_sptr_interface->m_Brk_Prs_rl, m_sptr_interface->m_Tir_Fx_rl, m_sptr_interface->m_Tir_My_rl, m_sptr_interface->m_Tir_Fz_rl, m_sptr_interface->m_Sus_Fz_rl,\
    m_sptr_interface->m_Axl_Trq_rr, m_sptr_interface->m_Brk_Prs_rr, m_sptr_interface->m_Tir_Fx_rr, m_sptr_interface->m_Tir_My_rr, m_sptr_interface->m_Tir_Fz_rr, m_sptr_interface->m_Sus_Fz_rr);
	m_sptr_subsys_whl_4disk->push_fm(m_sptr_interface->m_Brk_Trq_fl, m_sptr_interface->m_Brk_Trq_fr, m_sptr_interface->m_Brk_Trq_rl, m_sptr_interface->m_Brk_Trq_rr);

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

void NMSPC::Sys_Chassis_2Ind_Disk_Fiala::update_drv() {
	m_sptr_vhl_bdy->push_drv(m_vhl_bdy_drvs);
	m_sptr_subsys_whl_4disk->push_drv(m_subsys_whl_4disk_drvs);
	m_sptr_subsys_sus_2ind->push_drv(m_subsys_sus_2ind_drvs);
	m_sptr_subsys_tir_4fiala->push_drv(m_subsys_tir_4fiala_drvs);
	std::copy(m_vhl_bdy_drvs.begin(),m_vhl_bdy_drvs.end(),m_drvs.begin());
	std::copy(m_subsys_whl_4disk_drvs.begin(),m_subsys_whl_4disk_drvs.end(),\
				m_drvs.begin() + Vehicle_Body::m_derivatives_num);
	std::copy(m_subsys_sus_2ind_drvs.begin(),m_subsys_sus_2ind_drvs.end(),
				m_drvs.begin() + Vehicle_Body::m_derivatives_num + Subsys_Wheel_4Disk::m_derivatives_num);
	std::copy(m_subsys_tir_4fiala_drvs.begin(),m_subsys_tir_4fiala_drvs.end(),
	m_drvs.begin() + Vehicle_Body::m_derivatives_num + Subsys_Wheel_4Disk::m_derivatives_num + Subsys_Sus_2Ind::m_derivatives_num);
}

void NMSPC::Sys_Chassis_2Ind_Disk_Fiala::store_data() {
	m_sptr_store->push_back(d_vec{\

		//m_sptr_interface->m_Veh_hgt_cg,\

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
		// outputs from 184 to 197 are useless 
		m_sptr_interface->m_beta_c, \
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
		m_sptr_interface->m_beta_o, \

		//ineritial_frame
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

void NMSPC::Sys_Chassis_2Ind_Disk_Fiala::operator() (const d_vec &x, d_vec &dxdt, const real_Y &t) {
	pull_con_states(x);
	update_pv();
	update_fm();
	update_drv();
	store_data();
	dxdt = m_drvs;
}
