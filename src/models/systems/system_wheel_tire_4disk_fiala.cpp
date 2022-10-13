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
#include "system_wheel_tire_4disk_fiala.hpp"

void NMSPC::Sys_Wheel_Tire_4Disk_Fiala::push_con_states (d_vec &con_states) {
    m_sptr_subsys_whl_4disk->push_con_states(m_subsys_whl_4disk_con_states);
    m_sptr_subsys_tir_4fiala->push_con_states(m_subsys_tir_4fiala_con_states);
    std::copy(m_subsys_whl_4disk_con_states.begin(), m_subsys_whl_4disk_con_states.end(), con_states.begin());
    std::copy(m_subsys_tir_4fiala_con_states.begin(), m_subsys_tir_4fiala_con_states.end(), con_states.begin() + Subsys_Wheel_4Disk::m_con_states_num);
}

void NMSPC::Sys_Wheel_Tire_4Disk_Fiala::push_con_states_whl_only (d_vec &con_states) {
    m_sptr_subsys_whl_4disk->push_con_states(m_subsys_whl_4disk_con_states);
    std::copy(m_subsys_whl_4disk_con_states.begin(), m_subsys_whl_4disk_con_states.end(), con_states.begin());
}

void NMSPC::Sys_Wheel_Tire_4Disk_Fiala::pull_con_states (const d_vec &con_states) {
    std::copy(con_states.begin(), \
              con_states.begin() + Subsys_Wheel_4Disk::m_con_states_num, \
              m_subsys_whl_4disk_con_states.begin());
    std::copy(con_states.begin() + Subsys_Wheel_4Disk::m_con_states_num, \
              con_states.begin() + Subsys_Wheel_4Disk::m_con_states_num + Subsys_Tire_4Fiala::m_con_states_num, \
              m_subsys_tir_4fiala_con_states.begin());
    m_sptr_subsys_whl_4disk->pull_con_states(m_subsys_whl_4disk_con_states);
    m_sptr_subsys_tir_4fiala->pull_con_states(m_subsys_tir_4fiala_con_states);
}

void NMSPC::Sys_Wheel_Tire_4Disk_Fiala::pull_con_states_whl_only(const d_vec &con_states) {
    std::copy(con_states.begin(), con_states.begin() + Subsys_Wheel_4Disk::m_con_states_num, m_subsys_whl_4disk_con_states.begin());
    m_sptr_subsys_whl_4disk->pull_con_states(m_subsys_whl_4disk_con_states);
}

void NMSPC::Sys_Wheel_Tire_4Disk_Fiala::update_pv() {

    m_sptr_subsys_whl_4disk->pull_pv(m_sptr_interface->m_Gnd_Pz_fl, m_sptr_interface->m_Gnd_Pz_fr, m_sptr_interface->m_Gnd_Pz_rl, m_sptr_interface->m_Gnd_Pz_rr);
	m_sptr_subsys_whl_4disk->push_pv(m_sptr_interface->m_Tir_omega_fl, m_sptr_interface->m_Sus_TirPz_fl, m_sptr_interface->m_Sus_Tirvz_fl, m_sptr_interface->m_Tir_Pz_fl, m_sptr_interface->m_Tir_vz_fl, m_sptr_interface->m_Tir_rhoz_fl, m_sptr_interface->m_Tir_Re_fl, \
    m_sptr_interface->m_Tir_omega_fr, m_sptr_interface->m_Sus_TirPz_fr, m_sptr_interface->m_Sus_Tirvz_fr, m_sptr_interface->m_Tir_Pz_fr, m_sptr_interface->m_Tir_vz_fr, m_sptr_interface->m_Tir_rhoz_fr, m_sptr_interface->m_Tir_Re_fr, \
    m_sptr_interface->m_Tir_omega_rl, m_sptr_interface->m_Sus_TirPz_rl, m_sptr_interface->m_Sus_Tirvz_rl, m_sptr_interface->m_Tir_Pz_rl, m_sptr_interface->m_Tir_vz_rl, m_sptr_interface->m_Tir_rhoz_rl, m_sptr_interface->m_Tir_Re_rl, \
    m_sptr_interface->m_Tir_omega_rr, m_sptr_interface->m_Sus_TirPz_rr, m_sptr_interface->m_Sus_Tirvz_rr, m_sptr_interface->m_Tir_Pz_rr, m_sptr_interface->m_Tir_vz_rr, m_sptr_interface->m_Tir_rhoz_rr, m_sptr_interface->m_Tir_Re_rr);

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

void NMSPC::Sys_Wheel_Tire_4Disk_Fiala::update_fm() {
	m_sptr_subsys_tir_4fiala->pull_fm(m_sptr_interface->m_Sus_Fz_fl, m_sptr_interface->m_Gnd_scale_fl, m_sptr_interface->m_Tir_Prs_fl, m_sptr_interface->m_Air_Tamb_fl, \
    m_sptr_interface->m_Sus_Fz_fr, m_sptr_interface->m_Gnd_scale_fr, m_sptr_interface->m_Tir_Prs_fr, m_sptr_interface->m_Air_Tamb_fr, \
    m_sptr_interface->m_Sus_Fz_rl, m_sptr_interface->m_Gnd_scale_rl, m_sptr_interface->m_Tir_Prs_rl, m_sptr_interface->m_Air_Tamb_rl, \
    m_sptr_interface->m_Sus_Fz_rr, m_sptr_interface->m_Gnd_scale_rr, m_sptr_interface->m_Tir_Prs_rr, m_sptr_interface->m_Air_Tamb_rr);
	m_sptr_subsys_tir_4fiala->push_fm(m_sptr_interface->m_Sus_TirFx_fl, m_sptr_interface->m_Sus_TirFy_fl, m_sptr_interface->m_Sus_TirFz_fl, m_sptr_interface->m_Tir_Fx_fl, m_sptr_interface->m_Tir_Fy_fl, m_sptr_interface->m_Tir_Fz_fl, m_sptr_interface->m_Tir_Mx_fl, m_sptr_interface->m_Tir_My_fl, m_sptr_interface->m_Tir_Mz_fl, \
    m_sptr_interface->m_Sus_TirFx_fr, m_sptr_interface->m_Sus_TirFy_fr, m_sptr_interface->m_Sus_TirFz_fr, m_sptr_interface->m_Tir_Fx_fr, m_sptr_interface->m_Tir_Fy_fr, m_sptr_interface->m_Tir_Fz_fr, m_sptr_interface->m_Tir_Mx_fr, m_sptr_interface->m_Tir_My_fr, m_sptr_interface->m_Tir_Mz_fr, \
    m_sptr_interface->m_Sus_TirFx_rl, m_sptr_interface->m_Sus_TirFy_rl, m_sptr_interface->m_Sus_TirFz_rl, m_sptr_interface->m_Tir_Fx_rl, m_sptr_interface->m_Tir_Fy_rl, m_sptr_interface->m_Tir_Fz_rl, m_sptr_interface->m_Tir_Mx_rl, m_sptr_interface->m_Tir_My_rl, m_sptr_interface->m_Tir_Mz_rl, \
    m_sptr_interface->m_Sus_TirFx_rr, m_sptr_interface->m_Sus_TirFy_rr, m_sptr_interface->m_Sus_TirFz_rr, m_sptr_interface->m_Tir_Fx_rr, m_sptr_interface->m_Tir_Fy_rr, m_sptr_interface->m_Tir_Fz_rr, m_sptr_interface->m_Tir_Mx_rr, m_sptr_interface->m_Tir_My_rr, m_sptr_interface->m_Tir_Mz_rr);

	m_sptr_subsys_whl_4disk->pull_fm(m_sptr_interface->m_Axl_Trq_fl, m_sptr_interface->m_Brk_Prs_fl, m_sptr_interface->m_Tir_Fx_fl, m_sptr_interface->m_Tir_My_fl, m_sptr_interface->m_Tir_Fz_fl, m_sptr_interface->m_Sus_Fz_fl,\
    m_sptr_interface->m_Axl_Trq_fr, m_sptr_interface->m_Brk_Prs_fr, m_sptr_interface->m_Tir_Fx_fr, m_sptr_interface->m_Tir_My_fr, m_sptr_interface->m_Tir_Fz_fr, m_sptr_interface->m_Sus_Fz_fr,\
    m_sptr_interface->m_Axl_Trq_rl, m_sptr_interface->m_Brk_Prs_rl, m_sptr_interface->m_Tir_Fx_rl, m_sptr_interface->m_Tir_My_rl, m_sptr_interface->m_Tir_Fz_rl, m_sptr_interface->m_Sus_Fz_rl,\
    m_sptr_interface->m_Axl_Trq_rr, m_sptr_interface->m_Brk_Prs_rr, m_sptr_interface->m_Tir_Fx_rr, m_sptr_interface->m_Tir_My_rr, m_sptr_interface->m_Tir_Fz_rr, m_sptr_interface->m_Sus_Fz_rr);
	m_sptr_subsys_whl_4disk->push_fm(m_sptr_interface->m_Brk_Trq_fl, m_sptr_interface->m_Brk_Trq_fr, m_sptr_interface->m_Brk_Trq_rl, m_sptr_interface->m_Brk_Trq_rr);
}

void NMSPC::Sys_Wheel_Tire_4Disk_Fiala::update_drv() {
	m_sptr_subsys_whl_4disk->push_drv(m_subsys_whl_4disk_drvs);
	m_sptr_subsys_tir_4fiala->push_drv(m_subsys_tir_4fiala_drvs);
	std::copy(m_subsys_whl_4disk_drvs.begin(),m_subsys_whl_4disk_drvs.end(),m_drvs.begin());
	std::copy(m_subsys_tir_4fiala_drvs.begin(),m_subsys_tir_4fiala_drvs.end(),m_drvs.begin() + Subsys_Wheel_4Disk::m_derivatives_num);
}

void NMSPC::Sys_Wheel_Tire_4Disk_Fiala::store_data() {
	m_sptr_store->push_back(d_vec{\
            m_sptr_interface->m_Tir_omega_fl,
            m_sptr_interface->m_Sus_TirPz_fl,
            m_sptr_interface->m_Sus_Tirvz_fl,
            m_sptr_interface->m_Sus_TirFx_fl, 
            m_sptr_interface->m_Sus_TirFy_fl, 
            m_sptr_interface->m_Tir_Mx_fl,    
            m_sptr_interface->m_Tir_My_fl,    
            m_sptr_interface->m_Tir_Mz_fl,

            m_sptr_interface->m_Tir_omega_fr,
            m_sptr_interface->m_Sus_TirPz_fr,
            m_sptr_interface->m_Sus_Tirvz_fr,
            m_sptr_interface->m_Sus_TirFx_fr, 
            m_sptr_interface->m_Sus_TirFy_fr, 
            m_sptr_interface->m_Tir_Mx_fr,    
            m_sptr_interface->m_Tir_My_fr,    
            m_sptr_interface->m_Tir_Mz_fr,

            m_sptr_interface->m_Tir_omega_rl,
            m_sptr_interface->m_Sus_TirPz_rl,
            m_sptr_interface->m_Sus_Tirvz_rl,
            m_sptr_interface->m_Sus_TirFx_rl, 
            m_sptr_interface->m_Sus_TirFy_rl, 
            m_sptr_interface->m_Tir_Mx_rl,    
            m_sptr_interface->m_Tir_My_rl,    
            m_sptr_interface->m_Tir_Mz_rl, 

            m_sptr_interface->m_Tir_omega_rr,
            m_sptr_interface->m_Sus_TirPz_rr,
            m_sptr_interface->m_Sus_Tirvz_rr,
            m_sptr_interface->m_Sus_TirFx_rr,
            m_sptr_interface->m_Sus_TirFy_rr, 
            m_sptr_interface->m_Tir_Mx_rr,    
            m_sptr_interface->m_Tir_My_rr,    
            m_sptr_interface->m_Tir_Mz_rr
/*
            m_subsys_tir_4fiala_con_states[5],
            m_subsys_tir_4fiala_con_states[11],
            m_subsys_tir_4fiala_con_states[17],
            m_subsys_tir_4fiala_con_states[23],

            m_sptr_interface->m_Brk_Trq_fl,
            m_sptr_interface->m_Brk_Trq_fr,
            m_sptr_interface->m_Brk_Trq_rl,
            m_sptr_interface->m_Brk_Trq_rr
            */
		});
}

void NMSPC::Sys_Wheel_Tire_4Disk_Fiala::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
    m_t = t;
	pull_con_states(x);
	update_pv();
	update_fm();
	update_drv();
	store_data();
	dxdt = m_drvs;
}
