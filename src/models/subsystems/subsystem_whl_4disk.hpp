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
#ifndef SUBSYSTEM_WHEEL_4DISK_HPP
#define SUBSYSTEM_WHEEL_4DISK_HPP
#include "components/common.hpp"
#include "components/wheel_disk.hpp"
namespace NMSPC{
class Subsys_Wheel_4Disk {
public:
    static const int m_whls_num = 4;
    static const int m_con_states_num = Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num;

    Subsys_Wheel_4Disk (\
	real_Y unloaded_radius_f=0.309384029954441, real_Y IYY_f=0.740633832792491, real_Y mass_f=5.0,\
	real_Y br_f=1e-3, real_Y disk_abore_f=0.05, real_Y num_pads_f=2.0, \
	real_Y Rm_f=0.177, real_Y mu_kinetic_f=0.2, real_Y mu_static_f=0.3, \
	real_Y init_omega_f=0.0, real_Y init_Pz_f=0.0, real_Y init_vz_f=0.0, \
	bool init_locked_flag_f=false, \
    bool init_locked_state_f=false, \
    real_Y unloaded_radius_r=0.309384029954441, real_Y IYY_r=0.740633832792491, real_Y mass_r=5.0,\
	real_Y br_r=1e-3, real_Y disk_abore_r=0.05, real_Y num_pads_r=2.0, \
	real_Y Rm_r=0.177, real_Y mu_kinetic_r=0.2, real_Y mu_static_r=0.3, \
	real_Y init_omega_r=0.0, real_Y init_Pz_r=0.0, real_Y init_vz_r=0.0, \
	bool init_locked_flag_r=false, \
    bool init_locked_state_r=false) 
    {
        m_whl_fl = Wheel_Disk(unloaded_radius_f, IYY_f, mass_f,br_f, disk_abore_f, num_pads_f, \
        Rm_f, mu_kinetic_f, mu_static_f, init_omega_f, init_Pz_f, init_vz_f, init_locked_flag_f, init_locked_state_f);
        m_whl_fr = Wheel_Disk(unloaded_radius_f, IYY_f, mass_f,br_f, disk_abore_f, num_pads_f, \
        Rm_f, mu_kinetic_f, mu_static_f, init_omega_f, init_Pz_f, init_vz_f, init_locked_flag_f, init_locked_state_f);
        m_whl_rl = Wheel_Disk(unloaded_radius_r, IYY_r, mass_r,br_r, disk_abore_r, num_pads_r, \
        Rm_r, mu_kinetic_r, mu_static_r, init_omega_r, init_Pz_r, init_vz_r, init_locked_flag_r, init_locked_state_r);
        m_whl_rr = Wheel_Disk(unloaded_radius_r, IYY_r, mass_r,br_r, disk_abore_r, num_pads_r, \
        Rm_r, mu_kinetic_r, mu_static_r, init_omega_r, init_Pz_r, init_vz_r, init_locked_flag_r, init_locked_state_r);
    };


    void push_con_states (d_vec &con_states);
	void pull_con_states (const d_vec &con_states);

    void push_dis_states (b_vec &dis_states);
    void pull_dis_states (const b_vec &dis_states);

	void pull_pv (const real_Y &Gnd_Pz_fl, const real_Y &Gnd_Pz_fr, const real_Y &Gnd_Pz_rl, const real_Y &Gnd_Pz_rr);
    void push_pv (real_Y &Tir_omega_fl, real_Y &Sus_TirPz_fl, real_Y &Sus_Tirvz_fl, real_Y &Tir_Pz_fl, real_Y &Tir_vz_fl, real_Y &Tir_rhoz_fl, real_Y &Tir_Re_fl, \
    real_Y &Tir_omega_fr, real_Y &Sus_TirPz_fr, real_Y &Sus_Tirvz_fr, real_Y &Tir_Pz_fr, real_Y &Tir_vz_fr, real_Y &Tir_rhoz_fr, real_Y &Tir_Re_fr, \
    real_Y &Tir_omega_rl, real_Y &Sus_TirPz_rl, real_Y &Sus_Tirvz_rl, real_Y &Tir_Pz_rl, real_Y &Tir_vz_rl, real_Y &Tir_rhoz_rl, real_Y &Tir_Re_rl, \
    real_Y &Tir_omega_rr, real_Y &Sus_TirPz_rr, real_Y &Sus_Tirvz_rr, real_Y &Tir_Pz_rr, real_Y &Tir_vz_rr, real_Y &Tir_rhoz_rr, real_Y &Tir_Re_rr);
	void pull_fm (const real_Y &Axl_Trq_fl, const real_Y &Brk_Prs_fl, const real_Y &Tir_Fx_fl, const real_Y &Tir_My_fl, const real_Y &Tir_Fz_fl, const real_Y &Sus_Fz_fl,\
    const real_Y &Axl_Trq_fr, const real_Y &Brk_Prs_fr, const real_Y &Tir_Fx_fr, const real_Y &Tir_My_fr, const real_Y &Tir_Fz_fr, const real_Y &Sus_Fz_fr,\
    const real_Y &Axl_Trq_rl, const real_Y &Brk_Prs_rl, const real_Y &Tir_Fx_rl, const real_Y &Tir_My_rl, const real_Y &Tir_Fz_rl, const real_Y &Sus_Fz_rl,\
    const real_Y &Axl_Trq_rr, const real_Y &Brk_Prs_rr, const real_Y &Tir_Fx_rr, const real_Y &Tir_My_rr, const real_Y &Tir_Fz_rr, const real_Y &Sus_Fz_rr);
    void push_fm (real_Y &Brk_Trq_fl, real_Y &Brk_Trq_fr, real_Y &Brk_Trq_rl, real_Y &Brk_Trq_rr);

	void push_drv (d_vec &derivatives);

    private:
    Wheel_Disk m_whl_fl, m_whl_fr, m_whl_rl, m_whl_rr;
    d_vec m_whl_con_states_fl = d_vec(Wheel_Disk::m_con_states_num, NaN);
    d_vec m_whl_con_states_fr = d_vec(Wheel_Disk::m_con_states_num, NaN);
    d_vec m_whl_con_states_rl = d_vec(Wheel_Disk::m_con_states_num, NaN);
    d_vec m_whl_con_states_rr = d_vec(Wheel_Disk::m_con_states_num, NaN);
    d_vec m_whl_drvs_fl = d_vec(Wheel_Disk::m_derivatives_num, NaN);
    d_vec m_whl_drvs_fr = d_vec(Wheel_Disk::m_derivatives_num, NaN);
    d_vec m_whl_drvs_rl = d_vec(Wheel_Disk::m_derivatives_num, NaN);
    d_vec m_whl_drvs_rr = d_vec(Wheel_Disk::m_derivatives_num, NaN);
};

} //end of name space
#endif //SUBSYSTEM_WHEEL_4DISK_HPP