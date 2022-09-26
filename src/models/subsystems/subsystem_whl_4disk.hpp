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
    static const int m_con_states_num = Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num;

    Subsys_Wheel_4Disk (\
	double unloaded_radius_f=0.31, double IYY_f=0.74, double mass_f=5.0,\
	double br_f=1e-3, double disk_abore_f=0.05, double num_pads_f=2.0, \
	double Rm_f=0.177, double mu_kinetic_f=0.2, double mu_static_f=0.3, \
	double init_omega_f=0.0, double init_Pz_f=0.0, double init_vz_f=0.0, \
	bool init_locked_flag_f=false, \
    double unloaded_radius_r=0.31, double IYY_r=0.74, double mass_r=5.0,\
	double br_r=1e-3, double disk_abore_r=0.05, double num_pads_r=2.0, \
	double Rm_r=0.177, double mu_kinetic_r=0.2, double mu_static_r=0.3, \
	double init_omega_r=0.0, double init_Pz_r=0.0, double init_vz_r=0.0, \
	bool init_locked_flag_r=false) {
        m_whl_fl = Wheel_Disk(unloaded_radius_f, IYY_f, mass_f,br_f, disk_abore_f, num_pads_f, \
        Rm_f, mu_kinetic_f, mu_static_f, init_omega_f, init_Pz_f, init_vz_f, init_locked_flag_f);
        m_whl_fr = Wheel_Disk(unloaded_radius_f, IYY_f, mass_f,br_f, disk_abore_f, num_pads_f, \
        Rm_f, mu_kinetic_f, mu_static_f, init_omega_f, init_Pz_f, init_vz_f, init_locked_flag_f);
        m_whl_rl = Wheel_Disk(unloaded_radius_r, IYY_r, mass_r,br_r, disk_abore_r, num_pads_r, \
        Rm_r, mu_kinetic_r, mu_static_r, init_omega_r, init_Pz_r, init_vz_r, init_locked_flag_r);
        m_whl_rr = Wheel_Disk(unloaded_radius_r, IYY_r, mass_r,br_r, disk_abore_r, num_pads_r, \
        Rm_r, mu_kinetic_r, mu_static_r, init_omega_r, init_Pz_r, init_vz_r, init_locked_flag_r);
    };


    void push_con_states (d_vec &con_states);
	void pull_con_states (const d_vec &con_states);

	void pull_pv (const double &Gnd_Pz_fl, const double &Gnd_Pz_fr, const double &Gnd_Pz_rl, const double &Gnd_Pz_rr);
    void push_pv (double &Tir_omega_fl, double &Sus_TirPz_fl, double &Sus_Tirvz_fl, double &Tir_Pz_fl, double &Tir_vz_fl, double &Tir_rhoz_fl, double &Tir_Re_fl, \
    double &Tir_omega_fr, double &Sus_TirPz_fr, double &Sus_Tirvz_fr, double &Tir_Pz_fr, double &Tir_vz_fr, double &Tir_rhoz_fr, double &Tir_Re_fr, \
    double &Tir_omega_rl, double &Sus_TirPz_rl, double &Sus_Tirvz_rl, double &Tir_Pz_rl, double &Tir_vz_rl, double &Tir_rhoz_rl, double &Tir_Re_rl, \
    double &Tir_omega_rr, double &Sus_TirPz_rr, double &Sus_Tirvz_rr, double &Tir_Pz_rr, double &Tir_vz_rr, double &Tir_rhoz_rr, double &Tir_Re_rr);
	void pull_fm (const double &Axl_Trq_fl, const double &Brk_Prs_fl, const double &Tir_Fx_fl, const double &Tir_My_fl, const double &Tir_Fz_fl, const double &Sus_Fz_fl,\
    const double &Axl_Trq_fr, const double &Brk_Prs_fr, const double &Tir_Fx_fr, const double &Tir_My_fr, const double &Tir_Fz_fr, const double &Sus_Fz_fr,\
    const double &Axl_Trq_rl, const double &Brk_Prs_rl, const double &Tir_Fx_rl, const double &Tir_My_rl, const double &Tir_Fz_rl, const double &Sus_Fz_rl,\
    const double &Axl_Trq_rr, const double &Brk_Prs_rr, const double &Tir_Fx_rr, const double &Tir_My_rr, const double &Tir_Fz_rr, const double &Sus_Fz_rr);
    void push_fm (double &Brk_Trq_fl, double &Brk_Trq_fr, double &Brk_Trq_rl, double &Brk_Trq_rr);

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