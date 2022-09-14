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
    static const int m_pv_inputs_num = Wheel_Disk::m_pv_inputs_num + Wheel_Disk::m_pv_inputs_num + Wheel_Disk::m_pv_inputs_num + Wheel_Disk::m_pv_inputs_num;
    static const int m_fm_inputs_num = Wheel_Disk::m_fm_inputs_num + Wheel_Disk::m_fm_inputs_num + Wheel_Disk::m_fm_inputs_num + Wheel_Disk::m_fm_inputs_num;
    static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;
    static const int m_con_states_num = Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num;
    static const int m_pv_outputs_num = Wheel_Disk::m_pv_outputs_num + Wheel_Disk::m_pv_outputs_num + Wheel_Disk::m_pv_outputs_num + Wheel_Disk::m_pv_outputs_num;
    static const int m_fm_outputs_num = Wheel_Disk::m_fm_outputs_num + Wheel_Disk::m_fm_outputs_num + Wheel_Disk::m_fm_outputs_num + Wheel_Disk::m_fm_outputs_num;
    static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;

    void add_whls(std::shared_ptr<Wheel_Disk> sptr_whl_fl, std::shared_ptr<Wheel_Disk> sptr_whl_fr,\
    std::shared_ptr<Wheel_Disk> sptr_whl_rl, std::shared_ptr<Wheel_Disk> sptr_whl_rr) 
    {m_sptr_whl_fl=sptr_whl_fl; m_sptr_whl_fr=sptr_whl_fr;m_sptr_whl_rl=sptr_whl_rl; m_sptr_whl_rr=sptr_whl_rr;}

    void push_con_states (d_vec &con_states);
	void pull_con_states (const d_vec &con_states);

	void pull_pv (const double &Gnd_Pz_fl, const double &Gnd_Pz_fr, const double &Gnd_Pz_rl, const double &Gnd_Pz_rr);
    void push_pv (double &Tir_omega_fl, double &Tir_Pz_fl, double &Tir_vz_fl, double &Tir_rhoz_fl, double &Tir_Re_fl, \
    double &Tir_omega_fr, double &Tir_Pz_fr, double &Tir_vz_fr, double &Tir_rhoz_fr, double &Tir_Re_fr, \
    double &Tir_omega_rl, double &Tir_Pz_rl, double &Tir_vz_rl, double &Tir_rhoz_rl, double &Tir_Re_rl, \
    double &Tir_omega_rr, double &Tir_Pz_rr, double &Tir_vz_rr, double &Tir_rhoz_rr, double &Tir_Re_rr);
	void pull_fm (const double &Axl_Trq_fl, const double &Brk_Prs_fl, const double &Tir_Fx_fl, const double &Tir_My_fl, const double &Tir_Fz_fl, const double &Sus_Fz_fl,\
    const double &Axl_Trq_fr, const double &Brk_Prs_fr, const double &Tir_Fx_fr, const double &Tir_My_fr, const double &Tir_Fz_fr, const double &Sus_Fz_fr,\
    const double &Axl_Trq_rl, const double &Brk_Prs_rl, const double &Tir_Fx_rl, const double &Tir_My_rl, const double &Tir_Fz_rl, const double &Sus_Fz_rl,\
    const double &Axl_Trq_rr, const double &Brk_Prs_rr, const double &Tir_Fx_rr, const double &Tir_My_rr, const double &Tir_Fz_rr, const double &Sus_Fz_rr);
    void push_fm (double &Brk_Trq_fl, double &Brk_Trq_fr, double &Brk_Trq_rl, double &Brk_Trq_rr);

	void push_drv (d_vec &derivatives);

    private:
    std::shared_ptr<Wheel_Disk> m_sptr_whl_fl, m_sptr_whl_fr, m_sptr_whl_rl, m_sptr_whl_rr;
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