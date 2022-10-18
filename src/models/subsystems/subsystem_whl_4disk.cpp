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
#include "subsystem_whl_4disk.hpp"

void NMSPC::Subsys_Wheel_4Disk::push_con_states(d_vec &con_states) {
    m_whl_fl.push_con_states(m_whl_con_states_fl);
    m_whl_fr.push_con_states(m_whl_con_states_fr);
    m_whl_rl.push_con_states(m_whl_con_states_rl);
    m_whl_rr.push_con_states(m_whl_con_states_rr);
    std::copy(m_whl_con_states_fl.begin(),m_whl_con_states_fl.end(),con_states.begin() + 0 * Wheel_Disk::m_con_states_num);
    std::copy(m_whl_con_states_fr.begin(),m_whl_con_states_fr.end(),con_states.begin() + 1 * Wheel_Disk::m_con_states_num);
    std::copy(m_whl_con_states_rl.begin(),m_whl_con_states_rl.end(),con_states.begin() + 2 * Wheel_Disk::m_con_states_num);
    std::copy(m_whl_con_states_rr.begin(),m_whl_con_states_rr.end(),con_states.begin() + 3 * Wheel_Disk::m_con_states_num);
}

void NMSPC::Subsys_Wheel_4Disk::pull_con_states(const d_vec &con_states) {
    std::copy(con_states.begin() + 0 * Wheel_Disk::m_con_states_num,con_states.begin() + 1 * Wheel_Disk::m_con_states_num,m_whl_con_states_fl.begin());
    std::copy(con_states.begin() + 1 * Wheel_Disk::m_con_states_num,con_states.begin() + 2 * Wheel_Disk::m_con_states_num,m_whl_con_states_fr.begin());
    std::copy(con_states.begin() + 2 * Wheel_Disk::m_con_states_num,con_states.begin() + 3 * Wheel_Disk::m_con_states_num,m_whl_con_states_rl.begin());
    std::copy(con_states.begin() + 3 * Wheel_Disk::m_con_states_num,con_states.end(),m_whl_con_states_rr.begin());
    m_whl_fl.pull_con_states(m_whl_con_states_fl);
    m_whl_fr.pull_con_states(m_whl_con_states_fr);
    m_whl_rl.pull_con_states(m_whl_con_states_rl);
    m_whl_rr.pull_con_states(m_whl_con_states_rr);
}

void NMSPC::Subsys_Wheel_4Disk::pull_pv(const real_Y &Gnd_Pz_fl, const real_Y &Gnd_Pz_fr, const real_Y &Gnd_Pz_rl, const real_Y &Gnd_Pz_rr) {
    m_whl_fl.pull_pv(Gnd_Pz_fl);
    m_whl_fr.pull_pv(Gnd_Pz_fr);
    m_whl_rl.pull_pv(Gnd_Pz_rl);
    m_whl_rr.pull_pv(Gnd_Pz_rr);
}
void NMSPC::Subsys_Wheel_4Disk::push_pv(real_Y &Tir_omega_fl, real_Y &Sus_TirPz_fl, real_Y &Sus_Tirvz_fl, real_Y &Tir_Pz_fl, real_Y &Tir_vz_fl, real_Y &Tir_rhoz_fl, real_Y &Tir_Re_fl, \
real_Y &Tir_omega_fr, real_Y &Sus_TirPz_fr, real_Y &Sus_Tirvz_fr, real_Y &Tir_Pz_fr, real_Y &Tir_vz_fr, real_Y &Tir_rhoz_fr, real_Y &Tir_Re_fr, \
real_Y &Tir_omega_rl, real_Y &Sus_TirPz_rl, real_Y &Sus_Tirvz_rl, real_Y &Tir_Pz_rl, real_Y &Tir_vz_rl, real_Y &Tir_rhoz_rl, real_Y &Tir_Re_rl, \
real_Y &Tir_omega_rr, real_Y &Sus_TirPz_rr, real_Y &Sus_Tirvz_rr, real_Y &Tir_Pz_rr, real_Y &Tir_vz_rr, real_Y &Tir_rhoz_rr, real_Y &Tir_Re_rr) {
    m_whl_fl.push_pv(Tir_omega_fl, Sus_TirPz_fl, Sus_Tirvz_fl, Tir_Pz_fl, Tir_vz_fl, Tir_rhoz_fl, Tir_Re_fl);
    m_whl_fr.push_pv(Tir_omega_fr, Sus_TirPz_fr, Sus_Tirvz_fr, Tir_Pz_fr, Tir_vz_fr, Tir_rhoz_fr, Tir_Re_fr);
    m_whl_rl.push_pv(Tir_omega_rl, Sus_TirPz_rl, Sus_Tirvz_rl, Tir_Pz_rl, Tir_vz_rl, Tir_rhoz_rl, Tir_Re_rl);
    m_whl_rr.push_pv(Tir_omega_rr, Sus_TirPz_rr, Sus_Tirvz_rr, Tir_Pz_rr, Tir_vz_rr, Tir_rhoz_rr, Tir_Re_rr);
}

void NMSPC::Subsys_Wheel_4Disk::pull_fm(const real_Y &Axl_Trq_fl, const real_Y &Brk_Prs_fl, const real_Y &Tir_Fx_fl, const real_Y &Tir_My_fl, const real_Y &Tir_Fz_fl, const real_Y &Sus_Fz_fl,\
const real_Y &Axl_Trq_fr, const real_Y &Brk_Prs_fr, const real_Y &Tir_Fx_fr, const real_Y &Tir_My_fr, const real_Y &Tir_Fz_fr, const real_Y &Sus_Fz_fr,\
const real_Y &Axl_Trq_rl, const real_Y &Brk_Prs_rl, const real_Y &Tir_Fx_rl, const real_Y &Tir_My_rl, const real_Y &Tir_Fz_rl, const real_Y &Sus_Fz_rl,\
const real_Y &Axl_Trq_rr, const real_Y &Brk_Prs_rr, const real_Y &Tir_Fx_rr, const real_Y &Tir_My_rr, const real_Y &Tir_Fz_rr, const real_Y &Sus_Fz_rr) {
    m_whl_fl.pull_fm(Axl_Trq_fl, Brk_Prs_fl, Tir_Fx_fl, Tir_My_fl, Tir_Fz_fl, Sus_Fz_fl);
    m_whl_fr.pull_fm(Axl_Trq_fr, Brk_Prs_fr, Tir_Fx_fr, Tir_My_fr, Tir_Fz_fr, Sus_Fz_fr);
    m_whl_rl.pull_fm(Axl_Trq_rl, Brk_Prs_rl, Tir_Fx_rl, Tir_My_rl, Tir_Fz_rl, Sus_Fz_rl);
    m_whl_rr.pull_fm(Axl_Trq_rr, Brk_Prs_rr, Tir_Fx_rr, Tir_My_rr, Tir_Fz_rr, Sus_Fz_rr);
}

void NMSPC::Subsys_Wheel_4Disk::push_fm(real_Y &Brk_Trq_fl, real_Y &Brk_Trq_fr, real_Y &Brk_Trq_rl, real_Y &Brk_Trq_rr) {
    m_whl_fl.push_fm(Brk_Trq_fl);
    m_whl_fr.push_fm(Brk_Trq_fr);
    m_whl_rl.push_fm(Brk_Trq_rl);
    m_whl_rr.push_fm(Brk_Trq_rr);
}

void NMSPC::Subsys_Wheel_4Disk::push_drv(d_vec &derivatives) {
    m_whl_fl.push_drv(m_whl_drvs_fl);
    m_whl_fr.push_drv(m_whl_drvs_fr);
    m_whl_rl.push_drv(m_whl_drvs_rl);
    m_whl_rr.push_drv(m_whl_drvs_rr);
    std::copy(m_whl_drvs_fl.begin(),m_whl_drvs_fl.end(),derivatives.begin() + 0 * Wheel_Disk::m_derivatives_num);
    std::copy(m_whl_drvs_fr.begin(),m_whl_drvs_fr.end(),derivatives.begin() + 1 * Wheel_Disk::m_derivatives_num);
    std::copy(m_whl_drvs_rl.begin(),m_whl_drvs_rl.end(),derivatives.begin() + 2 * Wheel_Disk::m_derivatives_num);
    std::copy(m_whl_drvs_rr.begin(),m_whl_drvs_rr.end(),derivatives.begin() + 3 * Wheel_Disk::m_derivatives_num);
}