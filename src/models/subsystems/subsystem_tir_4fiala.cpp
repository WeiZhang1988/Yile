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
#include "subsystem_tir_4fiala.hpp"

void NMSPC::Subsys_Tire_4Fiala::push_con_states(d_vec &con_states) {
    //order: front left, rear right, rear left, front right
    m_tir_fl.push_con_states(m_tir_con_states_fl);
    m_tir_rr.push_con_states(m_tir_con_states_rr);
    m_tir_rl.push_con_states(m_tir_con_states_rl);
    m_tir_fr.push_con_states(m_tir_con_states_fr);

    std::copy(m_tir_con_states_fl.begin(),m_tir_con_states_fl.end(),con_states.begin() + 0 * Tire_Fiala::m_con_states_num);
    std::copy(m_tir_con_states_fr.begin(),m_tir_con_states_fr.end(),con_states.begin() + 1 * Tire_Fiala::m_con_states_num);
    std::copy(m_tir_con_states_rl.begin(),m_tir_con_states_rl.end(),con_states.begin() + 2 * Tire_Fiala::m_con_states_num);
    std::copy(m_tir_con_states_rr.begin(),m_tir_con_states_rr.end(),con_states.begin() + 3 * Tire_Fiala::m_con_states_num);
}

void NMSPC::Subsys_Tire_4Fiala::pull_con_states(const d_vec &con_states) {
    std::copy(con_states.begin() + 0 * Tire_Fiala::m_con_states_num,con_states.begin() + 1 * Tire_Fiala::m_con_states_num,m_tir_con_states_fl.begin());
    std::copy(con_states.begin() + 1 * Tire_Fiala::m_con_states_num,con_states.begin() + 2 * Tire_Fiala::m_con_states_num,m_tir_con_states_fr.begin());
    std::copy(con_states.begin() + 2 * Tire_Fiala::m_con_states_num,con_states.begin() + 3 * Tire_Fiala::m_con_states_num,m_tir_con_states_rl.begin());
    std::copy(con_states.begin() + 3 * Tire_Fiala::m_con_states_num,con_states.end(),m_tir_con_states_rr.begin());
    //order: front left, rear right, rear left, front right
    m_tir_fl.pull_con_states(m_tir_con_states_fl);
    m_tir_rr.pull_con_states(m_tir_con_states_rr);
    m_tir_rl.pull_con_states(m_tir_con_states_rl);
    m_tir_fr.pull_con_states(m_tir_con_states_fr);
}

void NMSPC::Subsys_Tire_4Fiala::pull_pv(const real_Y &Tir_omega_fl, const real_Y &Tir_rhoz_fl, const real_Y &Tir_Re_fl, const real_Y &Sus_vx_fl, const real_Y &Sus_vy_fl, const real_Y &Sus_vz_fl,\
const real_Y &Sus_gamma_fl, const real_Y &Sus_str_fl, const real_Y &Sus_r_fl, \
const real_Y &Tir_omega_fr, const real_Y &Tir_rhoz_fr, const real_Y &Tir_Re_fr, const real_Y &Sus_vx_fr, const real_Y &Sus_vy_fr, const real_Y &Sus_vz_fr,\
const real_Y &Sus_gamma_fr, const real_Y &Sus_str_fr, const real_Y &Sus_r_fr, \
const real_Y &Tir_omega_rl, const real_Y &Tir_rhoz_rl, const real_Y &Tir_Re_rl, const real_Y &Sus_vx_rl, const real_Y &Sus_vy_rl, const real_Y &Sus_vz_rl,\
const real_Y &Sus_gamma_rl, const real_Y &Sus_str_rl, const real_Y &Sus_r_rl, \
const real_Y &Tir_omega_rr, const real_Y &Tir_rhoz_rr, const real_Y &Tir_Re_rr, const real_Y &Sus_vx_rr, const real_Y &Sus_vy_rr, const real_Y &Sus_vz_rr,\
const real_Y &Sus_gamma_rr, const real_Y &Sus_str_rr, const real_Y &Sus_r_rr) {
    //order: front left, rear right, rear left, front right
    m_tir_fl.pull_pv(Tir_omega_fl, Tir_rhoz_fl, Tir_Re_fl, Sus_vx_fl, Sus_vy_fl, Sus_vz_fl, Sus_gamma_fl, Sus_str_fl, Sus_r_fl);
    m_tir_rr.pull_pv(Tir_omega_rr, Tir_rhoz_rr, Tir_Re_rr, Sus_vx_rr, Sus_vy_rr, Sus_vz_rr, Sus_gamma_rr, Sus_str_rr, Sus_r_rr);
    m_tir_rl.pull_pv(Tir_omega_rl, Tir_rhoz_rl, Tir_Re_rl, Sus_vx_rl, Sus_vy_rl, Sus_vz_rl, Sus_gamma_rl, Sus_str_rl, Sus_r_rl);
    m_tir_fr.pull_pv(Tir_omega_fr, Tir_rhoz_fr, Tir_Re_fr, Sus_vx_fr, Sus_vy_fr, Sus_vz_fr, Sus_gamma_fr, Sus_str_fr, Sus_r_fr);
}

void NMSPC::Subsys_Tire_4Fiala::pull_fm(const real_Y &Sus_Fz_fl, const real_Y &Gnd_scale_fl, const real_Y &Tir_Prs_fl, const real_Y &Air_Tamb_fl, \
const real_Y &Sus_Fz_fr, const real_Y &Gnd_scale_fr, const real_Y &Tir_Prs_fr, const real_Y &Air_Tamb_fr, \
const real_Y &Sus_Fz_rl, const real_Y &Gnd_scale_rl, const real_Y &Tir_Prs_rl, const real_Y &Air_Tamb_rl, \
const real_Y &Sus_Fz_rr, const real_Y &Gnd_scale_rr, const real_Y &Tir_Prs_rr, const real_Y &Air_Tamb_rr) {
    //order: front left, rear right, rear left, front right
    m_tir_fl.pull_fm(Sus_Fz_fl, Gnd_scale_fl, Tir_Prs_fl, Air_Tamb_fl);
    m_tir_rr.pull_fm(Sus_Fz_rr, Gnd_scale_rr, Tir_Prs_rr, Air_Tamb_rr);
    m_tir_rl.pull_fm(Sus_Fz_rl, Gnd_scale_rl, Tir_Prs_rl, Air_Tamb_rl);
    m_tir_fr.pull_fm(Sus_Fz_fr, Gnd_scale_fr, Tir_Prs_fr, Air_Tamb_fr);
}

void NMSPC::Subsys_Tire_4Fiala::push_fm(real_Y &Sus_TirFx_fl, real_Y &Sus_TirFy_fl, real_Y &Sus_TirFz_fl, real_Y &Tir_Fx_fl, real_Y &Tir_Fy_fl, real_Y &Tir_Fz_fl, real_Y &Tir_Mx_fl, real_Y &Tir_My_fl, real_Y &Tir_Mz_fl, \
real_Y &Sus_TirFx_fr, real_Y &Sus_TirFy_fr, real_Y &Sus_TirFz_fr, real_Y &Tir_Fx_fr, real_Y &Tir_Fy_fr, real_Y &Tir_Fz_fr, real_Y &Tir_Mx_fr, real_Y &Tir_My_fr, real_Y &Tir_Mz_fr, \
real_Y &Sus_TirFx_rl, real_Y &Sus_TirFy_rl, real_Y &Sus_TirFz_rl, real_Y &Tir_Fx_rl, real_Y &Tir_Fy_rl, real_Y &Tir_Fz_rl, real_Y &Tir_Mx_rl, real_Y &Tir_My_rl, real_Y &Tir_Mz_rl, \
real_Y &Sus_TirFx_rr, real_Y &Sus_TirFy_rr, real_Y &Sus_TirFz_rr, real_Y &Tir_Fx_rr, real_Y &Tir_Fy_rr, real_Y &Tir_Fz_rr, real_Y &Tir_Mx_rr, real_Y &Tir_My_rr, real_Y &Tir_Mz_rr) {
    //order: front left, rear right, rear left, front right
    m_tir_fl.push_fm(Sus_TirFx_fl, Sus_TirFy_fl, Sus_TirFz_fl, Tir_Fx_fl, Tir_Fy_fl, Tir_Fz_fl, Tir_Mx_fl, Tir_My_fl, Tir_Mz_fl);
    m_tir_rr.push_fm(Sus_TirFx_rr, Sus_TirFy_rr, Sus_TirFz_rr, Tir_Fx_rr, Tir_Fy_rr, Tir_Fz_rr, Tir_Mx_rr, Tir_My_rr, Tir_Mz_rr);
    m_tir_rl.push_fm(Sus_TirFx_rl, Sus_TirFy_rl, Sus_TirFz_rl, Tir_Fx_rl, Tir_Fy_rl, Tir_Fz_rl, Tir_Mx_rl, Tir_My_rl, Tir_Mz_rl);
    m_tir_fr.push_fm(Sus_TirFx_fr, Sus_TirFy_fr, Sus_TirFz_fr, Tir_Fx_fr, Tir_Fy_fr, Tir_Fz_fr, Tir_Mx_fr, Tir_My_fr, Tir_Mz_fr);
}

void NMSPC::Subsys_Tire_4Fiala::push_drv(d_vec &derivatives) {
    //order: front left, rear right, rear left, front right
    m_tir_fl.push_drv(m_tir_drvs_fl);
    m_tir_rr.push_drv(m_tir_drvs_rr);
    m_tir_rl.push_drv(m_tir_drvs_rl);
    m_tir_fr.push_drv(m_tir_drvs_fr);

    std::copy(m_tir_drvs_fl.begin(),m_tir_drvs_fl.end(),derivatives.begin() + 0 * Tire_Fiala::m_derivatives_num);
    std::copy(m_tir_drvs_fr.begin(),m_tir_drvs_fr.end(),derivatives.begin() + 1 * Tire_Fiala::m_derivatives_num);
    std::copy(m_tir_drvs_rl.begin(),m_tir_drvs_rl.end(),derivatives.begin() + 2 * Tire_Fiala::m_derivatives_num);
    std::copy(m_tir_drvs_rr.begin(),m_tir_drvs_rr.end(),derivatives.begin() + 3 * Tire_Fiala::m_derivatives_num);
}