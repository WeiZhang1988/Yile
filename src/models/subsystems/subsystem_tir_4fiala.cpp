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
#include "subsystem_tir_4fiala.hpp"

void NMSPC::Subsys_Tire_4Fiala::push_con_states(d_vec &con_states) {
    m_tir_fl.push_con_states(m_tir_con_states_fl);
    m_tir_fr.push_con_states(m_tir_con_states_fr);
    m_tir_rl.push_con_states(m_tir_con_states_rl);
    m_tir_rr.push_con_states(m_tir_con_states_rr);
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
    m_tir_fl.pull_con_states(m_tir_con_states_fl);
    m_tir_fr.pull_con_states(m_tir_con_states_fr);
    m_tir_rl.pull_con_states(m_tir_con_states_rl);
    m_tir_rr.pull_con_states(m_tir_con_states_rr);
}

void NMSPC::Subsys_Tire_4Fiala::pull_pv(const double &Tir_omega_fl, const double &Tir_rhoz_fl, const double &Tir_Re_fl, const double &Sus_vx_fl, const double &Sus_vy_fl, const double &Sus_vz_fl,\
const double &Sus_gamma_fl, const double &Sus_str_fl, const double &Sus_r_fl, \
const double &Tir_omega_fr, const double &Tir_rhoz_fr, const double &Tir_Re_fr, const double &Sus_vx_fr, const double &Sus_vy_fr, const double &Sus_vz_fr,\
const double &Sus_gamma_fr, const double &Sus_str_fr, const double &Sus_r_fr, \
const double &Tir_omega_rl, const double &Tir_rhoz_rl, const double &Tir_Re_rl, const double &Sus_vx_rl, const double &Sus_vy_rl, const double &Sus_vz_rl,\
const double &Sus_gamma_rl, const double &Sus_str_rl, const double &Sus_r_rl, \
const double &Tir_omega_rr, const double &Tir_rhoz_rr, const double &Tir_Re_rr, const double &Sus_vx_rr, const double &Sus_vy_rr, const double &Sus_vz_rr,\
const double &Sus_gamma_rr, const double &Sus_str_rr, const double &Sus_r_rr) {
    m_tir_fl.pull_pv(Tir_omega_fl, Tir_rhoz_fl, Tir_Re_fl, Sus_vx_fl, Sus_vy_fl, Sus_vz_fl, Sus_gamma_fl, Sus_str_fl, Sus_r_fl);
    m_tir_fr.pull_pv(Tir_omega_fr, Tir_rhoz_fr, Tir_Re_fr, Sus_vx_fr, Sus_vy_fr, Sus_vz_fr, Sus_gamma_fr, Sus_str_fr, Sus_r_fr);
    m_tir_rl.pull_pv(Tir_omega_rl, Tir_rhoz_rl, Tir_Re_rl, Sus_vx_rl, Sus_vy_rl, Sus_vz_rl, Sus_gamma_rl, Sus_str_rl, Sus_r_rl);
    m_tir_rr.pull_pv(Tir_omega_rr, Tir_rhoz_rr, Tir_Re_rr, Sus_vx_rr, Sus_vy_rr, Sus_vz_rr, Sus_gamma_rr, Sus_str_rr, Sus_r_rr);
}

void NMSPC::Subsys_Tire_4Fiala::pull_fm(const double &Sus_Fz_fl, const double &Gnd_scale_fl, const double &Tir_Prs_fl, const double &Air_Tamb_fl, \
const double &Sus_Fz_fr, const double &Gnd_scale_fr, const double &Tir_Prs_fr, const double &Air_Tamb_fr, \
const double &Sus_Fz_rl, const double &Gnd_scale_rl, const double &Tir_Prs_rl, const double &Air_Tamb_rl, \
const double &Sus_Fz_rr, const double &Gnd_scale_rr, const double &Tir_Prs_rr, const double &Air_Tamb_rr) {
    m_tir_fl.pull_fm(Sus_Fz_fl, Gnd_scale_fl, Tir_Prs_fl, Air_Tamb_fl);
    m_tir_fr.pull_fm(Sus_Fz_fr, Gnd_scale_fr, Tir_Prs_fr, Air_Tamb_fr);
    m_tir_rl.pull_fm(Sus_Fz_rl, Gnd_scale_rl, Tir_Prs_rl, Air_Tamb_rl);
    m_tir_rr.pull_fm(Sus_Fz_rr, Gnd_scale_rr, Tir_Prs_rr, Air_Tamb_rr);
}

void NMSPC::Subsys_Tire_4Fiala::push_fm(double &Sus_TirFx_fl, double &Sus_TirFy_fl, double &Sus_TirFz_fl, double &Tir_Fx_fl, double &Tir_Fy_fl, double &Tir_Fz_fl, double &Tir_Mx_fl, double &Tir_My_fl, double &Tir_Mz_fl, \
double &Sus_TirFx_fr, double &Sus_TirFy_fr, double &Sus_TirFz_fr, double &Tir_Fx_fr, double &Tir_Fy_fr, double &Tir_Fz_fr, double &Tir_Mx_fr, double &Tir_My_fr, double &Tir_Mz_fr, \
double &Sus_TirFx_rl, double &Sus_TirFy_rl, double &Sus_TirFz_rl, double &Tir_Fx_rl, double &Tir_Fy_rl, double &Tir_Fz_rl, double &Tir_Mx_rl, double &Tir_My_rl, double &Tir_Mz_rl, \
double &Sus_TirFx_rr, double &Sus_TirFy_rr, double &Sus_TirFz_rr, double &Tir_Fx_rr, double &Tir_Fy_rr, double &Tir_Fz_rr, double &Tir_Mx_rr, double &Tir_My_rr, double &Tir_Mz_rr) {
    m_tir_fl.push_fm(Sus_TirFx_fl, Sus_TirFy_fl, Sus_TirFz_fl, Tir_Fx_fl, Tir_Fy_fl, Tir_Fz_fl, Tir_Mx_fl, Tir_My_fl, Tir_Mz_fl);
    m_tir_fr.push_fm(Sus_TirFx_fr, Sus_TirFy_fr, Sus_TirFz_fr, Tir_Fx_fr, Tir_Fy_fr, Tir_Fz_fr, Tir_Mx_fr, Tir_My_fr, Tir_Mz_fr);
    m_tir_rl.push_fm(Sus_TirFx_rl, Sus_TirFy_rl, Sus_TirFz_rl, Tir_Fx_rl, Tir_Fy_rl, Tir_Fz_rl, Tir_Mx_rl, Tir_My_rl, Tir_Mz_rl);
    m_tir_rr.push_fm(Sus_TirFx_rr, Sus_TirFy_rr, Sus_TirFz_rr, Tir_Fx_rr, Tir_Fy_rr, Tir_Fz_rr, Tir_Mx_rr, Tir_My_rr, Tir_Mz_rr);
}

void NMSPC::Subsys_Tire_4Fiala::push_drv(d_vec &derivatives) {
    m_tir_fl.push_drv(m_tir_drvs_fl);
    m_tir_fr.push_drv(m_tir_drvs_fr);
    m_tir_rl.push_drv(m_tir_drvs_rl);
    m_tir_rr.push_drv(m_tir_drvs_rr);
    std::copy(m_tir_drvs_fl.begin(),m_tir_drvs_fl.end(),derivatives.begin() + 0 * Tire_Fiala::m_derivatives_num);
    std::copy(m_tir_drvs_fr.begin(),m_tir_drvs_fr.end(),derivatives.begin() + 1 * Tire_Fiala::m_derivatives_num);
    std::copy(m_tir_drvs_rl.begin(),m_tir_drvs_rl.end(),derivatives.begin() + 2 * Tire_Fiala::m_derivatives_num);
    std::copy(m_tir_drvs_rr.begin(),m_tir_drvs_rr.end(),derivatives.begin() + 3 * Tire_Fiala::m_derivatives_num);
}