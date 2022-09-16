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
#include "subsystem_sus_2ind.hpp"

void NMSPC::Subsys_Sus_2Ind::pull_pv(const double &Veh_hgt_cg, const double &Veh_r, \
const double &Strg_str_fl, const double &Tir_Pz_fl, const double &Tir_vz_fl,	const double &Tir_Re_fl, \
const double &Veh_Pz_fl, const double &Veh_vx_fl, const double &Veh_vy_fl, const double &Veh_vz_fl, \
const double &Strg_str_fr, const double &Tir_Pz_fr, const double &Tir_vz_fr,	const double &Tir_Re_fr, \
const double &Veh_Pz_fr, const double &Veh_vx_fr, const double &Veh_vy_fr, const double &Veh_vz_fr, \
const double &Strg_str_rl, const double &Tir_Pz_rl, const double &Tir_vz_rl,	const double &Tir_Re_rl, \
const double &Veh_Pz_rl, const double &Veh_vx_rl, const double &Veh_vy_rl, const double &Veh_vz_rl, \
const double &Strg_str_rr, const double &Tir_Pz_rr, const double &Tir_vz_rr,	const double &Tir_Re_rr, \
const double &Veh_Pz_rr, const double &Veh_vx_rr, const double &Veh_vy_rr, const double &Veh_vz_rr) {
    m_sptr_sus_f->pull_pv(Veh_hgt_cg, Veh_r, \
    Strg_str_fl, Tir_Pz_fl, Tir_vz_fl, Tir_Re_fl, \
    Veh_Pz_fl, Veh_vx_fl, Veh_vy_fl, Veh_vz_fl, \
    Strg_str_fr, Tir_Pz_fr, Tir_vz_fr, Tir_Re_fr, \
    Veh_Pz_fr, Veh_vx_fr, Veh_vy_fr, Veh_vz_fr);
    m_sptr_sus_r->pull_pv(Veh_hgt_cg, Veh_r, \
    Strg_str_rl, Tir_Pz_rl, Tir_vz_rl,	Tir_Re_rl, \
    Veh_Pz_rl, Veh_vx_rl, Veh_vy_rl, Veh_vz_rl, \
    Strg_str_rr, Tir_Pz_rr, Tir_vz_rr,	Tir_Re_rr, \
    Veh_Pz_rr, Veh_vx_rr, Veh_vy_rr, Veh_vz_rr);
}

void NMSPC::Subsys_Sus_2Ind::push_pv(double &Sus_str_fl, double &Sus_gamma_fl, double &Sus_caster_fl, double &Sus_r_fl, double &Sus_vx_fl, double &Sus_vy_fl, double &Sus_vz_fl, \
double &Sus_str_fr, double &Sus_gamma_fr, double &Sus_caster_fr, double &Sus_r_fr, double &Sus_vx_fr, double &Sus_vy_fr, double &Sus_vz_fr, \
double &Sus_str_rl, double &Sus_gamma_rl, double &Sus_caster_rl, double &Sus_r_rl, double &Sus_vx_rl, double &Sus_vy_rl, double &Sus_vz_rl, \
double &Sus_str_rr, double &Sus_gamma_rr, double &Sus_caster_rr, double &Sus_r_rr, double &Sus_vx_rr, double &Sus_vy_rr, double &Sus_vz_rr) {
    m_sptr_sus_f->push_pv(Sus_str_fl, Sus_gamma_fl, Sus_caster_fl, Sus_r_fl, Sus_vx_fl, Sus_vy_fl, Sus_vz_fl, \
    Sus_str_fr, Sus_gamma_fr, Sus_caster_fr, Sus_r_fr, Sus_vx_fr, Sus_vy_fr, Sus_vz_fr);
    m_sptr_sus_r->push_pv(Sus_str_rl, Sus_gamma_rl, Sus_caster_rl, Sus_r_rl, Sus_vx_rl, Sus_vy_rl, Sus_vz_rl, \
    Sus_str_rr, Sus_gamma_rr, Sus_caster_rr, Sus_r_rr, Sus_vx_rr, Sus_vy_rr, Sus_vz_rr);
}

void NMSPC::Subsys_Sus_2Ind::pull_fm(const double &Sus_TirFx_fl, const double &Sus_TirFy_fl, const double &Tir_Mx_fl, const double &Tir_My_fl, const double &Tir_Mz_fl, \
const double &Sus_TirFx_fr, const double &Sus_TirFy_fr, const double &Tir_Mx_fr, const double &Tir_My_fr, const double &Tir_Mz_fr, \
const double &Sus_TirFx_rl, const double &Sus_TirFy_rl, const double &Tir_Mx_rl, const double &Tir_My_rl, const double &Tir_Mz_rl, \
const double &Sus_TirFx_rr, const double &Sus_TirFy_rr, const double &Tir_Mx_rr, const double &Tir_My_rr, const double &Tir_Mz_rr) {
    m_sptr_sus_f->pull_fm(Sus_TirFx_fl, Sus_TirFy_fl, Tir_Mx_fl, Tir_My_fl, Tir_Mz_fl, Sus_TirFx_fr, Sus_TirFy_fr, Tir_Mx_fr, Tir_My_fr, Tir_Mz_fr);
    m_sptr_sus_r->pull_fm(Sus_TirFx_rl, Sus_TirFy_rl, Tir_Mx_rl, Tir_My_rl, Tir_Mz_rl, Sus_TirFx_rr, Sus_TirFy_rr, Tir_Mx_rr, Tir_My_rr, Tir_Mz_rr);
}

void NMSPC::Subsys_Sus_2Ind::push_fm(double &Sus_VehFx_fl, double &Sus_VehFy_fl, double &Sus_VehFz_fl, double &Sus_VehMx_fl, double &Sus_VehMy_fl, double &Sus_VehMz_fl, double &Sus_Fz_fl, \
double &Sus_VehFx_fr, double &Sus_VehFy_fr, double &Sus_VehFz_fr, double &Sus_VehMx_fr, double &Sus_VehMy_fr, double &Sus_VehMz_fr, double &Sus_Fz_fr, \
double &Sus_VehFx_rl, double &Sus_VehFy_rl, double &Sus_VehFz_rl, double &Sus_VehMx_rl, double &Sus_VehMy_rl, double &Sus_VehMz_rl, double &Sus_Fz_rl, \
double &Sus_VehFx_rr, double &Sus_VehFy_rr, double &Sus_VehFz_rr, double &Sus_VehMx_rr, double &Sus_VehMy_rr, double &Sus_VehMz_rr, double &Sus_Fz_rr) {
    m_sptr_sus_f->push_fm(Sus_VehFx_fl, Sus_VehFy_fl, Sus_VehFz_fl, Sus_VehMx_fl, Sus_VehMy_fl, Sus_VehMz_fl, Sus_Fz_fl, \
    Sus_VehFx_fr, Sus_VehFy_fr, Sus_VehFz_fr, Sus_VehMx_fr, Sus_VehMy_fr, Sus_VehMz_fr, Sus_Fz_fr);
    m_sptr_sus_r->push_fm(Sus_VehFx_rl, Sus_VehFy_rl, Sus_VehFz_rl, Sus_VehMx_rl, Sus_VehMy_rl, Sus_VehMz_rl, Sus_Fz_rl, \
    Sus_VehFx_rr, Sus_VehFy_rr, Sus_VehFz_rr, Sus_VehMx_rr, Sus_VehMy_rr, Sus_VehMz_rr, Sus_Fz_rr);
}