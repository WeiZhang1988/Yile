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

void NMSPC::Subsys_Sus_2Ind::pull_pv(const real_Y &Veh_hgt_cg, const real_Y &Veh_r, \
    const real_Y &Strg_str_fl, const real_Y &Sus_TirPz_fl, const real_Y &Sus_Tirvz_fl,	const real_Y &Tir_Re_fl, \
	const real_Y &Int_Pz_fl, const real_Y &Int_Vz_fl, \
	const real_Y &Veh_vx_fl, const real_Y &Veh_vy_fl, const real_Y &Veh_vz_fl, \
	const real_Y &Strg_str_fr, const real_Y &Sus_TirPz_fr, const real_Y &Sus_Tirvz_fr,	const real_Y &Tir_Re_fr, \
	const real_Y &Int_Pz_fr, const real_Y &Int_Vz_fr, \
	const real_Y &Veh_vx_fr, const real_Y &Veh_vy_fr, const real_Y &Veh_vz_fr, \
    const real_Y &Strg_str_rl, const real_Y &Sus_TirPz_rl, const real_Y &Sus_Tirvz_rl,	const real_Y &Tir_Re_rl, \
	const real_Y &Int_Pz_rl, const real_Y &Int_Vz_rl, \
	const real_Y &Veh_vx_rl, const real_Y &Veh_vy_rl, const real_Y &Veh_vz_rl, \
	const real_Y &Strg_str_rr, const real_Y &Sus_TirPz_rr, const real_Y &Sus_Tirvz_rr,	const real_Y &Tir_Re_rr, \
	const real_Y &Int_Pz_rr, const real_Y &Int_Vz_rr, \
	const real_Y &Veh_vx_rr, const real_Y &Veh_vy_rr, const real_Y &Veh_vz_rr) {
    m_sus_f.pull_pv(Veh_hgt_cg, Veh_r, \
    Strg_str_fl, Sus_TirPz_fl, Sus_Tirvz_fl, Tir_Re_fl, \
    Int_Pz_fl, Int_Vz_fl, \
    Veh_vx_fl, Veh_vy_fl, Veh_vz_fl, \
    Strg_str_fr, Sus_TirPz_fr, Sus_Tirvz_fr, Tir_Re_fr, \
    Int_Pz_fr, Int_Vz_fr, \
    Veh_vx_fr, Veh_vy_fr, Veh_vz_fr);
    m_sus_r.pull_pv(Veh_hgt_cg, Veh_r, \
    Strg_str_rl, Sus_TirPz_rl, Sus_Tirvz_rl,	Tir_Re_rl, \
    Int_Pz_rl, Int_Vz_rl, \
    Veh_vx_rl, Veh_vy_rl, Veh_vz_rl, \
    Strg_str_rr, Sus_TirPz_rr, Sus_Tirvz_rr,	Tir_Re_rr, \
    Int_Pz_rr, Int_Vz_rr, \
    Veh_vx_rr, Veh_vy_rr, Veh_vz_rr);
}

void NMSPC::Subsys_Sus_2Ind::push_pv(real_Y &Sus_str_fl, real_Y &Sus_gamma_fl, real_Y &Sus_caster_fl, real_Y &Sus_r_fl, real_Y &Sus_vx_fl, real_Y &Sus_vy_fl, real_Y &Sus_vz_fl, \
real_Y &Sus_str_fr, real_Y &Sus_gamma_fr, real_Y &Sus_caster_fr, real_Y &Sus_r_fr, real_Y &Sus_vx_fr, real_Y &Sus_vy_fr, real_Y &Sus_vz_fr, \
real_Y &Sus_str_rl, real_Y &Sus_gamma_rl, real_Y &Sus_caster_rl, real_Y &Sus_r_rl, real_Y &Sus_vx_rl, real_Y &Sus_vy_rl, real_Y &Sus_vz_rl, \
real_Y &Sus_str_rr, real_Y &Sus_gamma_rr, real_Y &Sus_caster_rr, real_Y &Sus_r_rr, real_Y &Sus_vx_rr, real_Y &Sus_vy_rr, real_Y &Sus_vz_rr) {
    m_sus_f.push_pv(Sus_str_fl, Sus_gamma_fl, Sus_caster_fl, Sus_r_fl, Sus_vx_fl, Sus_vy_fl, Sus_vz_fl, \
    Sus_str_fr, Sus_gamma_fr, Sus_caster_fr, Sus_r_fr, Sus_vx_fr, Sus_vy_fr, Sus_vz_fr);
    m_sus_r.push_pv(Sus_str_rl, Sus_gamma_rl, Sus_caster_rl, Sus_r_rl, Sus_vx_rl, Sus_vy_rl, Sus_vz_rl, \
    Sus_str_rr, Sus_gamma_rr, Sus_caster_rr, Sus_r_rr, Sus_vx_rr, Sus_vy_rr, Sus_vz_rr);
}

void NMSPC::Subsys_Sus_2Ind::pull_fm_z () {
    m_sus_f.pull_fm_z();
    m_sus_r.pull_fm_z();
}

void NMSPC::Subsys_Sus_2Ind::push_fm_z (real_Y &Sus_VehFz_fl, real_Y &Sus_Fz_fl, real_Y &Sus_VehFz_fr, real_Y &Sus_Fz_fr, real_Y &Sus_VehFz_rl, real_Y &Sus_Fz_rl, real_Y &Sus_VehFz_rr, real_Y &Sus_Fz_rr) {
    m_sus_f.push_fm_z(Sus_VehFz_fl, Sus_Fz_fl, Sus_VehFz_fr, Sus_Fz_fr);
    m_sus_r.push_fm_z(Sus_VehFz_rl, Sus_Fz_rl, Sus_VehFz_rr, Sus_Fz_rr);
}

void NMSPC::Subsys_Sus_2Ind::pull_fm_o (const real_Y &Sus_TirFx_fl, const real_Y &Sus_TirFy_fl, const real_Y &Tir_Mx_fl, const real_Y &Tir_My_fl, const real_Y &Tir_Mz_fl, \
const real_Y &Sus_TirFx_fr, const real_Y &Sus_TirFy_fr, const real_Y &Tir_Mx_fr, const real_Y &Tir_My_fr, const real_Y &Tir_Mz_fr, \
const real_Y &Sus_TirFx_rl, const real_Y &Sus_TirFy_rl, const real_Y &Tir_Mx_rl, const real_Y &Tir_My_rl, const real_Y &Tir_Mz_rl, \
const real_Y &Sus_TirFx_rr, const real_Y &Sus_TirFy_rr, const real_Y &Tir_Mx_rr, const real_Y &Tir_My_rr, const real_Y &Tir_Mz_rr) {
    m_sus_f.pull_fm_o(Sus_TirFx_fl, Sus_TirFy_fl, Tir_Mx_fl, Tir_My_fl, Tir_Mz_fl, Sus_TirFx_fr, Sus_TirFy_fr, Tir_Mx_fr, Tir_My_fr, Tir_Mz_fr);
    m_sus_r.pull_fm_o(Sus_TirFx_rl, Sus_TirFy_rl, Tir_Mx_rl, Tir_My_rl, Tir_Mz_rl, Sus_TirFx_rr, Sus_TirFy_rr, Tir_Mx_rr, Tir_My_rr, Tir_Mz_rr);
}

void NMSPC::Subsys_Sus_2Ind::push_fm_o (real_Y &Sus_VehFx_fl, real_Y &Sus_VehFy_fl, real_Y &Sus_VehMx_fl, real_Y &Sus_VehMy_fl, real_Y &Sus_VehMz_fl, \
real_Y &Sus_VehFx_fr, real_Y &Sus_VehFy_fr, real_Y &Sus_VehMx_fr, real_Y &Sus_VehMy_fr, real_Y &Sus_VehMz_fr, \
real_Y &Sus_VehFx_rl, real_Y &Sus_VehFy_rl, real_Y &Sus_VehMx_rl, real_Y &Sus_VehMy_rl, real_Y &Sus_VehMz_rl, \
real_Y &Sus_VehFx_rr, real_Y &Sus_VehFy_rr, real_Y &Sus_VehMx_rr, real_Y &Sus_VehMy_rr, real_Y &Sus_VehMz_rr) {
    m_sus_f.push_fm_o(Sus_VehFx_fl, Sus_VehFy_fl, Sus_VehMx_fl, Sus_VehMy_fl, Sus_VehMz_fl, \
    Sus_VehFx_fr, Sus_VehFy_fr, Sus_VehMx_fr, Sus_VehMy_fr, Sus_VehMz_fr);
    m_sus_r.push_fm_o(Sus_VehFx_rl, Sus_VehFy_rl, Sus_VehMx_rl, Sus_VehMy_rl, Sus_VehMz_rl, \
    Sus_VehFx_rr, Sus_VehFy_rr, Sus_VehMx_rr, Sus_VehMy_rr, Sus_VehMz_rr);
}

void NMSPC::Subsys_Sus_2Ind::pull_fm(const real_Y &Sus_TirFx_fl, const real_Y &Sus_TirFy_fl, const real_Y &Tir_Mx_fl, const real_Y &Tir_My_fl, const real_Y &Tir_Mz_fl, \
const real_Y &Sus_TirFx_fr, const real_Y &Sus_TirFy_fr, const real_Y &Tir_Mx_fr, const real_Y &Tir_My_fr, const real_Y &Tir_Mz_fr, \
const real_Y &Sus_TirFx_rl, const real_Y &Sus_TirFy_rl, const real_Y &Tir_Mx_rl, const real_Y &Tir_My_rl, const real_Y &Tir_Mz_rl, \
const real_Y &Sus_TirFx_rr, const real_Y &Sus_TirFy_rr, const real_Y &Tir_Mx_rr, const real_Y &Tir_My_rr, const real_Y &Tir_Mz_rr) {
    //m_sus_f.pull_fm(Sus_TirFx_fl, Sus_TirFy_fl, Tir_Mx_fl, Tir_My_fl, Tir_Mz_fl, Sus_TirFx_fr, Sus_TirFy_fr, Tir_Mx_fr, Tir_My_fr, Tir_Mz_fr);
    //m_sus_r.pull_fm(Sus_TirFx_rl, Sus_TirFy_rl, Tir_Mx_rl, Tir_My_rl, Tir_Mz_rl, Sus_TirFx_rr, Sus_TirFy_rr, Tir_Mx_rr, Tir_My_rr, Tir_Mz_rr);
    pull_fm_z();
    pull_fm_o(Sus_TirFx_fl, Sus_TirFy_fl, Tir_Mx_fl, Tir_My_fl, Tir_Mz_fl, \
    Sus_TirFx_fr, Sus_TirFy_fr, Tir_Mx_fr, Tir_My_fr, Tir_Mz_fr, \
    Sus_TirFx_rl, Sus_TirFy_rl, Tir_Mx_rl, Tir_My_rl, Tir_Mz_rl, \
    Sus_TirFx_rr, Sus_TirFy_rr, Tir_Mx_rr, Tir_My_rr, Tir_Mz_rr);
}

void NMSPC::Subsys_Sus_2Ind::push_fm(real_Y &Sus_VehFx_fl, real_Y &Sus_VehFy_fl, real_Y &Sus_VehFz_fl, real_Y &Sus_VehMx_fl, real_Y &Sus_VehMy_fl, real_Y &Sus_VehMz_fl, real_Y &Sus_Fz_fl, \
real_Y &Sus_VehFx_fr, real_Y &Sus_VehFy_fr, real_Y &Sus_VehFz_fr, real_Y &Sus_VehMx_fr, real_Y &Sus_VehMy_fr, real_Y &Sus_VehMz_fr, real_Y &Sus_Fz_fr, \
real_Y &Sus_VehFx_rl, real_Y &Sus_VehFy_rl, real_Y &Sus_VehFz_rl, real_Y &Sus_VehMx_rl, real_Y &Sus_VehMy_rl, real_Y &Sus_VehMz_rl, real_Y &Sus_Fz_rl, \
real_Y &Sus_VehFx_rr, real_Y &Sus_VehFy_rr, real_Y &Sus_VehFz_rr, real_Y &Sus_VehMx_rr, real_Y &Sus_VehMy_rr, real_Y &Sus_VehMz_rr, real_Y &Sus_Fz_rr) {
    //m_sus_f.push_fm(Sus_VehFx_fl, Sus_VehFy_fl, Sus_VehFz_fl, Sus_VehMx_fl, Sus_VehMy_fl, Sus_VehMz_fl, Sus_Fz_fl, \
    Sus_VehFx_fr, Sus_VehFy_fr, Sus_VehFz_fr, Sus_VehMx_fr, Sus_VehMy_fr, Sus_VehMz_fr, Sus_Fz_fr);
    //m_sus_r.push_fm(Sus_VehFx_rl, Sus_VehFy_rl, Sus_VehFz_rl, Sus_VehMx_rl, Sus_VehMy_rl, Sus_VehMz_rl, Sus_Fz_rl, \
    Sus_VehFx_rr, Sus_VehFy_rr, Sus_VehFz_rr, Sus_VehMx_rr, Sus_VehMy_rr, Sus_VehMz_rr, Sus_Fz_rr);
    push_fm_z (Sus_VehFz_fl,Sus_Fz_fl, Sus_VehFz_fr, Sus_Fz_fr, Sus_VehFz_rl, Sus_Fz_rl, Sus_VehFz_rr, Sus_Fz_rr);
    push_fm_o (Sus_VehFx_fl, Sus_VehFy_fl, Sus_VehMx_fl, Sus_VehMy_fl, Sus_VehMz_fl, \
    Sus_VehFx_fr, Sus_VehFy_fr, Sus_VehMx_fr, Sus_VehMy_fr, Sus_VehMz_fr, \
    Sus_VehFx_rl, Sus_VehFy_rl, Sus_VehMx_rl, Sus_VehMy_rl, Sus_VehMz_rl, \
    Sus_VehFx_rr, Sus_VehFy_rr, Sus_VehMx_rr, Sus_VehMy_rr, Sus_VehMz_rr);
}