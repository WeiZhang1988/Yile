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
#ifndef TIRE_FIALA_HPP
#define TIRE_FIALA_HPP
#include "common.hpp"

namespace NMSPC{
class Tire_Fiala {
public:
	static const int m_con_states_num = 6;									//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;					//amount of derivatives;
	static const int m_dis_states_num = 0;									//amount of discrete states;

	Tire_Fiala (\
	real_Y Lrelx=0.05, real_Y Lrely=0.15, \
	real_Y alpha_min=-1.50806269049027, real_Y alpha_max=1.47424202250808, real_Y mu_min=0.5, \
	real_Y mu_max=0.8, real_Y aMy=8e-4, real_Y bMy=1e-3, \
	real_Y cMy=1.6e-4, real_Y alphaMy=-3e-3, real_Y betaMy=0.97, \
	real_Y Fz_min=100.0, real_Y Fz_max=1e4, real_Y cKappa=1e7, \
	real_Y cAlpha=4.5e4, real_Y bMz=0.0, real_Y width=0.209045013245853, \
	real_Y cGamma=0.0, \
	real_Y init_kappa=0.0, real_Y init_alpha=0.0, \
	real_Y init_Mroll=0.0) :
	m_Lrelx(Lrelx), m_Lrely(Lrely), \
	m_alpha_min(alpha_min), m_alpha_max(alpha_max), \
	m_mu_min(mu_min), m_mu_max(mu_max), \
	m_aMy(aMy), m_bMy(bMy), m_cMy(cMy), \
	m_alphaMy(alphaMy), m_betaMy(betaMy), m_Fz_min(Fz_min), m_Fz_max(Fz_max), \
	m_cKappa(cKappa), m_cAlpha(cAlpha), m_bMz(bMz), \
	m_width(width), m_cGamma(cGamma), \
	m_kappa(init_kappa), m_alpha_prime(init_alpha), \
	m_Mroll(init_Mroll) {}

	Tire_Fiala &operator= (const Tire_Fiala &org);

	void push_con_states (d_vec &con_states);
	void pull_con_states (const d_vec &con_states);

	void pull_pv (const real_Y &Tir_omega, const real_Y &Tir_rhoz, const real_Y &Tir_Re, const real_Y &Sus_vx, const real_Y &Sus_vy, const real_Y &Sus_vz,\
	const real_Y &Sus_gamma, const real_Y &Sus_str, const real_Y &Sus_r);
	void push_pv () {};
	void pull_fm (const real_Y &Sus_Fz, const real_Y &Gnd_scale, const real_Y &Tir_Prs, const real_Y &Air_Tamb);
	void push_fm (real_Y &Sus_TirFx, real_Y &Sus_TirFy, real_Y &Sus_TirFz, \
	real_Y &Tir_Fx, real_Y &Tir_Fy, real_Y &Tir_Fz, real_Y &Tir_Mx, real_Y &Tir_My, real_Y &Tir_Mz);

	void push_drv (d_vec &derivatives);

private:
	//built-in parameters
	const real_Y m_lpf_wc = 200.0 * pi;
	const real_Y m_lpf_init = 0.0;
	const piece_wise_linear low_speed = piece_wise_linear(d_vec{0.0,1.0},d_vec{1.0,0.0});
	//configurable parameters
	real_Y m_Lrelx, m_Lrely, m_alpha_min, m_alpha_max, \
	m_mu_min, m_mu_max, m_aMy, m_bMy, m_cMy, \
	m_alphaMy, m_betaMy, m_Fz_min, m_Fz_max, \
	m_cKappa, m_cAlpha, m_bMz, m_width, m_cGamma;
	//inputs
	real_Y m_Tir_omega = NaN;
	real_Y m_Tir_rhoz = NaN;
	real_Y m_Tir_Re = NaN;
	real_Y m_Sus_vx = NaN;
	real_Y m_Sus_vy = NaN;
	real_Y m_Sus_vz = NaN;
	real_Y m_Sus_gamma = NaN;
	real_Y m_Sus_str = NaN;
	real_Y m_Sus_r = NaN;

	real_Y m_Sus_Fz = NaN;
	real_Y m_Gnd_scale = NaN;
	real_Y m_Tir_Prs = NaN;
	real_Y m_Air_Tamb = NaN;
	//continuous states
	real_Y m_kappa, m_alpha_prime, m_Mroll;
	real_Y m_Sus_lpf_str = m_lpf_init;
	real_Y m_Sus_lpf_gamma = m_lpf_init;
	real_Y m_Sus_lpf_Fz = m_lpf_init;
	//continuous states derivatives
	real_Y m_drv_kappa, m_drv_alpha_prime, m_drv_Mroll;
	real_Y m_drv_Sus_lpf_str, m_drv_Sus_lpf_gamma, m_drv_Sus_lpf_Fz;
	//discrete states
	//middle variables
	real_Y m_Tir_vx = NaN;
	real_Y m_Tir_vy = NaN;
	real_Y m_Tir_vz = NaN;
	real_Y m_Tir_r = NaN;
	real_Y m_Tir_gamma = NaN;
	real_Y m_Tir_str = NaN;

	real_Y m_Tir_Fx = NaN;
	real_Y m_Tir_Fy = NaN;
	real_Y m_Tir_Fz = NaN;

	real_Y m_sat_Fz = NaN;
	real_Y m_My = NaN;
	real_Y m_mu = NaN;
	real_Y m_kappa_critical = NaN;
	real_Y m_Fx1 = NaN;
	real_Y m_Fx2 = NaN;
	real_Y m_Fx_stick = NaN;
	real_Y m_Fx_slide = NaN;
	real_Y m_alpha = NaN;
	real_Y m_tan_alpha = NaN;
	real_Y m_alpha_critical = NaN;
	real_Y m_H = NaN;
	real_Y m_Mz_stick = NaN;
	real_Y m_Fy_stick = NaN;
	real_Y m_Mz_slide = NaN;
	real_Y m_Fy_slide = NaN;
	real_Y m_cos_a = NaN;
	real_Y m_cos_b = NaN;
	real_Y m_cos_c = NaN;
	real_Y m_sin_a = NaN;
	real_Y m_sin_b = NaN;
	real_Y m_sin_c = NaN;
	real_Y m_DCM_00	= NaN;
	real_Y m_DCM_01	= NaN;
	real_Y m_DCM_02	= NaN;
	real_Y m_DCM_10	= NaN;
	real_Y m_DCM_11	= NaN;
	real_Y m_DCM_12	= NaN;
	real_Y m_DCM_20	= NaN;
	real_Y m_DCM_21	= NaN;
	real_Y m_DCM_22	= NaN;
	//outputs
	real_Y m_Sus_TirFx = NaN;
	real_Y m_Sus_TirFy = NaN;
	real_Y m_Sus_TirFz = NaN;
	//real_Y m_Tir_Fx = NaN;
	//real_Y m_Tir_Fy = NaN;
	//real_Y m_Tir_Fz = NaN;
	real_Y m_Tir_Mx = NaN;
	real_Y m_Tir_My = NaN;
	real_Y m_Tir_Mz = NaN;
};

}	//end of name space
#endif //TIRE_FIALA_HPP
