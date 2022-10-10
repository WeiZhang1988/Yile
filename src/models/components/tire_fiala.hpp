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
	double Lrelx=0.05, double Lrely=0.15, \
	double alpha_min=-1.50806269049027, double alpha_max=1.47424202250808, double mu_min=0.5, \
	double mu_max=0.8, double aMy=8e-4, double bMy=1e-3, \
	double cMy=1.6e-4, double alphaMy=-3e-3, double betaMy=0.97, \
	double Fz_min=100.0, double Fz_max=1e4, double cKappa=1e7, \
	double cAlpha=4.5e4, double bMz=0.0, double width=0.209045013245853, \
	double cGamma=0.0, \
	double init_kappa=0.0, double init_alpha=0.0, \
	double init_Mroll=0.0) :
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

	void pull_pv (const double &Tir_omega, const double &Tir_rhoz, const double &Tir_Re, const double &Sus_vx, const double &Sus_vy, const double &Sus_vz,\
	const double &Sus_gamma, const double &Sus_str, const double &Sus_r);
	void push_pv () {};
	void pull_fm (const double &Sus_Fz, const double &Gnd_scale, const double &Tir_Prs, const double &Air_Tamb);
	void push_fm (double &Sus_TirFx, double &Sus_TirFy, double &Sus_TirFz, \
	double &Tir_Fx, double &Tir_Fy, double &Tir_Fz, double &Tir_Mx, double &Tir_My, double &Tir_Mz);

	void push_drv (d_vec &derivatives);

private:
	//built-in parameters
	const double m_lpf_wc = 200.0 * pi;
	const double m_lpf_init = 0.0;
	const piece_wise_linear low_speed = piece_wise_linear(d_vec{0,1},d_vec{1,0});
	//configurable parameters
	double m_Lrelx, m_Lrely, m_alpha_min, m_alpha_max, \
	m_mu_min, m_mu_max, m_aMy, m_bMy, m_cMy, \
	m_alphaMy, m_betaMy, m_Fz_min, m_Fz_max, \
	m_cKappa, m_cAlpha, m_bMz, m_width, m_cGamma;
	//inputs
	double m_Tir_omega = NaN;
	double m_Tir_rhoz = NaN;
	double m_Tir_Re = NaN;
	double m_Sus_vx = NaN;
	double m_Sus_vy = NaN;
	double m_Sus_vz = NaN;
	double m_Sus_gamma = NaN;
	double m_Sus_str = NaN;
	double m_Sus_r = NaN;

	double m_Sus_Fz = NaN;
	double m_Gnd_scale = NaN;
	double m_Tir_Prs = NaN;
	double m_Air_Tamb = NaN;
	//continuous states
	double m_kappa, m_alpha_prime, m_Mroll;
	double m_Sus_lpf_str = m_lpf_init;
	double m_Sus_lpf_gamma = m_lpf_init;
	double m_Sus_lpf_Fz = m_lpf_init;
	//continuous states derivatives
	double m_drv_kappa, m_drv_alpha_prime, m_drv_Mroll;
	double m_drv_Sus_lpf_str, m_drv_Sus_lpf_gamma, m_drv_Sus_lpf_Fz;
	//discrete states
	//middle variables
	double m_Tir_vx = NaN;
	double m_Tir_vy = NaN;
	double m_Tir_vz = NaN;
	double m_Tir_r = NaN;
	double m_Tir_gamma = NaN;
	double m_Tir_str = NaN;

	double m_Tir_Fx = NaN;
	double m_Tir_Fy = NaN;
	double m_Tir_Fz = NaN;

	double m_sat_Fz = NaN;
	double m_My = NaN;
	double m_mu = NaN;
	double m_kappa_critical = NaN;
	double m_Fx1 = NaN;
	double m_Fx2 = NaN;
	double m_Fx_stick = NaN;
	double m_Fx_slide = NaN;
	double m_alpha = NaN;
	double m_tan_alpha = NaN;
	double m_alpha_critical = NaN;
	double m_H = NaN;
	double m_Mz_stick = NaN;
	double m_Fy_stick = NaN;
	double m_Mz_slide = NaN;
	double m_Fy_slide = NaN;
	double m_cos_a = NaN;
	double m_cos_b = NaN;
	double m_cos_c = NaN;
	double m_sin_a = NaN;
	double m_sin_b = NaN;
	double m_sin_c = NaN;
	double m_DCM_00	= NaN;
	double m_DCM_01	= NaN;
	double m_DCM_02	= NaN;
	double m_DCM_10	= NaN;
	double m_DCM_11	= NaN;
	double m_DCM_12	= NaN;
	double m_DCM_20	= NaN;
	double m_DCM_21	= NaN;
	double m_DCM_22	= NaN;
	//outputs
	double m_Sus_TirFx = NaN;
	double m_Sus_TirFy = NaN;
	double m_Sus_TirFz = NaN;
	//double m_Tir_Fx = NaN;
	//double m_Tir_Fy = NaN;
	//double m_Tir_Fz = NaN;
	double m_Tir_Mx = NaN;
	double m_Tir_My = NaN;
	double m_Tir_Mz = NaN;
};

}	//end of name space
#endif //TIRE_FIALA_HPP
