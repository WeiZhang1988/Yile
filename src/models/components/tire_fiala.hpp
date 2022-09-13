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
	static const int m_params_num = 18;										//amount of parameters
	static const int m_pv_inputs_num = 9;									//amount of pv inputs
	static const int m_fm_inputs_num = 4;									//amount of fm inputs
	static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;		//amount of inputs
	static const int m_con_states_num = 3;									//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;					//amount of derivatives;
	static const int m_dis_states_num = 0;									//amount of discrete states;
	static const int m_pv_outputs_num = 0;									//amount of pv outputs
	static const int m_fm_outputs_num = 6;									//amount of fm outputs
	static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;	//amount of outputs

	Tire_Fiala (\
	double Lrelx=0.05, double Lrely=0.15, \
	double alpha_min=-1.5708, double alpha_max=1.5708, double mu_min=0.8, \
	double mu_max=1.0, double aMy=8e-4, double bMy=1e-3, \
	double cMy=1.6e-4, double alphaMy=-3e-3, double betaMy=0.97, \
	double Fz_min=100.0, double Fz_max=1e4, double cKappa=1e7, \
	double cAlpha=4.5e4, double bMz=0.0, double width=0.209, \
	double cGamma=1e3, \
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
	
	Tire_Fiala (const d_vec &params, const d_vec &init_states);
	Tire_Fiala (const std::string &filename);

	void push_con_states (d_vec &con_states);
	
	void pull_con_states (const d_vec &con_states);
	void update_pv (const d_vec &inputs, d_vec &outputs);
	void update_fm (const d_vec &inputs, d_vec &outputs);
	void update_drv (d_vec &outputs);

private:
	//parameters
	double m_Lrelx, m_Lrely, m_alpha_min, m_alpha_max, \
	m_mu_min, m_mu_max, m_aMy, m_bMy, m_cMy, \
	m_alphaMy, m_betaMy, m_Fz_min, m_Fz_max, \
	m_cKappa, m_cAlpha, m_bMz, m_width, m_cGamma;
	//inputs
	double m_Whl_omega = NaN;
	double m_Sus_vx = NaN;
	double m_Sus_vy = NaN;
	double m_Sus_vz = NaN;
	double m_Sus_gamma = NaN;
	double m_Sus_str = NaN;
	double m_Sus_psidot = NaN;
	double m_Whl_Re = NaN;
	double m_Whl_rhoz = NaN;
	double m_Sus_Fz = NaN;
	double m_Gnd_scale = NaN;
	double m_Tir_Prs = NaN;
	double m_Air_Tamb = NaN;
	//continuous states
	double m_kappa, m_alpha_prime, m_Mroll;
	//continuous states derivatives
	double m_drv_kappa, m_drv_alpha_prime, m_drv_Mroll;
	//discrete states
	//middle variables
	double m_Tir_vx = NaN;
	double m_Tir_vy = NaN;
	double m_Tir_vz = NaN;
	double m_Tir_gamma = NaN;
	double m_Tir_str = NaN;
	double m_Tir_psidot = NaN;
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
	double m_Sus_Fx = NaN;
	double m_Sus_Fy = NaN;
	//double m_Sus_Fz = NaN;
	double m_Tir_Mx = NaN;
	double m_Tir_My = NaN;
	double m_Tir_Mz = NaN;
};

}	//end of name space
#endif //TIRE_FIALA_HPP
