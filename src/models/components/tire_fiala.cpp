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
#include "tire_fiala.hpp"

void NMSPC::Tire_Fiala::push_con_states(d_vec &con_states) {
	con_states[0] = m_kappa;
	con_states[1] = m_alpha_prime;
	con_states[2] = m_Mroll;
	con_states[3] = m_Sus_lpf_str;
	con_states[4] = m_Sus_lpf_gamma;
	con_states[5] = m_Sus_lpf_Fz;
}

void NMSPC::Tire_Fiala::pull_con_states(const d_vec &con_states) {
	m_kappa = con_states[0];
	m_alpha_prime = con_states[1];
	m_Mroll = con_states[2];
	m_Sus_lpf_str = con_states[3];
	m_Sus_lpf_gamma = con_states[4];
	m_Sus_lpf_Fz = con_states[5];
}

void NMSPC::Tire_Fiala::pull_pv(const double &Tir_omega, const double &Tir_rhoz, const double &Tir_Re, const double &Sus_vx, const double &Sus_vy, const double &Sus_vz,\
const double &Sus_gamma, const double &Sus_str, const double &Sus_r) {
	//pull inputs
	m_Tir_omega = Tir_omega;
	m_Tir_rhoz = Tir_rhoz;
	m_Tir_Re = Tir_Re;
	m_Sus_vx = Sus_vx;
	m_Sus_vy = Sus_vy;
	m_Sus_vz = Sus_vz;
	m_Sus_gamma = Sus_gamma;
	m_Sus_str = Sus_str;
	m_Sus_r = Sus_r;
	//frame transfer
	m_Tir_r = -m_Sus_r;
	m_Tir_gamma = m_Sus_lpf_gamma + pi;
	m_Tir_str = m_Sus_lpf_str;

	m_cos_a = cos(m_Tir_gamma);
	m_cos_b = cos(0.0);
	m_cos_c	= cos(m_Tir_str);
	m_sin_a = sin(m_Tir_gamma);
	m_sin_b = sin(0.0);
	m_sin_c	= sin(m_Tir_str);

	m_DCM_00 = m_cos_c * m_cos_b;
	m_DCM_01 = m_sin_c * m_cos_b;
	m_DCM_02 = -m_sin_b;
	m_DCM_10 = m_cos_c * m_sin_b * m_sin_a - m_sin_c * m_cos_a;
	m_DCM_11 = m_sin_c * m_sin_b * m_sin_a + m_cos_c * m_cos_a;
	m_DCM_12 = m_cos_b * m_sin_a;
	m_DCM_20 = m_cos_c * m_sin_b * m_cos_a + m_sin_c * m_sin_a;
	m_DCM_21 = m_sin_c * m_sin_b * m_cos_a - m_cos_c * m_sin_a;
	m_DCM_22 = m_cos_b * m_cos_a;

	m_Tir_vx = m_DCM_00 * m_Sus_vx + m_DCM_10 * m_Sus_vy + m_DCM_20 * m_Sus_vz;
	m_Tir_vy = m_DCM_01 * m_Sus_vx + m_DCM_11 * m_Sus_vy + m_DCM_21 * m_Sus_vz;
	m_Tir_vz = m_DCM_02 * m_Sus_vx + m_DCM_12 * m_Sus_vy + m_DCM_22 * m_Sus_vz;
}

void NMSPC::Tire_Fiala::pull_fm (const double &Sus_Fz, const double &Gnd_scale, const double &Tir_Prs, const double &Air_Tamb) {
	//pull inputs
	m_Sus_Fz = Sus_Fz;
	m_Gnd_scale = Gnd_scale;
	m_Tir_Prs = Tir_Prs;
	m_Air_Tamb = Air_Tamb;

	//process
	m_sat_Fz = saturation(saturation(m_Sus_lpf_Fz,-10.0*9.8*2000,inf), m_Fz_min, m_Fz_max);
	m_alpha = saturation(m_alpha_prime * tanh(abs(m_Tir_vy)), m_alpha_min, \
    m_alpha_max);
    m_tan_alpha = tan(m_alpha);
    m_mu = (m_mu_max - (m_mu_max - m_mu_min) * saturation(sqrt(pow(\
    m_kappa, 2.0) + pow(m_tan_alpha, 2.0)), 0.0, 1.0)) * m_Gnd_scale;
    m_My = -(m_aMy + abs(m_Tir_vx) * m_bMy + pow(m_Tir_vx, 2.0) * m_cMy) * \
    tanh(4.0 * m_Tir_omega) * m_Tir_Re * pow(m_sat_Fz, m_betaMy) * \
    pow(m_Tir_Prs, m_alphaMy);
    m_kappa_critical = m_mu * m_sat_Fz / 2.0 / m_cKappa;
    if (abs(m_kappa) <= m_kappa_critical) {
    	m_Fx_stick = m_cKappa * m_kappa;
    	m_Tir_Fx = m_Fx_stick;
    } else {
    	m_Fx1 = m_mu * m_sat_Fz;
    	m_Fx2 = abs(pow(m_Fx1, 2.0) / m_cKappa / div0protect(4.0 * \
    	m_kappa, 0.01));
    	m_Fx_slide = (m_Fx1 - m_Fx2) * tanh(4.0 * m_kappa);
    	m_Tir_Fx = m_Fx_slide;
    }
    m_alpha_critical = atan(m_mu * 3.0 * m_sat_Fz / m_cAlpha);
    if (abs(m_alpha) <= m_alpha_critical) {
    	m_H = 1.0 - abs(m_tan_alpha) * m_cAlpha / (m_mu * 3.0 * m_sat_Fz);
    	m_Mz_stick = -m_Tir_r * m_bMz - m_width * (1.0 - m_H) * \
    	pow(m_H, 3.0) * m_mu * m_sat_Fz * tanh(4.0 * m_alpha);
    	m_Fy_stick = m_Sus_lpf_gamma * m_cGamma - (1.0 - pow(m_H, 3.0)) * m_mu *\
    	m_sat_Fz * tanh(4.0 * m_alpha);
    	m_Tir_Mz = m_Mz_stick;
    	m_Tir_Fy = m_Fy_stick;
    } else {
    	m_Mz_slide = m_Tir_r * m_bMz;
    	m_Fy_slide = -m_sat_Fz * m_mu * tanh(4.0 * m_alpha);
    	m_Tir_Mz = m_Mz_slide;
    	m_Tir_Fy = m_Fy_slide;
    }
    m_Tir_Mx = cos(m_Sus_lpf_gamma) * m_Tir_Fy * m_Tir_Re;
    m_Tir_Fz = m_sat_Fz;
    m_Tir_My = m_Mroll;

	//frame transfer
	double dead_zone_Tir_Fx = dead_zone(m_Tir_Fx,-5.0,5.0);
	double dead_zone_Tir_Fy = dead_zone(m_Tir_Fy,-10.0,10.0);
	m_Sus_TirFx = m_DCM_00 * dead_zone_Tir_Fx + m_DCM_01 * dead_zone_Tir_Fy + m_DCM_02 * m_Tir_Fz;
	m_Sus_TirFy = m_DCM_10 * dead_zone_Tir_Fx + m_DCM_11 * dead_zone_Tir_Fy + m_DCM_12 * m_Tir_Fz;
	m_Sus_TirFz = m_DCM_20 * dead_zone_Tir_Fx + m_DCM_21 * dead_zone_Tir_Fy + m_DCM_22 * m_Tir_Fz;
    
}

void NMSPC::Tire_Fiala::push_fm (double &Sus_TirFx, double &Sus_TirFy, double &Sus_TirFz, \
double &Tir_Fx, double &Tir_Fy, double &Tir_Fz, double &Tir_Mx, double &Tir_My, double &Tir_Mz) {
    Sus_TirFx = m_Sus_TirFx;
    Sus_TirFy = m_Sus_TirFy;
    Sus_TirFz = m_Sus_TirFz;
	Tir_Fx = m_Tir_Fx;
    Tir_Fy = m_Tir_Fy;
    Tir_Fz = m_Tir_Fz;
    Tir_Mx = m_Tir_Mx;
    Tir_My = m_Tir_My;
    Tir_Mz = m_Tir_Mz;
}

void NMSPC::Tire_Fiala::push_drv (d_vec &derivatives) {
	//process
	m_drv_kappa = (m_Tir_omega * m_Tir_Re - m_Tir_vx - abs(m_Tir_vx) * \
    m_kappa) / m_Lrelx;
    m_drv_alpha_prime = (m_Tir_vy - abs(m_Tir_vx) * m_tan_alpha) / m_Lrely;
    m_drv_Mroll = ((1.0 - abs(m_Tir_vx)) * 2 * pi + abs(m_Tir_vx - \
    m_Tir_omega * m_Tir_Re) / saturation(m_Lrelx, 0.01, 10.0)) * \
    (m_My - m_Mroll);
	m_drv_Sus_lpf_str = (m_Sus_str - m_Sus_lpf_str) * m_lpf_wc;
	m_drv_Sus_lpf_gamma = (m_Sus_gamma - m_Sus_lpf_gamma) * m_lpf_wc;
	m_drv_Sus_lpf_Fz = (m_Sus_Fz - m_Sus_lpf_Fz) * m_lpf_wc;
	//push outputs
    derivatives[0] = m_drv_kappa;
    derivatives[1] = m_drv_alpha_prime;
    derivatives[2] = m_drv_Mroll;
	derivatives[3] = m_drv_Sus_lpf_str;
	derivatives[4] = m_drv_Sus_lpf_gamma;
	derivatives[5] = m_drv_Sus_lpf_Fz;
}
