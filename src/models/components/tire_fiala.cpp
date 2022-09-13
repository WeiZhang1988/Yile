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

NMSPC::Tire_Fiala::Tire_Fiala (const d_vec &params, \
const d_vec &init_states) {
	//initialize parameters
    m_Lrelx = params[0];
    m_Lrely = params[1];
    m_alpha_min = params[2];
    m_alpha_max = params[3];
	m_mu_min = params[4];
	m_mu_max = params[5];
	m_aMy = params[6];
	m_bMy = params[7];
	m_cMy = params[8];
	m_alphaMy = params[9];
	m_betaMy = params[10];
	m_Fz_min = params[11];
	m_Fz_max = params[12];
	m_cKappa = params[13];
	m_cAlpha = params[14];
	m_bMz = params[15];
	m_width = params[16];
	m_cGamma = params[17];
    //initialize continuous and discrete states
	m_kappa = init_states[0];
    m_alpha_prime = init_states[1];
	m_Mroll = init_states[2];
}

NMSPC::Tire_Fiala::Tire_Fiala (const std::string &filename) {
	boost::property_tree::ptree tree;
    if (std::filesystem::exists(filename)){
        boost::property_tree::read_json(filename,tree);
        if ("tire-fiala" == tree.get<std::string>("type")) {
            m_Lrelx = tree.get<double>("Lrelx");
            m_Lrely = tree.get<double>("Lrely");
            m_alpha_min = tree.get<double>("alpha_min");
            m_alpha_max = tree.get<double>("alpha_max");
            m_mu_min = tree.get<double>("mu_min");
            m_mu_max = tree.get<double>("mu_max");
            m_aMy = tree.get<double>("aMy");
            m_bMy = tree.get<double>("bMy");
            m_cMy = tree.get<double>("cMy");
            m_alphaMy = tree.get<double>("alphaMy");
            m_betaMy = tree.get<double>("betaMy");
            m_Fz_min = tree.get<double>("Fz_min");
			m_Fz_max = tree.get<double>("Fz_max");
			m_cKappa = tree.get<double>("cKappa");
            m_cAlpha = tree.get<double>("cAlpha");
            m_bMz = tree.get<double>("bMz");
			m_width = tree.get<double>("width");
			m_cGamma = tree.get<double>("cGamma");
			m_kappa = tree.get<double>("init_kappa");
			m_alpha_prime = tree.get<double>("init_alpha");
			m_Mroll = tree.get<double>("init_Mroll");
        } else {
            Tire_Fiala();
        }
    } else {
        Tire_Fiala();
    }
}

void NMSPC::Tire_Fiala::push_con_states (d_vec &con_states) {
	con_states[0] = m_kappa;
	con_states[1] = m_alpha_prime;
	con_states[2] = m_Mroll;
}

void NMSPC::Tire_Fiala::pull_con_states (const d_vec &con_states) {
	m_kappa = con_states[0];
	m_alpha_prime = con_states[1];
	m_Mroll = con_states[2];
}

void NMSPC::Tire_Fiala::update_pv (const d_vec &inputs, d_vec &outputs) {
	//pull inputs
	m_Whl_omega = inputs[0];
	m_Sus_vx = inputs[1];
	m_Sus_vy = inputs[2];
	m_Sus_vz = inputs[3];
	m_Sus_gamma = inputs[4];
	m_Sus_str = inputs[5];
	m_Sus_psidot = inputs[6];
	m_Whl_Re = inputs[7];
	m_Whl_rhoz = inputs[8];
	//frame transfer
	m_Tir_psidot = -m_Sus_psidot;
	m_Tir_gamma = m_Sus_gamma + pi;
	m_Tir_str = m_Sus_str;
	m_cos_a = cos(m_Tir_gamma);
	m_cos_c	= cos(m_Tir_str);
	m_sin_a = sin(m_Tir_gamma);
	m_sin_c	= sin(m_Tir_str);
	m_DCM_00 = m_cos_a;
	m_DCM_01 = m_sin_a;
	m_DCM_02 = 0.0;
	m_DCM_10 = -m_cos_c * m_sin_a;
	m_DCM_11 = m_cos_c * m_cos_a;
	m_DCM_12 = m_sin_c;
	m_DCM_20 = m_sin_c * m_sin_a;
	m_DCM_21 = -m_sin_c * m_cos_a;
	m_DCM_22 = m_cos_c;
	m_Tir_vx = m_DCM_00 * m_Sus_vx + m_DCM_10 * m_Sus_vy + m_DCM_20 * m_Sus_vz;
	m_Tir_vy = m_DCM_01 * m_Sus_vx + m_DCM_11 * m_Sus_vy + m_DCM_21 * m_Sus_vz;
	m_Tir_vz = m_DCM_02 * m_Sus_vx + m_DCM_12 * m_Sus_vy + m_DCM_22 * m_Sus_vz;
}

void NMSPC::Tire_Fiala::update_fm (const d_vec &inputs, d_vec &outputs) {
	//pull inputs
	m_Sus_Fz = inputs[0];
	m_Gnd_scale = inputs[1];
	m_Tir_Prs = inputs[2];
	m_Air_Tamb = inputs[3];

	//process
	m_sat_Fz = saturation(m_Sus_Fz, m_Fz_min, m_Fz_max);
	m_alpha = saturation(m_alpha_prime * tanh(abs(m_Tir_vy)), m_alpha_min, \
    m_alpha_max);
    m_tan_alpha = tan(m_alpha);
    m_mu = (m_mu_max - (m_mu_max - m_mu_min) * saturation(sqrt(pow(\
    m_kappa, 2.0) + pow(m_tan_alpha, 2.0)), 0.0, 1.0)) * m_Gnd_scale;
    m_My = -(m_aMy + abs(m_Tir_vx) * m_bMy + pow(m_Tir_vx, 2.0) * m_cMy) * \
    tanh(4.0 * m_Whl_omega) * m_Whl_Re * pow(m_sat_Fz, m_betaMy) * \
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
    	m_Mz_stick = -m_Tir_psidot * m_bMz - m_width * (1.0 - m_H) * \
    	pow(m_H, 3.0) * m_mu * m_sat_Fz * tanh(4.0 * m_alpha);
    	m_Fy_stick = m_Tir_gamma * m_cGamma - (1.0 - pow(m_H, 3.0)) * m_mu *\
    	m_sat_Fz * tanh(4.0 * m_alpha);
    	m_Tir_Mz = m_Mz_stick;
    	m_Tir_Fy = m_Fy_stick;
    } else {
    	m_Mz_slide = m_Tir_psidot * m_bMz;
    	m_Fy_slide = -m_sat_Fz * m_mu * tanh(4.0 * m_alpha);
    	m_Tir_Mz = m_Mz_slide;
    	m_Tir_Fy = m_Fy_slide;
    }
    m_Tir_Mx = cos(m_Tir_gamma) * m_Tir_Fy * m_Whl_Re;
    m_Tir_Fz = m_sat_Fz;
    m_Tir_My = m_Mroll;

	//frame transfer
	m_Sus_Fx = m_DCM_00 * m_Tir_Fx + m_DCM_01 * m_Tir_Fy + m_DCM_02 * m_Tir_Fz;
	m_Sus_Fy = m_DCM_10 * m_Tir_Fx + m_DCM_11 * m_Tir_Fy + m_DCM_12 * m_Tir_Fz;
	m_Sus_Fz = m_DCM_20 * m_Tir_Fx + m_DCM_21 * m_Tir_Fy + m_DCM_22 * m_Tir_Fz;
    //push outputs
    outputs[0] = m_Sus_Fx;
    outputs[1] = m_Sus_Fy;
    outputs[2] = m_Sus_Fz;
    outputs[3] = m_Tir_Mx;
    outputs[4] = m_Tir_My;
    outputs[5] = m_Tir_Mz;
    
}

void NMSPC::Tire_Fiala::update_drv (d_vec &outputs) {
	//process
	m_drv_kappa = (m_Whl_omega * m_Whl_Re - m_Tir_vx - abs(m_Tir_vx) * \
    m_kappa) / m_Lrelx;
    m_drv_alpha_prime = (m_Tir_vy - abs(m_Tir_vx) * m_tan_alpha) / m_Lrely;
    m_drv_Mroll = ((1.0 - abs(m_Tir_vx)) * 2 * pi + abs(m_Tir_vx - \
    m_Whl_omega * m_Whl_Re) / saturation(m_Lrelx, 0.01, 10.0)) * \
    (m_My - m_Mroll);
	//push outputs
    outputs[0] = m_drv_kappa;
    outputs[1] = m_drv_alpha_prime;
    outputs[2] = m_drv_Mroll;
}
