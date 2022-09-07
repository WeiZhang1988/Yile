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
#include "low_pass_filter.hpp"

void NMSPC::Low_Pass_Filter::push_con_states (d_vec &con_states) {
	con_states[0] = m_toe_fl;
	con_states[1] = m_toe_fr;
	con_states[2] = m_toe_rl;
	con_states[3] = m_toe_rr;
	con_states[4] = m_camber_fl;
	con_states[5] = m_camber_fr;
	con_states[6] = m_camber_rl;
	con_states[7] = m_camber_rr;
	con_states[8] = m_Fz_fl;
	con_states[9] = m_Fz_fr;
	con_states[10] = m_Fz_rl;
	con_states[11] = m_Fz_rr;
}

void NMSPC::Low_Pass_Filter::pull_con_states (d_vec &con_states) {
	m_toe_fl = con_states[0];
	m_toe_fr = con_states[1];
	m_toe_rl = con_states[2];
	m_toe_rr = con_states[3];
	m_camber_fl = con_states[4];
	m_camber_fr = con_states[5];
	m_camber_rl = con_states[6];
	m_camber_rr = con_states[7];
	m_Fz_fl = con_states[8];
	m_Fz_fr = con_states[9];
	m_Fz_rl = con_states[10];
	m_Fz_rr = con_states[11];
}

void NMSPC::Low_Pass_Filter::update_pv (const d_vec &inputs, d_vec &outputs) {
	m_output_toe_fl = m_toe_fl;
	m_output_toe_fr = m_toe_fr;
	m_output_toe_rl = m_toe_rl;
	m_output_toe_rr = m_toe_rr;
	m_output_camber_fl = m_camber_fl;
	m_output_camber_fr = m_camber_fr;
	m_output_camber_rl = m_camber_rl;
	m_output_camber_rr = m_camber_rr;
	m_output_Fz_fl = m_Fz_fl;
	m_output_Fz_fr = m_Fz_fr;
	m_output_Fz_rl = m_Fz_rl;
	m_output_Fz_rr = m_Fz_rr;
	outputs[0] = m_output_toe_fl;
	outputs[1] = m_output_toe_fr;
	outputs[2] = m_output_toe_rl;
	outputs[3] = m_output_toe_rr;
	outputs[4] = m_output_camber_fl;
	outputs[5] = m_output_camber_fr;
	outputs[6] = m_output_camber_rl;
	outputs[7] = m_output_camber_rr;
	outputs[8] = m_output_Fz_fl;
	outputs[9] = m_output_Fz_fr;
	outputs[10] = m_output_Fz_rl;
	outputs[11] = m_output_Fz_rr;
}

void NMSPC::Low_Pass_Filter::update_fm (const d_vec &inputs, d_vec &outputs) {
	m_input_toe_fl = inputs[0];
	m_input_toe_fr = inputs[1];
	m_input_toe_rl = inputs[2];
	m_input_toe_rr = inputs[3];
	m_input_camber_fl = inputs[4];
	m_input_camber_fr = inputs[5];
	m_input_camber_rl = inputs[6];
	m_input_camber_rr = inputs[7];
	m_input_Fz_fl = inputs[8];
	m_input_Fz_fr = inputs[9];
	m_input_Fz_rl = inputs[10];
	m_input_Fz_rr = inputs[11];
}

void NMSPC::Low_Pass_Filter::update_drv (d_vec &outputs) {
	m_drv_toe_fl = (m_input_toe_fl - m_toe_fl) * m_wc;
	m_drv_toe_fr = (m_input_toe_fr - m_toe_fr) * m_wc;
	m_drv_toe_rl = (m_input_toe_rl - m_toe_rl) * m_wc;
	m_drv_toe_rr = (m_input_toe_rr - m_toe_rr) * m_wc;
	m_drv_camber_fl = (m_input_camber_fl - m_camber_fl) * m_wc;
	m_drv_camber_fr = (m_input_camber_fr - m_camber_fr) * m_wc;
	m_drv_camber_rl = (m_input_camber_rl - m_camber_rl) * m_wc;
	m_drv_camber_rr = (m_input_camber_rr - m_camber_rr) * m_wc;
	m_drv_Fz_fl = (m_input_Fz_rr - m_Fz_rr) * m_wc;
	m_drv_Fz_fr = (m_input_Fz_rr - m_Fz_rr) * m_wc;
	m_drv_Fz_rl = (m_input_Fz_rr - m_Fz_rr) * m_wc;
	m_drv_Fz_rr = (m_input_Fz_rr - m_Fz_rr) * m_wc;
	outputs[0] = m_drv_toe_fl;
	outputs[1] = m_drv_toe_fr;
	outputs[2] = m_drv_toe_rl;
	outputs[3] = m_drv_toe_rr;
	outputs[4] = m_drv_camber_fl;
	outputs[5] = m_drv_camber_fr;
	outputs[6] = m_drv_camber_rl;
	outputs[7] = m_drv_camber_rr;
	outputs[8] = m_drv_Fz_fl;
	outputs[9] = m_drv_Fz_fr;
	outputs[10] = m_drv_Fz_rl;
	outputs[11] = m_drv_Fz_rr;
}
