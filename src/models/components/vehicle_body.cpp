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
#include "vehicle_body.hpp"

void NMSPC::Vehicle_Body::push_con_states (d_vec &con_states) {
	con_states[0] = m_xe_x;
	con_states[1] = m_xe_y;
	con_states[2] = m_xe_z;
	con_states[3] = m_vb_x;
	con_states[4] = m_vb_y;
	con_states[5] = m_vb_z;
	con_states[6] = m_phai;
	con_states[7] = m_theta;
	con_states[8] = m_psi;
	con_states[9] = m_p;
	con_states[10] = m_q;
	con_states[11] = m_r;
}

void NMSPC::Vehicle_Body::pull_con_states (const d_vec &con_states) {
	m_xe_x	= con_states[0];
	m_xe_y	= con_states[1];
	m_xe_z	= con_states[2];
	m_vb_x	= con_states[3];
	m_vb_y	= con_states[4];
	m_vb_z	= con_states[5];
	m_phai	= con_states[6];
	m_theta	= con_states[7];
	m_psi	= con_states[8];
	m_p		= con_states[9];
	m_q		= con_states[10];
	m_r		= con_states[11];
}

void NMSPC::Vehicle_Body::pull_pv(const double &Air_Wx, const double &Air_Wy, const double &Air_Wz) {
	m_Air_Wx = Air_Wx;
	m_Air_Wy = Air_Wy;
	m_Air_Wz = Air_Wz;

	//process
	m_cos_phai 	= cos(m_phai);
	m_cos_theta	= cos(m_theta);
	m_cos_psi	= cos(m_psi);
	m_sin_phai 	= sin(m_phai);
	m_sin_theta	= sin(m_theta);
	m_sin_psi	= sin(m_psi);
	m_DCM_00	= m_cos_theta * m_cos_phai;
	m_DCM_01	= m_cos_theta * m_sin_phai;
	m_DCM_02	= -m_sin_theta;
	m_DCM_10	= m_sin_psi * m_sin_theta * m_cos_phai - m_cos_psi * m_sin_phai;
	m_DCM_11	= m_sin_psi * m_sin_theta * m_sin_phai + m_cos_psi * m_cos_phai;
	m_DCM_12	= m_sin_psi * m_cos_theta;
	m_DCM_20	= m_cos_psi * m_sin_theta * m_cos_phai + m_sin_psi * m_sin_phai;
	m_DCM_21	= m_cos_psi * m_sin_theta * m_sin_phai - m_sin_psi * m_cos_phai;
	m_DCM_22	= m_cos_psi * m_cos_theta;

	m_ve_x = m_DCM_00 * m_vb_x + m_DCM_10 * m_vb_y + m_DCM_20 * m_vb_z;
	m_ve_y = m_DCM_01 * m_vb_x + m_DCM_11 * m_vb_y + m_DCM_21 * m_vb_z;
	m_ve_z = m_DCM_02 * m_vb_x + m_DCM_12 * m_vb_y + m_DCM_22 * m_vb_z;
	
	m_xe_x_c = m_xe_x;
	m_xe_y_c = m_xe_y;
	m_xe_z_c = m_xe_z;
	m_ve_x_c = m_ve_x;
	m_ve_y_c = m_ve_y;
	m_ve_z_c = m_ve_z;
	m_vb_x_c = m_vb_x;
	m_vb_y_c = m_vb_y;
	m_vb_z_c = m_vb_z;
	m_phai_c = m_phai;
	m_theta_c = m_theta;
	m_psi_c = m_psi;
	m_p_c = m_p;
	m_q_c = m_q;
	m_r_c = m_r;
	m_beta_c = atan(m_vb_y_c / div0protect_abs(m_vb_x_c, m_xdot_tol));
	
	double tmpR_x, tmpR_y, tmpR_z;
	double tmpRes1_x, tmpRes1_y, tmpRes1_z;
	double tmpRes2_x, tmpRes2_y, tmpRes2_z;
	
	tmpR_x = m_Rbar_x + m_longOff;
	tmpR_y = m_Rbar_y + m_latOff;
	tmpR_z = m_Rbar_z + m_vertOff;
	tmpRes1_x = m_DCM_00 * tmpR_x + m_DCM_10 * tmpR_y + m_DCM_20 * tmpR_z;
	tmpRes1_y = m_DCM_01 * tmpR_x + m_DCM_11 * tmpR_y + m_DCM_21 * tmpR_z;
	tmpRes1_z = m_DCM_02 * tmpR_x + m_DCM_12 * tmpR_y + m_DCM_22 * tmpR_z;
	m_xe_x_o = m_xe_x_c + tmpRes1_x;
	m_xe_y_o = m_xe_y_c + tmpRes1_y;
	m_xe_z_o = m_xe_z_c + tmpRes1_z;
	tmpRes1_x = m_q * tmpR_z - m_r * tmpR_y;
	tmpRes1_y = m_r * tmpR_x - m_p * tmpR_z;
	tmpRes1_z = m_p * tmpR_y - m_q * tmpR_x;
	tmpRes2_x = m_DCM_00 * tmpRes1_x + m_DCM_10 * tmpRes1_y + m_DCM_20 * tmpRes1_z;
	tmpRes2_y = m_DCM_01 * tmpRes1_x + m_DCM_11 * tmpRes1_y + m_DCM_21 * tmpRes1_z;
	tmpRes2_z = m_DCM_02 * tmpRes1_x + m_DCM_12 * tmpRes1_y + m_DCM_22 * tmpRes1_z;
	m_ve_x_o = m_ve_x_c + tmpRes2_x;
	m_ve_y_o = m_ve_y_c + tmpRes2_y;
	m_ve_z_o = m_ve_z_c + tmpRes2_z;
	m_xb_x_o = tmpR_x;
	m_xb_y_o = tmpR_y;
	m_xb_z_o = tmpR_z;
	m_vb_x_o = tmpRes1_x + m_vb_x_c;
	m_vb_y_o = tmpRes1_y + m_vb_y_c;
	m_vb_z_o = tmpRes1_z + m_vb_z_c;
	m_beta_o = atan(m_vb_y_o / div0protect_abs(m_vb_x_o, m_xdot_tol));
	
	tmpR_x = m_HPbar_fl_x;
	tmpR_y = m_HPbar_fl_y;
	tmpR_z = m_HPbar_fl_z;
	tmpRes1_x = m_DCM_00 * tmpR_x + m_DCM_10 * tmpR_y + m_DCM_20 * tmpR_z;
	tmpRes1_y = m_DCM_01 * tmpR_x + m_DCM_11 * tmpR_y + m_DCM_21 * tmpR_z;
	tmpRes1_z = m_DCM_02 * tmpR_x + m_DCM_12 * tmpR_y + m_DCM_22 * tmpR_z;
	m_xe_x_fl = m_xe_x_c + tmpRes1_x;
	m_xe_y_fl = m_xe_y_c + tmpRes1_y;
	m_xe_z_fl = m_xe_z_c + tmpRes1_z;
	tmpRes1_x = m_q * tmpR_z - m_r * tmpR_y;
	tmpRes1_y = m_r * tmpR_x - m_p * tmpR_z;
	tmpRes1_z = m_p * tmpR_y - m_q * tmpR_x;
	tmpRes2_x = m_DCM_00 * tmpRes1_x + m_DCM_10 * tmpRes1_y + m_DCM_20 * tmpRes1_z;
	tmpRes2_y = m_DCM_01 * tmpRes1_x + m_DCM_11 * tmpRes1_y + m_DCM_21 * tmpRes1_z;
	tmpRes2_z = m_DCM_02 * tmpRes1_x + m_DCM_12 * tmpRes1_y + m_DCM_22 * tmpRes1_z;
	m_ve_x_fl = m_ve_x_c + tmpRes2_x;
	m_ve_y_fl = m_ve_y_c + tmpRes2_y;
	m_ve_z_fl = m_ve_z_c + tmpRes2_z;
	m_xb_x_fl = m_HPbar_fl_x;
	m_xb_y_fl = m_HPbar_fl_y;
	m_xb_z_fl = m_HPbar_fl_z;
	m_vb_x_fl = tmpRes1_x + m_vb_x_c;
	m_vb_y_fl = tmpRes1_y + m_vb_y_c;
	m_vb_z_fl = tmpRes1_z + m_vb_z_c;
	
	tmpR_x = m_HPbar_fr_x;
	tmpR_y = m_HPbar_fr_y;
	tmpR_z = m_HPbar_fr_z;
	tmpRes1_x = m_DCM_00 * tmpR_x + m_DCM_10 * tmpR_y + m_DCM_20 * tmpR_z;
	tmpRes1_y = m_DCM_01 * tmpR_x + m_DCM_11 * tmpR_y + m_DCM_21 * tmpR_z;
	tmpRes1_z = m_DCM_02 * tmpR_x + m_DCM_12 * tmpR_y + m_DCM_22 * tmpR_z;
	m_xe_x_fr = m_xe_x_c + tmpRes1_x;
	m_xe_y_fr = m_xe_y_c + tmpRes1_y;
	m_xe_z_fr = m_xe_z_c + tmpRes1_z;
	tmpRes1_x = m_q * tmpR_z - m_r * tmpR_y;
	tmpRes1_y = m_r * tmpR_x - m_p * tmpR_z;
	tmpRes1_z = m_p * tmpR_y - m_q * tmpR_x;
	tmpRes2_x = m_DCM_00 * tmpRes1_x + m_DCM_10 * tmpRes1_y + m_DCM_20 * tmpRes1_z;
	tmpRes2_y = m_DCM_01 * tmpRes1_x + m_DCM_11 * tmpRes1_y + m_DCM_21 * tmpRes1_z;
	tmpRes2_z = m_DCM_02 * tmpRes1_x + m_DCM_12 * tmpRes1_y + m_DCM_22 * tmpRes1_z;
	m_ve_x_fr = m_ve_x_c + tmpRes2_x;
	m_ve_y_fr = m_ve_y_c + tmpRes2_y;
	m_ve_z_fr = m_ve_z_c + tmpRes2_z;
	m_xb_x_fr = m_HPbar_fr_x;
	m_xb_y_fr = m_HPbar_fr_y;
	m_xb_z_fr = m_HPbar_fr_z;
	m_vb_x_fr = tmpRes1_x + m_vb_x_c;
	m_vb_y_fr = tmpRes1_y + m_vb_y_c;
	m_vb_z_fr = tmpRes1_z + m_vb_z_c;
	
	tmpR_x = m_HPbar_rl_x;
	tmpR_y = m_HPbar_rl_y;
	tmpR_z = m_HPbar_rl_z;
	tmpRes1_x = m_DCM_00 * tmpR_x + m_DCM_10 * tmpR_y + m_DCM_20 * tmpR_z;
	tmpRes1_y = m_DCM_01 * tmpR_x + m_DCM_11 * tmpR_y + m_DCM_21 * tmpR_z;
	tmpRes1_z = m_DCM_02 * tmpR_x + m_DCM_12 * tmpR_y + m_DCM_22 * tmpR_z;
	m_xe_x_rl = m_xe_x_c + tmpRes1_x;
	m_xe_y_rl = m_xe_y_c + tmpRes1_y;
	m_xe_z_rl = m_xe_z_c + tmpRes1_z;
	tmpRes1_x = m_q * tmpR_z - m_r * tmpR_y;
	tmpRes1_y = m_r * tmpR_x - m_p * tmpR_z;
	tmpRes1_z = m_p * tmpR_y - m_q * tmpR_x;
	tmpRes2_x = m_DCM_00 * tmpRes1_x + m_DCM_10 * tmpRes1_y + m_DCM_20 * tmpRes1_z;
	tmpRes2_y = m_DCM_01 * tmpRes1_x + m_DCM_11 * tmpRes1_y + m_DCM_21 * tmpRes1_z;
	tmpRes2_z = m_DCM_02 * tmpRes1_x + m_DCM_12 * tmpRes1_y + m_DCM_22 * tmpRes1_z;
	m_ve_x_rl = m_ve_x_c + tmpRes2_x;
	m_ve_y_rl = m_ve_y_c + tmpRes2_y;
	m_ve_z_rl = m_ve_z_c + tmpRes2_z;
	m_xb_x_rl = m_HPbar_rl_x;
	m_xb_y_rl = m_HPbar_rl_y;
	m_xb_z_rl = m_HPbar_rl_z;
	m_vb_x_rl = tmpRes1_x + m_vb_x_c;
	m_vb_y_rl = tmpRes1_y + m_vb_y_c;
	m_vb_z_rl = tmpRes1_z + m_vb_z_c;
	
	tmpR_x = m_HPbar_rr_x;
	tmpR_y = m_HPbar_rr_y;
	tmpR_z = m_HPbar_rr_z;
	tmpRes1_x = m_DCM_00 * tmpR_x + m_DCM_10 * tmpR_y + m_DCM_20 * tmpR_z;
	tmpRes1_y = m_DCM_01 * tmpR_x + m_DCM_11 * tmpR_y + m_DCM_21 * tmpR_z;
	tmpRes1_z = m_DCM_02 * tmpR_x + m_DCM_12 * tmpR_y + m_DCM_22 * tmpR_z;
	m_xe_x_rr = m_xe_x_c + tmpRes1_x;
	m_xe_y_rr = m_xe_y_c + tmpRes1_y;
	m_xe_z_rr = m_xe_z_c + tmpRes1_z;
	tmpRes1_x = m_q * tmpR_z - m_r * tmpR_y;
	tmpRes1_y = m_r * tmpR_x - m_p * tmpR_z;
	tmpRes1_z = m_p * tmpR_y - m_q * tmpR_x;
	tmpRes2_x = m_DCM_00 * tmpRes1_x + m_DCM_10 * tmpRes1_y + m_DCM_20 * tmpRes1_z;
	tmpRes2_y = m_DCM_01 * tmpRes1_x + m_DCM_11 * tmpRes1_y + m_DCM_21 * tmpRes1_z;
	tmpRes2_z = m_DCM_02 * tmpRes1_x + m_DCM_12 * tmpRes1_y + m_DCM_22 * tmpRes1_z;
	m_ve_x_rr = m_ve_x_c + tmpRes2_x;
	m_ve_y_rr = m_ve_y_c + tmpRes2_y;
	m_ve_z_rr = m_ve_z_c + tmpRes2_z;
	m_xb_x_rr = m_HPbar_rr_x;
	m_xb_y_rr = m_HPbar_rr_y;
	m_xb_z_rr = m_HPbar_rr_z;
	m_vb_x_rr = tmpRes1_x + m_vb_x_c;
	m_vb_y_rr = tmpRes1_y + m_vb_y_c;
	m_vb_z_rr = tmpRes1_z + m_vb_z_c;

}

void NMSPC::Vehicle_Body::push_pv(double &h_c, double &xe_x_c, double &xe_y_c, double &xe_z_c, double &ve_x_c, double &ve_y_c, double &ve_z_c, \
double &vb_x_c, double &vb_y_c, double &vb_z_c, double &phai_c, double &theta_c, double &psi_c, double &p_c, double &q_c, double &r_c, double &beta_c, \
double &xe_x_o, double &xe_y_o, double &xe_z_o, double &ve_x_o, double &ve_y_o, double &ve_z_o, \
double &xb_x_o, double &xb_y_o, double &xb_z_o, double &vb_x_o, double &vb_y_o, double &vb_z_o, double &beta_o, \
double &xe_x_fl, double &xe_y_fl, double &xe_z_fl, double &ve_x_fl, double &ve_y_fl, double &ve_z_fl, \
double &xb_x_fl, double &xb_y_fl, double &xb_z_fl, double &vb_x_fl, double &vb_y_fl, double &vb_z_fl, \
double &xe_x_fr, double &xe_y_fr, double &xe_z_fr, double &ve_x_fr, double &ve_y_fr, double &ve_z_fr, \
double &xb_x_fr, double &xb_y_fr, double &xb_z_fr, double &vb_x_fr, double &vb_y_fr, double &vb_z_fr, \
double &xe_x_rl, double &xe_y_rl, double &xe_z_rl, double &ve_x_rl, double &ve_y_rl, double &ve_z_rl, \
double &xb_x_rl, double &xb_y_rl, double &xb_z_rl, double &vb_x_rl, double &vb_y_rl, double &vb_z_rl, \
double &xe_x_rr, double &xe_y_rr, double &xe_z_rr, double &ve_x_rr, double &ve_y_rr, double &ve_z_rr, \
double &xb_x_rr, double &xb_y_rr, double &xb_z_rr, double &vb_x_rr, double &vb_y_rr, double &vb_z_rr) { 
	//push outputs
	h_c = m_h;
	xe_x_c = m_xe_x_c;
	xe_y_c = m_xe_y_c;
	xe_z_c = m_xe_z_c;
	ve_x_c = m_ve_x_c;
	ve_y_c = m_ve_y_c;
	ve_z_c = m_ve_z_c;
	vb_x_c = m_vb_x_c;
	vb_y_c = m_vb_y_c;
	vb_z_c = m_vb_z_c;
	phai_c = m_phai_c;
	theta_c = m_theta_c;
	psi_c = m_psi_c;
	p_c = m_p_c;
	q_c = m_q_c;
	r_c = m_r_c;
	beta_c = m_beta_c;
	xe_x_o = m_xe_x_o;
	xe_y_o = m_xe_y_o;
	xe_z_o = m_xe_z_o;
	ve_x_o = m_ve_x_o;
	ve_y_o = m_ve_y_o;
	ve_z_o = m_ve_z_o;
	xb_x_o = m_xb_x_o;
	xb_y_o = m_xb_y_o;
	xb_z_o = m_xb_z_o;
	vb_x_o = m_vb_x_o;
	vb_y_o = m_vb_y_o;
	vb_z_o = m_vb_z_o;
	beta_o = m_beta_o;
	xe_x_fl = m_xe_x_fl;
	xe_y_fl = m_xe_y_fl;
	xe_z_fl = m_xe_z_fl;
	ve_x_fl = m_ve_x_fl;
	ve_y_fl = m_ve_y_fl;
	ve_z_fl = m_ve_z_fl;
	xb_x_fl = m_xb_x_fl;
	xb_y_fl = m_xb_y_fl;
	xb_z_fl = m_xb_z_fl;
	vb_x_fl = m_vb_x_fl;
	vb_y_fl = m_vb_y_fl;
	vb_z_fl = m_vb_z_fl;
	xe_x_fr = m_xe_x_fr;
	xe_y_fr = m_xe_y_fr;
	xe_z_fr = m_xe_z_fr;
	ve_x_fr = m_ve_x_fr;
	ve_y_fr = m_ve_y_fr;
	ve_z_fr = m_ve_z_fr;
	xb_x_fr = m_xb_x_fr;
	xb_y_fr = m_xb_y_fr;
	xb_z_fr = m_xb_z_fr;
	vb_x_fr = m_vb_x_fr;
	vb_y_fr = m_vb_y_fr;
	vb_z_fr = m_vb_z_fr;
	xe_x_rl = m_xe_x_rl;
	xe_y_rl = m_xe_y_rl;
	xe_z_rl = m_xe_z_rl;
	ve_x_rl = m_ve_x_rl;
	ve_y_rl = m_ve_y_rl;
	ve_z_rl = m_ve_z_rl;
	xb_x_rl = m_xb_x_rl;
	xb_y_rl = m_xb_y_rl;
	xb_z_rl = m_xb_z_rl;
	vb_x_rl = m_vb_x_rl;
	vb_y_rl = m_vb_y_rl;
	vb_z_rl = m_vb_z_rl;
	xe_x_rr = m_xe_x_rr;
	xe_y_rr = m_xe_y_rr;
	xe_z_rr = m_xe_z_rr;
	ve_x_rr = m_ve_x_rr;
	ve_y_rr = m_ve_y_rr;
	ve_z_rr = m_ve_z_rr;
	xb_x_rr = m_xb_x_rr;
	xb_y_rr = m_xb_y_rr;
	xb_z_rr = m_xb_z_rr;
	vb_x_rr = m_vb_x_rr;
	vb_y_rr = m_vb_y_rr;
	vb_z_rr = m_vb_z_rr;
	
}

void NMSPC::Vehicle_Body::pull_fm(const double &Air_Tair, \
const double &Sus_Fx_fl, const double &Sus_Fx_fr, const double &Sus_Fx_rl, const double &Sus_Fx_rr, \
const double &Sus_Fy_fl, const double &Sus_Fy_fr, const double &Sus_Fy_rl, const double &Sus_Fy_rr, \
const double &Sus_Fz_fl, const double &Sus_Fz_fr, const double &Sus_Fz_rl, const double &Sus_Fz_rr, \
const double &Sus_Mx_fl, const double &Sus_Mx_fr, const double &Sus_Mx_rl, const double &Sus_Mx_rr ,\
const double &Sus_My_fl, const double &Sus_My_fr, const double &Sus_My_rl, const double &Sus_My_rr, \
const double &Sus_Mz_fl, const double &Sus_Mz_fr, const double &Sus_Mz_rl, const double &Sus_Mz_rr, \
const double &Ext_Fx_ext, const double &Ext_Fy_ext, const double &Ext_Fz_ext, \
const double &Ext_Mx_ext, const double &Ext_My_ext, const double &Ext_Mz_ext) {
	//pull inputs
	m_Air_Tair  = Air_Tair;
	m_Sus_Fx_fl = Sus_Fx_fl;
	m_Sus_Fx_fr = Sus_Fx_fr;
	m_Sus_Fx_rl = Sus_Fx_rl;
	m_Sus_Fx_rr = Sus_Fx_rr;
	m_Sus_Fy_fl = Sus_Fy_fl;
	m_Sus_Fy_fr = Sus_Fy_fr;
	m_Sus_Fy_rl = Sus_Fy_rl;
	m_Sus_Fy_rr = Sus_Fy_rr;
	m_Sus_Fz_fl = Sus_Fz_fl;
	m_Sus_Fz_fr = Sus_Fz_fr;
	m_Sus_Fz_rl = Sus_Fz_rl;
	m_Sus_Fz_rr = Sus_Fz_rr;
	m_Sus_Mx_fl = Sus_Mx_fl;
	m_Sus_Mx_fr = Sus_Mx_fr;
	m_Sus_Mx_rl = Sus_Mx_rl;
	m_Sus_Mx_rr = Sus_Mx_rr;
	m_Sus_My_fl = Sus_My_fl;
	m_Sus_My_fr = Sus_My_fr;
	m_Sus_My_rl = Sus_My_rl;
	m_Sus_My_rr = Sus_My_rr;
	m_Sus_Mz_fl = Sus_Mz_fl;
	m_Sus_Mz_fr = Sus_Mz_fr;
	m_Sus_Mz_rl = Sus_Mz_rl;
	m_Sus_Mz_rr = Sus_Mz_rr;
	m_Ext_Fx_ext = Ext_Fx_ext;
	m_Ext_Fy_ext = Ext_Fy_ext; 
	m_Ext_Fz_ext = Ext_Fz_ext;
	m_Ext_Mx_ext = Ext_Mx_ext;
	m_Ext_My_ext = Ext_My_ext;
	m_Ext_Mz_ext = Ext_Mz_ext;
	
	//process
	
	m_F_VehiclB_x = m_Sus_Fx_fl + m_Sus_Fx_fr + m_Sus_Fx_rl + m_Sus_Fx_rr;
	m_F_VehiclB_y = m_Sus_Fy_fl + m_Sus_Fy_fr + m_Sus_Fy_rl + m_Sus_Fy_rr;
	m_F_VehiclB_z = m_Sus_Fz_fl + m_Sus_Fz_fr + m_Sus_Fz_rl + m_Sus_Fz_rr;
	m_M_roll = -m_Sus_Fz_fl * m_Wbar_fl + m_Sus_Fz_fr * m_Wbar_fr - \
	m_Sus_Fz_rl * m_Wbar_rl + m_Sus_Fz_rr * m_Wbar_rr - \
	m_F_VehiclB_y * m_Xbar_h;
	m_M_pitch = -(m_Sus_Fz_fl + m_Sus_Fz_fr) * m_Xbar_a + \
	(m_Sus_Fz_rl + m_Sus_Fz_rr) * m_Xbar_b + 
	m_F_VehiclB_x * m_Xbar_h;
	m_M_yaw = m_Sus_Fx_fl * m_Wbar_fl - m_Sus_Fx_fr *  m_Wbar_fr + \
	m_Sus_Fx_rl * m_Wbar_rl - m_Sus_Fx_rr * m_Wbar_rr + \
	(m_Sus_Fy_fl + m_Sus_Fy_fr) * m_Xbar_a - (m_Sus_Fy_rl + m_Sus_Fy_rr) * m_Xbar_b;
	
	calculate_gravity();
	calculate_aero_drag();
	
	m_Fb_x = m_Ext_Fx_ext + m_Fg_x + m_F_VehiclB_x - m_Fd_x;
	m_Fb_y = m_Ext_Fy_ext + m_Fg_y + m_F_VehiclB_y - m_Fd_y;
	m_Fb_z = m_Ext_Fz_ext + m_Fg_z + m_F_VehiclB_z - m_Fd_z;
	m_Mb_x = m_Ext_Mx_ext + m_M_roll + m_Sus_Mx_fl + m_Sus_Mx_fr + m_Sus_Mx_rl + m_Sus_Mx_rr - m_Md_x;
	m_Mb_y = m_Ext_My_ext + m_M_pitch + m_Sus_My_fl + m_Sus_My_fr + m_Sus_My_rl + m_Sus_My_rr - m_Md_y;
	m_Mb_z = m_Ext_Mz_ext + m_M_yaw + m_Sus_Mz_fl + m_Sus_Mz_fr + m_Sus_Mz_rl + m_Sus_Mz_rr - m_Md_z;
	
}

void NMSPC::Vehicle_Body::push_drv (d_vec &derivatives) {
	m_drv_xe_x = m_ve_x_c;
	m_drv_xe_y = m_ve_y_c;
	m_drv_xe_z = m_ve_z_c;

	m_drv_vb_x = m_Fb_x/m_Mbar + m_vb_y * m_r - m_vb_z * m_q;
	m_drv_vb_y = m_Fb_y/m_Mbar + m_vb_z * m_p - m_vb_x * m_r;
	m_drv_vb_z = m_Fb_z/m_Mbar + m_vb_x * m_q - m_vb_y * m_p;
	
	m_drv_phai = m_p + (m_q * m_sin_phai + m_r * m_cos_phai) * m_sin_theta / m_cos_theta;
	m_drv_theta = m_q * m_cos_phai - m_r * m_sin_phai;
	m_drv_psi = (m_q * m_sin_phai + m_r * m_cos_phai) / m_cos_theta;

	double tmpRes1_x = m_Ibar_xx * m_p + m_Ibar_xy * m_q + m_Ibar_xz * m_r;
	double tmpRes1_y = m_Ibar_yx * m_p + m_Ibar_yy * m_q + m_Ibar_yz * m_r;
	double tmpRes1_z = m_Ibar_zx * m_p + m_Ibar_zy * m_q + m_Ibar_zz * m_r;
	double tmpRes2_x = m_Mb_x - m_q * tmpRes1_z + m_r * tmpRes1_y;
	double tmpRes2_y = m_Mb_y - m_r * tmpRes1_x + m_p * tmpRes1_z;
	double tmpRes2_z = m_Mb_z - m_p * tmpRes1_y + m_q * tmpRes1_x;
	double deno = m_Ibar_xx*m_Ibar_yy*m_Ibar_zz - m_Ibar_xx*m_Ibar_yz*m_Ibar_zy - m_Ibar_xy*m_Ibar_yx*m_Ibar_zz + m_Ibar_xy*m_Ibar_yz*m_Ibar_zx + m_Ibar_xz*m_Ibar_yx*m_Ibar_zy - m_Ibar_xz*m_Ibar_yy*m_Ibar_zx;
	double invIbar_xx = (m_Ibar_yy*m_Ibar_zz - m_Ibar_yz*m_Ibar_zy)/deno;
	double invIbar_xy =-(m_Ibar_xy*m_Ibar_zz - m_Ibar_xz*m_Ibar_zy)/deno;
	double invIbar_xz = (m_Ibar_xy*m_Ibar_yz - m_Ibar_xz*m_Ibar_yy)/deno;
	double invIbar_yx =-(m_Ibar_yx*m_Ibar_zz - m_Ibar_yz*m_Ibar_zx)/deno;
	double invIbar_yy = (m_Ibar_xx*m_Ibar_zz - m_Ibar_xz*m_Ibar_zx)/deno;
	double invIbar_yz =-(m_Ibar_xx*m_Ibar_yz - m_Ibar_xz*m_Ibar_yx)/deno;
	double invIbar_zx = (m_Ibar_yx*m_Ibar_zy - m_Ibar_yy*m_Ibar_zx)/deno;
	double invIbar_zy =-(m_Ibar_xx*m_Ibar_zy - m_Ibar_xy*m_Ibar_zx)/deno;
	double invIbar_zz = (m_Ibar_xx*m_Ibar_yy - m_Ibar_xy*m_Ibar_yx)/deno;

	m_drv_p = invIbar_xx * tmpRes2_x + invIbar_xy * tmpRes2_y + invIbar_xz * tmpRes2_z;
	m_drv_q = invIbar_yx * tmpRes2_x + invIbar_yy * tmpRes2_y + invIbar_yz * tmpRes2_z;
	m_drv_r = invIbar_zx * tmpRes2_x + invIbar_zy * tmpRes2_y + invIbar_zz * tmpRes2_z;
	
	//push outputs
	derivatives[0] = m_drv_xe_x;
	derivatives[1] = m_drv_xe_y;
	derivatives[2] = m_drv_xe_z;
	derivatives[3] = m_drv_vb_x;
	derivatives[4] = m_drv_vb_y;
	derivatives[5] = m_drv_vb_z;
	derivatives[6] = m_drv_phai;
	derivatives[7] = m_drv_theta;
	derivatives[8] = m_drv_psi;
	derivatives[9] = m_drv_p;
	derivatives[10] = m_drv_q;
	derivatives[11] = m_drv_r;
}

void NMSPC::Vehicle_Body::calculate_bar() {
	m_Mbar = m_mass + m_z1m + m_z2m + m_z3m + m_z4m + m_z5m + m_z6m + m_z7m;
	double A = (m_mass*m_a + m_z1m*m_z1R_x + m_z2m*m_z2R_x + m_z3m*m_z3R_x + \
	m_z4m*m_z4R_x + m_z5m*m_z5R_x + m_z6m*m_z6R_x + m_z7m*m_z7R_x) / m_Mbar;
	double D = (m_mass*m_d + m_z1m*m_z1R_y + m_z2m*m_z2R_y + m_z3m*m_z3R_y + \
	m_z4m*m_z4R_y + m_z5m*m_z5R_y + m_z6m*m_z6R_y + m_z7m*m_z7R_y) / m_Mbar;
	double H = (m_mass*m_h + m_z1m*m_z1R_z + m_z2m*m_z2R_z + m_z3m*m_z3R_z + \
	m_z4m*m_z4R_z + m_z5m*m_z5R_z + m_z6m*m_z6R_z + m_z7m*m_z7R_z) / m_Mbar;
	m_Rbar_x = m_a - A;
	m_Rbar_y = D - m_d;
	m_Rbar_z = m_h - H;
	m_Xbar_a = A;
	m_Xbar_b = m_a + m_b - A;
	m_Xbar_h = H;
	m_Wbar_fl = m_w_f / 2.0 + D;
	m_Wbar_fr = m_w_f / 2.0 - D;
	m_Wbar_rl = m_w_r / 2.0 + D;
	m_Wbar_rr = m_w_r / 2.0 - D;
	m_HPbar_fl_x = A;
	m_HPbar_fr_x = A;
	m_HPbar_rl_x = A - (m_a + m_b);
	m_HPbar_rr_x = A - (m_a + m_b);
	m_HPbar_fl_y = -m_Wbar_fl;
	m_HPbar_fr_y = m_Wbar_fr;
	m_HPbar_rl_y = -m_Wbar_rl;
	m_HPbar_rr_y = m_Wbar_rr;
	m_HPbar_fl_z = H;
	m_HPbar_fr_z = H;
	m_HPbar_rl_z = H;
	m_HPbar_rr_z = H;
	double lx1 = A-m_z1R_x;
	double lx2 = A-m_z2R_x;
	double lx3 = A-m_z3R_x;
	double lx4 = A-m_z4R_x;
	double lx5 = A-m_z5R_x;
	double lx6 = A-m_z6R_x;
	double lx7 = A-m_z7R_x;
	double ly1 = D-m_z1R_y;
	double ly2 = D-m_z2R_y;
	double ly3 = D-m_z3R_y;
	double ly4 = D-m_z4R_y;
	double ly5 = D-m_z5R_y;
	double ly6 = D-m_z6R_y;
	double ly7 = D-m_z7R_y;
	double lz1 = H-m_z1R_z;
	double lz2 = H-m_z2R_z;
	double lz3 = H-m_z3R_z;
	double lz4 = H-m_z4R_z;
	double lz5 = H-m_z5R_z;
	double lz6 = H-m_z6R_z;
	double lz7 = H-m_z7R_z;
	//--
	m_Ibar_xx = m_mass * (pow(m_d - D, 2.0) + pow(m_h - H, 2.0)) + \
	m_z1m * (pow(ly1, 2.0)+pow(lz1, 2.0)) + \
	m_z2m * (pow(ly2, 2.0)+pow(lz2, 2.0))+\
	m_z3m * (pow(ly3, 2.0)+pow(lz3, 2.0))+\
	m_z4m * (pow(ly4, 2.0)+pow(lz4, 2.0))+\
	m_z5m * (pow(ly5, 2.0)+pow(lz5, 2.0))+\
	m_z6m * (pow(ly6, 2.0)+pow(lz6, 2.0))+\
	m_z7m * (pow(ly7, 2.0)+pow(lz7, 2.0))+\
	m_Ixx + m_z1I_xx + m_z2I_xx + m_z3I_xx +\
	m_z4I_xx + m_z5I_xx + m_z6I_xx + m_z7I_xx;
	//--
	m_Ibar_xy = -(m_mass * (m_a - A) * (m_d - D) + \
	m_z1m * lx1 * ly1 + m_z2m * lx2 * ly2 + m_z3m * lx3 * ly3 + \
	m_z4m * lx4 * ly4 + m_z5m * lx5 * ly5 + m_z6m * lx6 * ly6 + \
	m_z7m * lx7 * ly7) + \
	m_Ixy + m_z1I_xy + m_z2I_xy + m_z3I_xy + \
	m_z4I_xy + m_z5I_xy + m_z6I_xy + m_z7I_xy;
	//--
	m_Ibar_xz = m_mass * (m_a - A) * (m_h - H)+ \
	m_z1m * lx1 * lz1 + m_z2m * lx2 * lz2 + m_z3m * lx3 * lz3 + \
	m_z4m * lx4 * lz4 + m_z5m * lx5 * lz5 + m_z6m * lx6 * lz6 + \
	m_z7m * lx7 * lz7 + \
	m_Ixz + m_z1I_xz + m_z2I_xz + m_z3I_xz + \
	m_z4I_xz + m_z5I_xz + m_z6I_xz + m_z7I_xz;
	//--
	m_Ibar_yx = m_Ibar_xy;
	//--
	m_Ibar_yy = m_mass * (pow(m_a-A, 2.0) + pow(m_h-H, 2.0)) + \
	m_z1m * (pow(lx1, 2.0) + pow(lz1, 2.0)) + \
	m_z2m * (pow(lx2, 2.0) + pow(lz2, 2.0)) + \
	m_z3m * (pow(lx3, 2.0) + pow(lz3, 2.0)) + \
	m_z4m * (pow(lx4, 2.0) + pow(lz4, 2.0)) + \
	m_z5m * (pow(lx5, 2.0) + pow(lz5, 2.0)) + \
	m_z6m * (pow(lx6, 2.0) + pow(lz6, 2.0)) + \
	m_z7m * (pow(lx7, 2.0) + pow(lz7, 2.0)) + \
	m_Iyy + m_z1I_yy + m_z2I_yy + m_z3I_yy + \
	m_z4I_yy + m_z5I_yy + m_z6I_yy + m_z7I_yy;
	//--
	m_Ibar_yz = -(m_mass * (m_d - D) * (m_h - H) + \
	m_z1m * ly1 * lz1 + m_z2m * ly2 * lz2 + m_z3m * ly3 * lz3 + \
	m_z4m * ly4 * lz4 + m_z5m * ly5 * lz5 + m_z6m * ly6 * lz6 + \
	m_z7m * ly7 * lz7) + \
	m_Iyz + m_z1I_yz + m_z2I_yz + m_z3I_yz + \
	m_z4I_yz + m_z5I_yz + m_z6I_yz + m_z7I_yz;
	//--
	m_Ibar_zx = m_Ibar_xz;
	//--
	m_Ibar_zy = m_Ibar_yz;
	//--
	m_Ibar_zz = m_mass * (pow(m_d - D, 2.0) + pow(m_a - A, 2.0)) + \
	m_z1m * (pow(ly1, 2.0) + pow(lx1, 2.0)) + \
	m_z2m * (pow(ly2, 2.0) + pow(lx2, 2.0)) + \
	m_z3m * (pow(ly3, 2.0) + pow(lx3, 2.0)) + \
	m_z4m * (pow(ly4, 2.0) + pow(lx4, 2.0)) + \
	m_z5m * (pow(ly5, 2.0) + pow(lx5, 2.0)) + \
	m_z6m * (pow(ly6, 2.0) + pow(lx6, 2.0)) + \
	m_z7m * (pow(ly7, 2.0) + pow(lx7, 2.0)) + \
	m_Izz + m_z1I_zz + m_z2I_zz + m_z3I_zz + \
	m_z4I_zz + m_z5I_zz + m_z6I_zz + m_z7I_zz;

}

void NMSPC::Vehicle_Body::calculate_gravity() {
	m_Fg_x = m_DCM_02 * m_Mbar * g;
	m_Fg_y = m_DCM_12 * m_Mbar * g;
	m_Fg_z = m_DCM_22 * m_Mbar * g;
}

void NMSPC::Vehicle_Body::calculate_aero_drag() {
	double vdb_x = m_vb_x - (m_DCM_00 * m_Air_Wx + m_DCM_01 * m_Air_Wy + m_DCM_02 * m_Air_Wz);
	double vdb_y = m_vb_y - (m_DCM_10 * m_Air_Wx + m_DCM_11 * m_Air_Wy + m_DCM_12 * m_Air_Wz);
	double vdb_z = m_vb_z - (m_DCM_20 * m_Air_Wx + m_DCM_21 * m_Air_Wy + m_DCM_22 * m_Air_Wz);
	double sum = pow(vdb_x,2) + pow(vdb_y,2) + pow(vdb_z,2);
	double wdir_x = tanh(4.0 * vdb_x);
	double wdir_y = tanh(4.0 * vdb_y);
	double wdir_z = tanh(4.0 * vdb_z);
	double gain = sum *0.5 * m_Af * m_Pabs * m_Cg / div0protect(m_Air_Tair,1e-5);
	double ata2 = atan2(vdb_y,vdb_x);

	m_Fd_x = m_Cd * gain * wdir_x;
	m_Fd_y = m_pwl_Cs(ata2) * gain * wdir_y;
	m_Fd_z = m_Cl * gain * wdir_z;
	m_Md_x = 0.0;
	m_Md_y = wdir_x * m_Cpm * gain * (m_a + m_b);
	m_Md_z = m_pwl_Cym(ata2) * gain * (m_a + m_b);
}
