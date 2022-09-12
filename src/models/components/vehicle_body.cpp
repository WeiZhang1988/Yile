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

NMSPC::Vehicle_Body::Vehicle_Body(const d_vec &params_d, const d_v_vec &params_v, const d_vec &init_states) {
	//parameters
	//chassis
	m_mass 		= params_d[0];
	m_a 		= params_d[1];
	m_b 		= params_d[2];
	m_d 		= params_d[3];
	m_h 		= params_d[4];
	m_Ixx 		= params_d[5];
	m_Ixy 		= params_d[6];
	m_Ixz 		= params_d[7];
	m_Iyx 		= params_d[8];
	m_Iyy 		= params_d[9];
	m_Iyz 		= params_d[10];
	m_Izx 		= params_d[11];
	m_Izy 		= params_d[12];
	m_Izz 		= params_d[13];
	m_w_f	= params_d[14];
	m_w_r	= params_d[15];
	//inertial loads
	//z1
	m_z1m 		= params_d[16];
	m_z1R_x 	= params_d[17];
	m_z1R_y 	= params_d[18];
	m_z1R_z		= params_d[19];
	m_z1I_xx 	= params_d[20];
	m_z1I_xy 	= params_d[21];
	m_z1I_xz 	= params_d[22];
	m_z1I_yx 	= params_d[23];
	m_z1I_yy 	= params_d[24];
	m_z1I_yz 	= params_d[25];
	m_z1I_zx 	= params_d[26];
	m_z1I_zy 	= params_d[27];
	m_z1I_zz 	= params_d[28];
	//z2
	m_z2m 		= params_d[29];
	m_z2R_x 	= params_d[30];
	m_z2R_y 	= params_d[31];
	m_z2R_z		= params_d[32];
	m_z2I_xx 	= params_d[33];
	m_z2I_xy 	= params_d[34];
	m_z2I_xz 	= params_d[35];
	m_z2I_yx 	= params_d[36];
	m_z2I_yy 	= params_d[37];
	m_z2I_yz 	= params_d[38];
	m_z2I_zx 	= params_d[39];
	m_z2I_zy 	= params_d[40];
	m_z2I_zz 	= params_d[41];
	//z3
	m_z3m 		= params_d[42];
	m_z3R_x 	= params_d[43];
	m_z3R_y 	= params_d[44];
	m_z3R_z		= params_d[45];
	m_z3I_xx 	= params_d[46];
	m_z3I_xy 	= params_d[47];
	m_z3I_xz 	= params_d[48];
	m_z3I_yx 	= params_d[49];
	m_z3I_yy 	= params_d[50];
	m_z3I_yz 	= params_d[51];
	m_z3I_zx 	= params_d[52];
	m_z3I_zy 	= params_d[53];
	m_z3I_zz 	= params_d[54];
	//z4
	m_z4m 		= params_d[55];
	m_z4R_x 	= params_d[56];
	m_z4R_y 	= params_d[57];
	m_z4R_z		= params_d[58];
	m_z4I_xx 	= params_d[59];
	m_z4I_xy 	= params_d[60];
	m_z4I_xz 	= params_d[61];
	m_z4I_yx 	= params_d[62];
	m_z4I_yy 	= params_d[63];
	m_z4I_yz 	= params_d[64];
	m_z4I_zx 	= params_d[65];
	m_z4I_zy 	= params_d[66];
	m_z4I_zz 	= params_d[67];
	//z5
	m_z5m 		= params_d[68];
	m_z5R_x 	= params_d[69];
	m_z5R_y 	= params_d[70];
	m_z5R_z		= params_d[71];
	m_z5I_xx 	= params_d[72];
	m_z5I_xy 	= params_d[73];
	m_z5I_xz 	= params_d[74];
	m_z5I_yx 	= params_d[75];
	m_z5I_yy 	= params_d[76];
	m_z5I_yz 	= params_d[77];
	m_z5I_zx 	= params_d[78];
	m_z5I_zy 	= params_d[79];
	m_z5I_zz 	= params_d[80];
	//z6
	m_z6m 		= params_d[81];
	m_z6R_x 	= params_d[82];
	m_z6R_y 	= params_d[83];
	m_z6R_z		= params_d[84];
	m_z6I_xx 	= params_d[85];
	m_z6I_xy 	= params_d[86];
	m_z6I_xz 	= params_d[87];
	m_z6I_yx 	= params_d[88];
	m_z6I_yy 	= params_d[89];
	m_z6I_yz 	= params_d[90];
	m_z6I_zx 	= params_d[91];
	m_z6I_zy 	= params_d[92];
	m_z6I_zz 	= params_d[93];
	//z7
	m_z7m 		= params_d[94];
	m_z7R_x 	= params_d[95];
	m_z7R_y 	= params_d[96];
	m_z7R_z		= params_d[97];
	m_z7I_xx 	= params_d[98];
	m_z7I_xy 	= params_d[99];
	m_z7I_xz 	= params_d[100];
	m_z7I_yx 	= params_d[101];
	m_z7I_yy 	= params_d[102];
	m_z7I_yz 	= params_d[103];
	m_z7I_zx 	= params_d[104];
	m_z7I_zy 	= params_d[105];
	m_z7I_zz 	= params_d[106];
	//aerodynamics
	m_Pabs	 = params_d[107];
	m_Cg	 = params_d[108];
	m_Af  	 = params_d[109];
	m_Cd  	 = params_d[110]; 
	m_Cl  	 = params_d[111]; 
	m_Cpm 	 = params_d[112]; 
	m_beta_w = params_v[0];
	m_Cs 	 = params_v[1];
	m_Cym 	 = params_v[2];
	//simulation
	m_xdot_tol = params_d[113]; 
	m_longOff  = params_d[114]; 
	m_latOff   = params_d[115]; 
	m_vertOff  = params_d[116];
	//initialize states
	m_xe_x	= init_states[0];
	m_xe_y	= init_states[1];
	m_xe_z	= init_states[2];
	m_vb_x	= init_states[3];
	m_vb_y	= init_states[4];
	m_vb_z	= init_states[5];
	m_phai	= init_states[6];
	m_theta	= init_states[7];
	m_psi	= init_states[8];
	m_p		= init_states[9];
	m_q		= init_states[10];
	m_r		= init_states[11];
	//caculate bar
	calculate_bar();
	//caculate table;
	m_pwl_Cs = piece_wise_linear(m_beta_w,m_Cs);
	m_pwl_Cym = piece_wise_linear(m_beta_w,m_Cym);
}

NMSPC::Vehicle_Body::Vehicle_Body (const std::string &filename) {
	boost::property_tree::ptree tree;
    if (std::filesystem::exists(filename)){
        boost::property_tree::read_json(filename,tree);
        if ("vehicle-body" == tree.get<std::string>("type")) {
        	//parameters
			//chassis
			m_mass 		= tree.get<double>("mass");
			m_a 		= tree.get<double>("a");
			m_b 		= tree.get<double>("b");
			m_d 		= tree.get<double>("d");
			m_h 		= tree.get<double>("h");
			m_Ixx 		= tree.get<double>("Ixx");
			m_Ixy 		= tree.get<double>("Ixy");
			m_Ixz 		= tree.get<double>("Ixz");
			m_Iyx 		= tree.get<double>("Iyx");
			m_Iyy 		= tree.get<double>("Iyy");
			m_Iyz 		= tree.get<double>("Iyz");
			m_Izx 		= tree.get<double>("Izx");
			m_Izy 		= tree.get<double>("Izy");
			m_Izz 		= tree.get<double>("Izz");
			m_w_f		= tree.get<double>("w_f");
			m_w_r		= tree.get<double>("w_r");
			//inertial loads
			//z1
			m_z1m 		= tree.get<double>("z1m");
			m_z1R_x 	= tree.get<double>("z1R_x");
			m_z1R_y 	= tree.get<double>("z1R_y");
			m_z1R_z		= tree.get<double>("z1R_z");
			m_z1I_xx 	= tree.get<double>("z1I_xx");
			m_z1I_xy 	= tree.get<double>("z1I_xy");
			m_z1I_xz 	= tree.get<double>("z1I_xz");
			m_z1I_yx 	= tree.get<double>("z1I_yx");
			m_z1I_yy 	= tree.get<double>("z1I_yy");
			m_z1I_yz 	= tree.get<double>("z1I_yz");
			m_z1I_zx 	= tree.get<double>("z1I_zx");
			m_z1I_zy 	= tree.get<double>("z1I_zy");
			m_z1I_zz 	= tree.get<double>("z1I_zz");
			//z2
			m_z2m 		= tree.get<double>("z2m");
			m_z2R_x 	= tree.get<double>("z2R_x");
			m_z2R_y 	= tree.get<double>("z2R_y");
			m_z2R_z		= tree.get<double>("z2R_z");
			m_z2I_xx 	= tree.get<double>("z2I_xx");
			m_z2I_xy 	= tree.get<double>("z2I_xy");
			m_z2I_xz 	= tree.get<double>("z2I_xz");
			m_z2I_yx 	= tree.get<double>("z2I_yx");
			m_z2I_yy 	= tree.get<double>("z2I_yy");
			m_z2I_yz 	= tree.get<double>("z2I_yz");
			m_z2I_zx 	= tree.get<double>("z2I_zx");
			m_z2I_zy 	= tree.get<double>("z2I_zy");
			m_z2I_zz 	= tree.get<double>("z2I_zz");
			//z3
			m_z3m 		= tree.get<double>("z3m");
			m_z3R_x 	= tree.get<double>("z3R_x");
			m_z3R_y 	= tree.get<double>("z3R_y");
			m_z3R_z		= tree.get<double>("z3R_z");
			m_z3I_xx 	= tree.get<double>("z3I_xx");
			m_z3I_xy 	= tree.get<double>("z3I_xy");
			m_z3I_xz 	= tree.get<double>("z3I_xz");
			m_z3I_yx 	= tree.get<double>("z3I_yx");
			m_z3I_yy 	= tree.get<double>("z3I_yy");
			m_z3I_yz 	= tree.get<double>("z3I_yz");
			m_z3I_zx 	= tree.get<double>("z3I_zx");
			m_z3I_zy 	= tree.get<double>("z3I_zy");
			m_z3I_zz 	= tree.get<double>("z3I_zz");
			//z4
			m_z4m 		= tree.get<double>("z4m");
			m_z4R_x 	= tree.get<double>("z4R_x");
			m_z4R_y 	= tree.get<double>("z4R_y");
			m_z4R_z		= tree.get<double>("z4R_z");
			m_z4I_xx 	= tree.get<double>("z4I_xx");
			m_z4I_xy 	= tree.get<double>("z4I_xy");
			m_z4I_xz 	= tree.get<double>("z4I_xz");
			m_z4I_yx 	= tree.get<double>("z4I_yx");
			m_z4I_yy 	= tree.get<double>("z4I_yy");
			m_z4I_yz 	= tree.get<double>("z4I_yz");
			m_z4I_zx 	= tree.get<double>("z4I_zx");
			m_z4I_zy 	= tree.get<double>("z4I_zy");
			m_z4I_zz 	= tree.get<double>("z4I_zz");
			//z5
			m_z5m 		= tree.get<double>("z5m");
			m_z5R_x 	= tree.get<double>("z5R_x");
			m_z5R_y 	= tree.get<double>("z5R_y");
			m_z5R_z		= tree.get<double>("z5R_z");
			m_z5I_xx 	= tree.get<double>("z5I_xx");
			m_z5I_xy 	= tree.get<double>("z5I_xy");
			m_z5I_xz 	= tree.get<double>("z5I_xz");
			m_z5I_yx 	= tree.get<double>("z5I_yx");
			m_z5I_yy 	= tree.get<double>("z5I_yy");
			m_z5I_yz 	= tree.get<double>("z5I_yz");
			m_z5I_zx 	= tree.get<double>("z5I_zx");
			m_z5I_zy 	= tree.get<double>("z5I_zy");
			m_z5I_zz 	= tree.get<double>("z5I_zz");
			//z6
			m_z6m 		= tree.get<double>("z6m");
			m_z6R_x 	= tree.get<double>("z6R_x");
			m_z6R_y 	= tree.get<double>("z6R_y");
			m_z6R_z		= tree.get<double>("z6R_z");
			m_z6I_xx 	= tree.get<double>("z6I_xx");
			m_z6I_xy 	= tree.get<double>("z6I_xy");
			m_z6I_xz 	= tree.get<double>("z6I_xz");
			m_z6I_yx 	= tree.get<double>("z6I_yx");
			m_z6I_yy 	= tree.get<double>("z6I_yy");
			m_z6I_yz 	= tree.get<double>("z6I_yz");
			m_z6I_zx 	= tree.get<double>("z6I_zx");
			m_z6I_zy 	= tree.get<double>("z6I_zy");
			m_z6I_zz 	= tree.get<double>("z6I_zz");
			//z7
			m_z7m 		= tree.get<double>("z7m");
			m_z7R_x 	= tree.get<double>("z7R_x");
			m_z7R_y 	= tree.get<double>("z7R_y");
			m_z7R_z		= tree.get<double>("z7R_z");
			m_z7I_xx 	= tree.get<double>("z7I_xx");
			m_z7I_xy 	= tree.get<double>("z7I_xy");
			m_z7I_xz 	= tree.get<double>("z7I_xz");
			m_z7I_yx 	= tree.get<double>("z7I_yx");
			m_z7I_yy 	= tree.get<double>("z7I_yy");
			m_z7I_yz 	= tree.get<double>("z7I_yz");
			m_z7I_zx 	= tree.get<double>("z7I_zx");
			m_z7I_zy 	= tree.get<double>("z7I_zy");
			m_z7I_zz 	= tree.get<double>("z7I_zz");
			//aerodynamics
			m_Pabs	 = tree.get<double>("Pabs");
			m_Cg	 = tree.get<double>("Cg");
			m_Af  	 = tree.get<double>("Af");
			m_Cd  	 = tree.get<double>("Cd");
			m_Cl  	 = tree.get<double>("Cl");
			m_Cpm 	 = tree.get<double>("Cpm");
			for (auto item : tree.get_child("beta_w")) {
            	m_beta_w.push_back(item.second.get<double>(""));
        	}
			for (auto item : tree.get_child("Cs")) {
            	m_Cs.push_back(item.second.get<double>(""));
        	}
			for (auto item : tree.get_child("Cym")) {
            	m_Cym.push_back(item.second.get<double>(""));
        	}
			//simulation
			m_xdot_tol = tree.get<double>("xdot_tol");
			m_longOff  = tree.get<double>("longOff"); 
			m_latOff   = tree.get<double>("latOff"); 
			m_vertOff  = tree.get<double>("vertOff");
			//initialize states
			m_xe_x	= tree.get<double>("init_xe_x");
			m_xe_y	= tree.get<double>("init_xe_y");
			m_xe_z	= tree.get<double>("init_xe_z");
			m_vb_x	= tree.get<double>("init_vb_x");
			m_vb_y	= tree.get<double>("init_vb_y");
			m_vb_z	= tree.get<double>("init_vb_z");
			m_phai	= tree.get<double>("init_phai");
			m_theta	= tree.get<double>("init_theta");
			m_psi	= tree.get<double>("init_psi");
			m_p		= tree.get<double>("init_p");
			m_q		= tree.get<double>("init_q");
			m_r		= tree.get<double>("init_r");
			//caculate bar
			calculate_bar();
			//caculate table;
			m_pwl_Cs = piece_wise_linear(m_beta_w,m_Cs);
			m_pwl_Cym = piece_wise_linear(m_beta_w,m_Cym);
        } else {
            Vehicle_Body();
        }
    } else {
        Vehicle_Body();
    }
}

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

void NMSPC::Vehicle_Body::update_pv(const d_vec &inputs, d_vec &outputs) { 
	//pull inputs: 
	//no inputs are required here
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

	//push outputs
	outputs[0] = m_xe_x_c;
	outputs[1] = m_xe_y_c;
	outputs[2] = m_xe_z_c;
	outputs[3] = m_ve_x_c;
	outputs[4] = m_ve_y_c;
	outputs[5] = m_ve_z_c;
	outputs[6] = m_vb_x_c;
	outputs[7] = m_vb_y_c;
	outputs[8] = m_vb_z_c;
	outputs[9] = m_phai_c;
	outputs[10] = m_theta_c;
	outputs[11] = m_psi_c;
	outputs[12] = m_p_c;
	outputs[13] = m_q_c;
	outputs[14] = m_r_c;
	outputs[15] = m_beta_c;
	outputs[16] = m_xe_x_o;
	outputs[17] = m_xe_y_o;
	outputs[18] = m_xe_z_o;
	outputs[19] = m_ve_x_o;
	outputs[20] = m_ve_y_o;
	outputs[21] = m_ve_z_o;
	outputs[22] = m_xb_x_o;
	outputs[23] = m_xb_y_o;
	outputs[24] = m_xb_z_o;
	outputs[25] = m_vb_x_o;
	outputs[26] = m_vb_y_o;
	outputs[27] = m_vb_z_o;
	outputs[28] = m_beta_o;
	outputs[29] = m_xe_x_fl;
	outputs[30] = m_xe_y_fl;
	outputs[31] = m_xe_z_fl;
	outputs[32] = m_ve_x_fl;
	outputs[33] = m_ve_y_fl;
	outputs[34] = m_ve_z_fl;
	outputs[35] = m_xb_x_fl;
	outputs[36] = m_xb_y_fl;
	outputs[37] = m_xb_z_fl;
	outputs[38] = m_vb_x_fl;
	outputs[39] = m_vb_y_fl;
	outputs[40] = m_vb_z_fl;
	outputs[41] = m_xe_x_fr;
	outputs[42] = m_xe_y_fr;
	outputs[43] = m_xe_z_fr;
	outputs[44] = m_ve_x_fr;
	outputs[45] = m_ve_y_fr;
	outputs[46] = m_ve_z_fr;
	outputs[47] = m_xb_x_fr;
	outputs[48] = m_xb_y_fr;
	outputs[49] = m_xb_z_fr;
	outputs[50] = m_vb_x_fr;
	outputs[51] = m_vb_y_fr;
	outputs[52] = m_vb_z_fr;
	outputs[53] = m_xe_x_rl;
	outputs[54] = m_xe_y_rl;
	outputs[55] = m_xe_z_rl;
	outputs[56] = m_ve_x_rl;
	outputs[57] = m_ve_y_rl;
	outputs[58] = m_ve_z_rl;
	outputs[59] = m_xb_x_rl;
	outputs[60] = m_xb_y_rl;
	outputs[61] = m_xb_z_rl;
	outputs[62] = m_vb_x_rl;
	outputs[63] = m_vb_y_rl;
	outputs[64] = m_vb_z_rl;
	outputs[65] = m_xe_x_rr;
	outputs[66] = m_xe_y_rr;
	outputs[67] = m_xe_z_rr;
	outputs[68] = m_ve_x_rr;
	outputs[69] = m_ve_y_rr;
	outputs[70] = m_ve_z_rr;
	outputs[71] = m_xb_x_rr;
	outputs[72] = m_xb_y_rr;
	outputs[73] = m_xb_z_rr;
	outputs[74] = m_vb_x_rr;
	outputs[75] = m_vb_y_rr;
	outputs[76] = m_vb_z_rr;
	
}

void NMSPC::Vehicle_Body::update_fm (const d_vec &inputs, d_vec &outputs) {
	//pull inputs
	m_Air_Wx = inputs[0];
	m_Air_Wy = inputs[1];
	m_Air_Wz = inputs[2];
	m_Air_Tair = inputs[3];
	m_Sus_Fx_fl = inputs[4];
	m_Sus_Fx_fr = inputs[5];
	m_Sus_Fx_rl = inputs[6];
	m_Sus_Fx_rr = inputs[7];
	m_Sus_Fy_fl = inputs[8];
	m_Sus_Fy_fr = inputs[9];
	m_Sus_Fy_rl = inputs[10];
	m_Sus_Fy_rr = inputs[11];
	m_Sus_Fz_fl = inputs[12];
	m_Sus_Fz_fr = inputs[13];
	m_Sus_Fz_rl = inputs[14];
	m_Sus_Fz_rr = inputs[15];
	m_Sus_Mx_fl = inputs[16];
	m_Sus_Mx_fr = inputs[17];
	m_Sus_Mx_rl = inputs[18];
	m_Sus_Mx_rr = inputs[19];
	m_Sus_My_fl = inputs[20];
	m_Sus_My_fr = inputs[21];
	m_Sus_My_rl = inputs[22];
	m_Sus_My_rr = inputs[23];
	m_Sus_Mz_fl = inputs[24];
	m_Sus_Mz_fr = inputs[25];
	m_Sus_Mz_rl = inputs[26];
	m_Sus_Mz_rr = inputs[27];
	m_Ext_Fx_ext = inputs[28]; 
	m_Ext_Fy_ext = inputs[29]; 
	m_Ext_Fz_ext = inputs[30];
	m_Ext_Mx_ext = inputs[31];
	m_Ext_My_ext = inputs[32];
	m_Ext_Mz_ext = inputs[33];
	
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

void NMSPC::Vehicle_Body::update_drv (d_vec &outputs) {
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
	outputs[0] = m_drv_xe_x;
	outputs[1] = m_drv_xe_y;
	outputs[2] = m_drv_xe_z;
	outputs[3] = m_drv_vb_x;
	outputs[4] = m_drv_vb_y;
	outputs[5] = m_drv_vb_z;
	outputs[6] = m_drv_phai;
	outputs[7] = m_drv_theta;
	outputs[8] = m_drv_psi;
	outputs[9] = m_drv_p;
	outputs[10] = m_drv_q;
	outputs[11] = m_drv_r;
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
