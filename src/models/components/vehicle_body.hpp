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
#ifndef VEHICLE_BODY_HPP
#define VEHICLE_BODY_HPP
#include "common.hpp"

namespace NMSPC{
class Vehicle_Body {
public:
	static const int m_params_num = 120;										//amount of parameters
	static const int m_pv_inputs_num = 0;									//amount of pv inputs
	static const int m_fm_inputs_num = 34;									//amount of fm inputs
	static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;		//amount of inputs
	static const int m_con_states_num = 12;									//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;					//amount of derivatives;
	static const int m_dis_states_num = 0;									//amount of discrete states;
	static const int m_pv_outputs_num = 78;									//amount of pv outputs
	static const int m_fm_outputs_num = 0;									//amount of fm outputs
	static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;	//amount of outputs

	Vehicle_Body (\
	//chassis
	double mass=1181.0, \
	double a=1.515, double b=1.504, double d=0.0, double h=0.134, \
	double Ixx=1.0, double Ixy=0.0, double Ixz=0.0, \
	double Iyx=0.0, double Iyy=1.0, double Iyz=0.0, \
	double Izx=0.0, double Izy=0.0, double Izz=1.0, \
	double w_f=1.0, double w_r=1.0, \
	//z1
	double z1m=0.0, double z1R_x=0.0, double z1R_y=0.0, \
	double z1R_z = 0.0, double z1I_xx=0.0, double z1I_xy=0.0, \
	double z1I_xz=0.0, double z1I_yx=0.0, double z1I_yy=0.0, \
	double z1I_yz=0.0, double z1I_zx=0.0, double z1I_zy=0.0, \
	double z1I_zz=0.0, \
	//z2
	double z2m=0.0, double z2R_x=0.0, double z2R_y=0.0, \
	double z2R_z = 0.0, double z2I_xx=0.0, double z2I_xy=0.0, \
	double z2I_xz=0.0, double z2I_yx=0.0, double z2I_yy=0.0, \
	double z2I_yz=0.0, double z2I_zx=0.0, double z2I_zy=0.0, \
	double z2I_zz=0.0, \
	//z3
	double z3m=0.0, double z3R_x=0.0, double z3R_y=0.0, \
	double z3R_z = 0.0, double z3I_xx=0.0, double z3I_xy=0.0, \
	double z3I_xz=0.0, double z3I_yx=0.0, double z3I_yy=0.0, \
	double z3I_yz=0.0, double z3I_zx=0.0, double z3I_zy=0.0, \
	double z3I_zz=0.0, \
	//z4
	double z4m=0.0, double z4R_x=0.0, double z4R_y=0.0, \
	double z4R_z = 0.0, double z4I_xx=0.0, double z4I_xy=0.0, \
	double z4I_xz=0.0, double z4I_yx=0.0, double z4I_yy=0.0, \
	double z4I_yz=0.0, double z4I_zx=0.0, double z4I_zy=0.0, \
	double z4I_zz=0.0, \
	//z5
	double z5m=0.0, double z5R_x=0.0, double z5R_y=0.0, \
	double z5R_z = 0.0, double z5I_xx=0.0, double z5I_xy=0.0, \
	double z5I_xz=0.0, double z5I_yx=0.0, double z5I_yy=0.0, \
	double z5I_yz=0.0, double z5I_zx=0.0, double z5I_zy=0.0, \
	double z5I_zz=0.0, \
	//z6
	double z6m=0.0, double z6R_x=0.0, double z6R_y=0.0, \
	double z6R_z = 0.0, double z6I_xx=0.0, double z6I_xy=0.0, \
	double z6I_xz=0.0, double z6I_yx=0.0, double z6I_yy=0.0, \
	double z6I_yz=0.0, double z6I_zx=0.0, double z6I_zy=0.0, \
	double z6I_zz=0.0, \
	//z7
	double z7m=0.0, double z7R_x=0.0, double z7R_y=0.0, \
	double z7R_z = 0.0, double z7I_xx=0.0, double z7I_xy=0.0, \
	double z7I_xz=0.0, double z7I_yx=0.0, double z7I_yy=0.0, \
	double z7I_yz=0.0, double z7I_zx=0.0, double z7I_zy=0.0, \
	double z7I_zz=0.0, \
	//aerodynamics
	double Pabs=101325.0, double Cg=287.058, \
	double Af=1.0, double Cd=1.0, double Cl=1.0, double Cpm=1.0, \
	d_vec beta_w={0.0,0.01,0.02,0.03}, \
	d_vec Cs	={0.0,0.03,0.06,0.09}, \
	d_vec Cym	={0.0,0.01,0.02,0.03}, \
	//simulation
	double xdot_tol=0.1, double longOff=0.1, double latOff=0.1, \
	double vertOff=0.1, \
	//init states
	double init_xe_x=0.0, double init_xe_y=0.0, double init_xe_z=0.0, \
	double init_vb_x=0.0, double init_vb_y=0.0, double init_vb_z=0.0, \
	double init_phai=0.0, double init_theta=0.0, double init_psi=0.0, \
	double init_p=0.0, double init_q=0.0, double init_r=0.0) :
	//chassis
	m_mass(mass), m_a(a), m_b(b), m_d(d), m_h(h), m_Ixx(Ixx), m_Ixy(Ixy), m_Ixz(Ixz), \
	m_Iyx(Iyx), m_Iyy(Iyy), m_Iyz(Iyz), m_Izx(Izx), m_Izy(Izy), m_Izz(Izz), \
	m_w_f(w_f), m_w_r(w_r), \
	//z1
	m_z1m(z1m), m_z1R_x(z1R_x), m_z1R_y(z1R_y), m_z1R_z(z1R_z), m_z1I_xx(z1I_xx), \
	m_z1I_xy(z1I_xy), m_z1I_xz(z1I_xz), m_z1I_yx(z1I_yx), m_z1I_yy(z1I_yy), m_z1I_yz(z1I_yz), \
	m_z1I_zx(z1I_zx), m_z1I_zy(z1I_zy), m_z1I_zz(z1I_zz), \
	//z2
	m_z2m(z2m), m_z2R_x(z2R_x), m_z2R_y(z2R_y), m_z2R_z(z2R_z), m_z2I_xx(z2I_xx), \
	m_z2I_xy(z2I_xy), m_z2I_xz(z2I_xz), m_z2I_yx(z2I_yx), m_z2I_yy(z2I_yy), m_z2I_yz(z2I_yz), \
	m_z2I_zx(z2I_zx), m_z2I_zy(z2I_zy), m_z2I_zz(z2I_zz), \
	//z3
	m_z3m(z3m), m_z3R_x(z3R_x), m_z3R_y(z3R_y), m_z3R_z(z3R_z), m_z3I_xx(z3I_xx), \
	m_z3I_xy(z3I_xy), m_z3I_xz(z3I_xz), m_z3I_yx(z3I_yx), m_z3I_yy(z3I_yy), m_z3I_yz(z3I_yz), \
	m_z3I_zx(z3I_zx), m_z3I_zy(z3I_zy), m_z3I_zz(z3I_zz), \
	//z4
	m_z4m(z4m), m_z4R_x(z4R_x), m_z4R_y(z4R_y), m_z4R_z(z4R_z), m_z4I_xx(z4I_xx), \
	m_z4I_xy(z4I_xy), m_z4I_xz(z4I_xz), m_z4I_yx(z4I_yx), m_z4I_yy(z4I_yy), m_z4I_yz(z4I_yz), \
	m_z4I_zx(z4I_zx), m_z4I_zy(z4I_zy), m_z4I_zz(z4I_zz), \
	//z5
	m_z5m(z5m), m_z5R_x(z5R_x), m_z5R_y(z5R_y), m_z5R_z(z5R_z), m_z5I_xx(z5I_xx), \
	m_z5I_xy(z5I_xy), m_z5I_xz(z5I_xz), m_z5I_yx(z5I_yx), m_z5I_yy(z5I_yy), m_z5I_yz(z5I_yz), \
	m_z5I_zx(z5I_zx), m_z5I_zy(z5I_zy), m_z5I_zz(z5I_zz), \
	//z6
	m_z6m(z6m), m_z6R_x(z6R_x), m_z6R_y(z6R_y), m_z6R_z(z6R_z), m_z6I_xx(z6I_xx), \
	m_z6I_xy(z6I_xy), m_z6I_xz(z6I_xz), m_z6I_yx(z6I_yx), m_z6I_yy(z6I_yy), m_z6I_yz(z6I_yz), \
	m_z6I_zx(z6I_zx), m_z6I_zy(z6I_zy), m_z6I_zz(z6I_zz), \
	//z7
	m_z7m(z7m), m_z7R_x(z7R_x), m_z7R_y(z7R_y), m_z7R_z(z7R_z), m_z7I_xx(z7I_xx), \
	m_z7I_xy(z7I_xy), m_z7I_xz(z7I_xz), m_z7I_yx(z7I_yx), m_z7I_yy(z7I_yy), m_z7I_yz(z7I_yz), \
	m_z7I_zx(z7I_zx), m_z7I_zy(z7I_zy), m_z7I_zz(z7I_zz), \
	//aerodynamics
	m_Pabs(Pabs), m_Cg(Cg), m_Af(Af), m_Cd(Cd), m_Cl(Cl), m_Cpm(Cpm), m_beta_w(beta_w), m_Cs(Cs), m_Cym(Cym), \
	//simulation
	m_xdot_tol(xdot_tol), m_longOff(longOff), m_latOff(latOff), m_vertOff(vertOff), \
	//init states
	m_xe_x(init_xe_x), m_xe_y(init_xe_y), m_xe_z(init_xe_z), \
	m_vb_x(init_vb_x), m_vb_y(init_vb_y), m_vb_z(init_vb_z), \
	m_phai(init_phai), m_theta(init_theta), m_psi(init_psi), \
	m_p(init_p), m_q(init_q), m_r(init_r) {
		calculate_bar();
		m_pwl_Cs = piece_wise_linear(m_beta_w,m_Cs);
		m_pwl_Cym = piece_wise_linear(m_beta_w,m_Cym);
	}
	
	Vehicle_Body (const d_vec &params_d, const d_v_vec &params_v, const d_vec &init_states);

	void push_con_states (d_vec &con_states);
	
	void pull_con_states (const d_vec &con_states);
	void update_pv (const d_vec &inputs, d_vec &outputs);
	void update_fm (const d_vec &inputs, d_vec &outputs);
	void update_drv (d_vec &outputs);

private:
	//parameters
	//chassis
	double m_mass, m_a, m_b, m_d, m_h, m_Ixx, m_Ixy, m_Ixz, \
	m_Iyx, m_Iyy, m_Iyz, m_Izx, m_Izy, m_Izz, \
	m_w_f, m_w_r;
	//inertial loads
	//z1
	double m_z1m, m_z1R_x, m_z1R_y, m_z1R_z, m_z1I_xx, \
	m_z1I_xy, m_z1I_xz, m_z1I_yx, m_z1I_yy, m_z1I_yz, \
	m_z1I_zx, m_z1I_zy, m_z1I_zz;
	//z2
	double m_z2m, m_z2R_x, m_z2R_y, m_z2R_z, m_z2I_xx, \
	m_z2I_xy, m_z2I_xz, m_z2I_yx, m_z2I_yy, m_z2I_yz, \
	m_z2I_zx, m_z2I_zy, m_z2I_zz;
	//z3
	double m_z3m, m_z3R_x, m_z3R_y, m_z3R_z, m_z3I_xx, \
	m_z3I_xy, m_z3I_xz, m_z3I_yx, m_z3I_yy, m_z3I_yz, \
	m_z3I_zx, m_z3I_zy, m_z3I_zz;
	//z4
	double m_z4m, m_z4R_x, m_z4R_y, m_z4R_z, m_z4I_xx, \
	m_z4I_xy, m_z4I_xz, m_z4I_yx, m_z4I_yy, m_z4I_yz, \
	m_z4I_zx, m_z4I_zy, m_z4I_zz;
	//z5
	double m_z5m, m_z5R_x, m_z5R_y, m_z5R_z, m_z5I_xx, \
	m_z5I_xy, m_z5I_xz, m_z5I_yx, m_z5I_yy, m_z5I_yz, \
	m_z5I_zx, m_z5I_zy, m_z5I_zz;
	//z6
	double m_z6m, m_z6R_x, m_z6R_y, m_z6R_z, m_z6I_xx, \
	m_z6I_xy, m_z6I_xz, m_z6I_yx, m_z6I_yy, m_z6I_yz, \
	m_z6I_zx, m_z6I_zy, m_z6I_zz;
	//z7
	double m_z7m, m_z7R_x, m_z7R_y, m_z7R_z, m_z7I_xx, \
	m_z7I_xy, m_z7I_xz, m_z7I_yx, m_z7I_yy, m_z7I_yz, \
	m_z7I_zx, m_z7I_zy, m_z7I_zz;
	//aerodynamics
	double m_Pabs, m_Cg, m_Af, m_Cd, m_Cl, m_Cpm; 
	d_vec m_beta_w, m_Cs, m_Cym;
	//simulation
	double m_xdot_tol, m_longOff, m_latOff, m_vertOff;
	//inputs
	//WindXYZ in inertial coordinate
	double m_Wx = NaN, m_Wy = NaN, m_Wz = NaN;
	//AirTemp
	double m_Tair = NaN;
	//FSusp
	double m_Fx_fl = NaN, m_Fx_fr = NaN, m_Fx_rl = NaN, m_Fx_rr = NaN, \
	m_Fy_fl = NaN, m_Fy_fr = NaN, m_Fy_rl = NaN, m_Fy_rr = NaN, \
	m_Fz_fl = NaN, m_Fz_fr = NaN, m_Fz_rl = NaN, m_Fz_rr = NaN;
	//MSusp
	double m_Mx_fl = NaN, m_Mx_fr = NaN, m_Mx_rl = NaN, m_Mx_rr = NaN, \
	m_My_fl = NaN, m_My_fr = NaN, m_My_rl = NaN, m_My_rr = NaN, \
	m_Mz_fl = NaN, m_Mz_fr = NaN, m_Mz_rl = NaN, m_Mz_rr = NaN;
	//FExt
	double m_Fx_ext = NaN, m_Fy_ext = NaN, m_Fz_ext = NaN;
	//MExt
	double m_Mx_ext = NaN, m_My_ext = NaN, m_Mz_ext = NaN;
	//continuous states
	double m_xe_x, m_xe_y, m_xe_z, \
	m_vb_x, m_vb_y, m_vb_z, \
	m_phai, m_theta, m_psi, \
	m_p, m_q, m_r;
	//continuous states derivative
	double m_drv_xe_x, m_drv_xe_y, m_drv_xe_z, \
	m_drv_vb_x, m_drv_vb_y, m_drv_vb_z, \
	m_drv_phai, m_drv_theta, m_drv_psi, \
	m_drv_p, m_drv_q, m_drv_r;
	//middle variables
	double m_cos_phai = NaN;
	double m_cos_theta = NaN;
	double m_cos_psi = NaN;
	double m_sin_phai = NaN;
	double m_sin_theta = NaN;
	double m_sin_psi = NaN;
	double m_ve_x = NaN;
	double m_ve_y = NaN;
	double m_ve_z = NaN;
	double m_Mbar = NaN;
	double m_Ibar_xx = NaN;
	double m_Ibar_xy = NaN;
	double m_Ibar_xz = NaN;
	double m_Ibar_yx = NaN;
	double m_Ibar_yy = NaN;
	double m_Ibar_yz = NaN;
	double m_Ibar_zx = NaN;
	double m_Ibar_zy = NaN;
	double m_Ibar_zz = NaN;
	double m_Rbar_x = NaN;
	double m_Rbar_y = NaN;
	double m_Rbar_z = NaN;
	double m_Xbar_a = NaN;
	double m_Xbar_b = NaN;
	double m_Xbar_h = NaN;
	double m_Wbar_fl = NaN;
	double m_Wbar_fr = NaN;
	double m_Wbar_rl = NaN;
	double m_Wbar_rr = NaN;
	double m_HPbar_fl_x = NaN;
	double m_HPbar_fr_x = NaN;
	double m_HPbar_rl_x = NaN;
	double m_HPbar_rr_x = NaN;
	double m_HPbar_fl_y = NaN;
	double m_HPbar_fr_y = NaN;
	double m_HPbar_rl_y = NaN;
	double m_HPbar_rr_y = NaN;
	double m_HPbar_fl_z = NaN;
	double m_HPbar_fr_z = NaN;
	double m_HPbar_rl_z = NaN;
	double m_HPbar_rr_z = NaN;

	//--suspension force in total
	double m_F_VehiclB_x = NaN;
	double m_F_VehiclB_y = NaN;
	double m_F_VehiclB_z = NaN;
	//--suspension moment in total
	double m_M_roll = NaN;
	double m_M_pitch = NaN;
	double m_M_yaw = NaN;
	//--gravity force 
	double m_Fg_x = NaN;
	double m_Fg_y = NaN;
	double m_Fg_z = NaN;
	//--aero drag force and moment
	double m_Fd_x = NaN;
	double m_Fd_y = NaN;
	double m_Fd_z = NaN;
	double m_Md_x = NaN;
	double m_Md_y = NaN;
	double m_Md_z = NaN;
	//--total force and moment on vehicle body
	double m_Fb_x = NaN;
	double m_Fb_y = NaN;
	double m_Fb_z = NaN;
	double m_Mb_x = NaN;
	double m_Mb_y = NaN;
	double m_Mb_z = NaN;
	//--Cs Cym
	piece_wise_linear m_pwl_Cs = piece_wise_linear(m_beta_w,m_Cs);;
	piece_wise_linear m_pwl_Cym = piece_wise_linear(m_beta_w,m_Cs);;
	//outputs
	//--center of mass
	double m_xe_x_c = NaN;
	double m_xe_y_c = NaN;
	double m_xe_z_c = NaN;
	double m_ve_x_c = NaN;
	double m_ve_y_c = NaN;
	double m_ve_z_c = NaN;
	double m_vb_x_c = NaN;
	double m_vb_y_c = NaN;
	double m_vb_z_c = NaN;
	double m_phai_c = NaN;
	double m_theta_c = NaN;
	double m_psi_c = NaN;
	double m_p_c = NaN;
	double m_q_c = NaN;
	double m_r_c = NaN;
	double m_beta_c = NaN;

	double m_DCM_00 = NaN;
	double m_DCM_01 = NaN;
	double m_DCM_02 = NaN;
	double m_DCM_10 = NaN;
	double m_DCM_11 = NaN;
	double m_DCM_12 = NaN;
	double m_DCM_20 = NaN;
	double m_DCM_21 = NaN;
	double m_DCM_22 = NaN;
	double m_DCM_T_00 = NaN;
	double m_DCM_T_01 = NaN;
	double m_DCM_T_02 = NaN;
	double m_DCM_T_10 = NaN;
	double m_DCM_T_11 = NaN;
	double m_DCM_T_12 = NaN;
	double m_DCM_T_20 = NaN;
	double m_DCM_T_21 = NaN;
	double m_DCM_T_22 = NaN;
	//--offset poit
	double m_xe_x_o = NaN;
	double m_xe_y_o = NaN;
	double m_xe_z_o = NaN;
	double m_ve_x_o = NaN;
	double m_ve_y_o = NaN;
	double m_ve_z_o = NaN;
	double m_xb_x_o = NaN;
	double m_xb_y_o = NaN;
	double m_xb_z_o = NaN;
	double m_vb_x_o = NaN;
	double m_vb_y_o = NaN;
	double m_vb_z_o = NaN;
	double m_beta_o = NaN;
	//--hard points
	double m_xe_x_fl = NaN;
	double m_xe_y_fl = NaN;
	double m_xe_z_fl = NaN;
	double m_ve_x_fl = NaN;
	double m_ve_y_fl = NaN;
	double m_ve_z_fl = NaN;
	double m_xb_x_fl = NaN;
	double m_xb_y_fl = NaN;
	double m_xb_z_fl = NaN;
	double m_vb_x_fl = NaN;
	double m_vb_y_fl = NaN;
	double m_vb_z_fl = NaN;
	
	double m_xe_x_fr = NaN;
	double m_xe_y_fr = NaN;
	double m_xe_z_fr = NaN;
	double m_ve_x_fr = NaN;
	double m_ve_y_fr = NaN;
	double m_ve_z_fr = NaN;
	double m_xb_x_fr = NaN;
	double m_xb_y_fr = NaN;
	double m_xb_z_fr = NaN;
	double m_vb_x_fr = NaN;
	double m_vb_y_fr = NaN;
	double m_vb_z_fr = NaN;
	
	double m_xe_x_rl = NaN;
	double m_xe_y_rl = NaN;
	double m_xe_z_rl = NaN;
	double m_ve_x_rl = NaN;
	double m_ve_y_rl = NaN;
	double m_ve_z_rl = NaN;
	double m_xb_x_rl = NaN;
	double m_xb_y_rl = NaN;
	double m_xb_z_rl = NaN;
	double m_vb_x_rl = NaN;
	double m_vb_y_rl = NaN;
	double m_vb_z_rl = NaN;
	
	double m_xe_x_rr = NaN;
	double m_xe_y_rr = NaN;
	double m_xe_z_rr = NaN;
	double m_ve_x_rr = NaN;
	double m_ve_y_rr = NaN;
	double m_ve_z_rr = NaN;
	double m_xb_x_rr = NaN;
	double m_xb_y_rr = NaN;
	double m_xb_z_rr = NaN;
	double m_vb_x_rr = NaN;
	double m_vb_y_rr = NaN;
	double m_vb_z_rr = NaN;
	//
	void calculate_bar();
	void calculate_gravity();
	void calculate_aero_drag();
};

}	//end of name space
#endif //VEHICLE_BODY_HPP
