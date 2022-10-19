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
	static const int m_con_states_num = 12;									//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;					//amount of derivatives;
	static const int m_dis_states_num = 0;									//amount of discrete states;
	//real_Y a=1.515, real_Y b=1.504, real_Y d=0.0, real_Y h=0.134, 
	Vehicle_Body (\
	//chassis
	real_Y mass=1181.0, \
	real_Y a=1.515, real_Y b=1.504, real_Y d=0.0, real_Y h=0.134, \
	real_Y Ixx=4.323333333333333e2, real_Y Ixy=0.0, real_Y Ixz=0.0, \
	real_Y Iyx=0.0, real_Y Iyy=1.922666666666667e3, real_Y Iyz=0.0, \
	real_Y Izx=0.0, real_Y Izy=0.0, real_Y Izz=2066.0, \
	real_Y w_f=1.9220, real_Y w_r=1.9220, \
	//z1
	real_Y z1m=0.0, \
	real_Y z1R_x=-0.25, real_Y z1R_y=0.125,real_Y z1R_z = 0.15, \
	real_Y z1I_xx=0.0,  real_Y z1I_xy=0.0, real_Y z1I_xz=0.0, \
	real_Y z1I_yx=0.0,  real_Y z1I_yy=0.0, real_Y z1I_yz=0.0, 
	real_Y z1I_zx=0.0,  real_Y z1I_zy=0.0, real_Y z1I_zz=0.0, \
	//z2
	real_Y z2m=0.0,    real_Y z2R_x=1.4,  real_Y z2R_y=0.0, real_Y z2R_z = 0.8, \
	real_Y z2I_xx=0.0, real_Y z2I_xy=0.0, real_Y z2I_xz=0.0, \
	real_Y z2I_yx=0.0, real_Y z2I_yy=0.0, real_Y z2I_yz=0.0, \
	real_Y z2I_zx=0.0, real_Y z2I_zy=0.0, real_Y z2I_zz=0.0, \
	//z3
	real_Y z3m=0.0,    real_Y z3R_x=0.75, real_Y z3R_y=-0.5, real_Y z3R_z = 0.4, \
	real_Y z3I_xx=0.0, real_Y z3I_xy=0.0, real_Y z3I_xz=0.0, \
	real_Y z3I_yx=0.0, real_Y z3I_yy=0.0, real_Y z3I_yz=0.0, \
	real_Y z3I_zx=0.0, real_Y z3I_zy=0.0, real_Y z3I_zz=0.0, \
	//z4
	real_Y z4m=0.0, real_Y z4R_x=0.75, real_Y z4R_y=0.5, real_Y z4R_z = 0.4, \
	real_Y z4I_xx=0.0, real_Y z4I_xy=0.0, real_Y z4I_xz=0.0, \
	real_Y z4I_yx=0.0, real_Y z4I_yy=0.0, real_Y z4I_yz=0.0, \
	real_Y z4I_zx=0.0, real_Y z4I_zy=0.0, real_Y z4I_zz=0.0, \
	//z5
	real_Y z5m=0.0, real_Y z5R_x=1.25, real_Y z5R_y=-0.5, real_Y z5R_z = 0.4, 
	real_Y z5I_xx=0.0, real_Y z5I_xy=0.0, real_Y z5I_xz=0.0, 
	real_Y z5I_yx=0.0, real_Y z5I_yy=0.0, real_Y z5I_yz=0.0, 
	real_Y z5I_zx=0.0, real_Y z5I_zy=0.0, real_Y z5I_zz=0.0, \
	//z6
	real_Y z6m=0.0, real_Y z6R_x=1.25, real_Y z6R_y=-0.5, real_Y z6R_z = 0.4, 
	real_Y z6I_xx=0.0, real_Y z6I_xy=0.0, real_Y z6I_xz=0.0, 
	real_Y z6I_yx=0.0, real_Y z6I_yy=0.0, real_Y z6I_yz=0.0, 
	real_Y z6I_zx=0.0, real_Y z6I_zy=0.0, real_Y z6I_zz=0.0, \
	//z7
	real_Y z7m=0.0, real_Y z7R_x=2.0, real_Y z7R_y=0.0, real_Y z7R_z = 0.25, 
	real_Y z7I_xx=0.0, real_Y z7I_xy=0.0, real_Y z7I_xz=0.0, 
	real_Y z7I_yx=0.0, real_Y z7I_yy=0.0, real_Y z7I_yz=0.0, 
	real_Y z7I_zx=0.0, real_Y z7I_zy=0.0, real_Y z7I_zz=0.0, \
	//aerodynamics
	//real_Y Pabs=101325.0, real_Y Cg=287.058;
	real_Y Pabs=101325.0, real_Y Cg=287.058, \
	real_Y Af=2.110, real_Y Cd=0.3, real_Y Cl=0.1, real_Y Cpm=0.1, \
	d_vec beta_w={0.0,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,\
	0.10,0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.20,\
	0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.30}, \
	d_vec Cs	={0.0,0.03,0.06,0.09,0.12,0.15,0.18,0.21,0.24,0.27,0.30,\
	0.33,0.36,0.39,0.42,0.45,0.48,0.51,0.54,0.57,0.60,\
	0.63,0.66,0.69,0.72,0.75,0.78,0.81,0.84,0.87,0.90}, \
	d_vec Cym	={0.0,0.01,0.02,0.03,0.04,0.05,0.06,0.07,0.08,0.09,\
	0.10,0.11,0.12,0.13,0.14,0.15,0.16,0.17,0.18,0.19,0.20,\
	0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.30}, \
	//simulation
	real_Y xdot_tol=0.1, real_Y longOff=0.0, real_Y latOff=0.0, real_Y vertOff=0.0112, \
	//init states
	real_Y init_xe_x=0.0, real_Y init_xe_y=3.125, real_Y init_xe_z=0.0, \
	real_Y init_vb_x=0.0, real_Y init_vb_y=0.0, real_Y init_vb_z=0.0, \
	real_Y init_phai=0.0, real_Y init_theta=0.0, real_Y init_psi=0.0, \
	real_Y init_p=0.0, real_Y init_q=0.0, real_Y init_r=0.0) :
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
	void push_con_states (d_vec &con_states);
	void pull_con_states (const d_vec &con_states);

	void pull_pv (const real_Y &Air_Wx, const real_Y &Air_Wy, const real_Y &Air_Wz);
	void push_pv(real_Y &h_c, real_Y &xe_x_c, real_Y &xe_y_c, real_Y &xe_z_c, real_Y &ve_x_c, real_Y &ve_y_c, real_Y &ve_z_c, \
	real_Y &vb_x_c, real_Y &vb_y_c, real_Y &vb_z_c, real_Y &phai_c, real_Y &theta_c, real_Y &psi_c, real_Y &p_c, real_Y &q_c, real_Y &r_c, real_Y &beta_c, \
	real_Y &xe_x_o, real_Y &xe_y_o, real_Y &xe_z_o, real_Y &ve_x_o, real_Y &ve_y_o, real_Y &ve_z_o, \
	real_Y &xb_x_o, real_Y &xb_y_o, real_Y &xb_z_o, real_Y &vb_x_o, real_Y &vb_y_o, real_Y &vb_z_o, real_Y &beta_o, \
	real_Y &xe_x_fl, real_Y &xe_y_fl, real_Y &xe_z_fl, real_Y &ve_x_fl, real_Y &ve_y_fl, real_Y &ve_z_fl, \
	real_Y &xb_x_fl, real_Y &xb_y_fl, real_Y &xb_z_fl, real_Y &vb_x_fl, real_Y &vb_y_fl, real_Y &vb_z_fl, \
	real_Y &xe_x_fr, real_Y &xe_y_fr, real_Y &xe_z_fr, real_Y &ve_x_fr, real_Y &ve_y_fr, real_Y &ve_z_fr, \
	real_Y &xb_x_fr, real_Y &xb_y_fr, real_Y &xb_z_fr, real_Y &vb_x_fr, real_Y &vb_y_fr, real_Y &vb_z_fr, \
	real_Y &xe_x_rl, real_Y &xe_y_rl, real_Y &xe_z_rl, real_Y &ve_x_rl, real_Y &ve_y_rl, real_Y &ve_z_rl, \
	real_Y &xb_x_rl, real_Y &xb_y_rl, real_Y &xb_z_rl, real_Y &vb_x_rl, real_Y &vb_y_rl, real_Y &vb_z_rl, \
	real_Y &xe_x_rr, real_Y &xe_y_rr, real_Y &xe_z_rr, real_Y &ve_x_rr, real_Y &ve_y_rr, real_Y &ve_z_rr, \
	real_Y &xb_x_rr, real_Y &xb_y_rr, real_Y &xb_z_rr, real_Y &vb_x_rr, real_Y &vb_y_rr, real_Y &vb_z_rr);
	void pull_fm (const real_Y &Air_Tair, \
	const real_Y &Sus_Fx_fl, const real_Y &Sus_Fx_fr, const real_Y &Sus_Fx_rl, const real_Y &Sus_Fx_rr, \
	const real_Y &Sus_Fy_fl, const real_Y &Sus_Fy_fr, const real_Y &Sus_Fy_rl, const real_Y &Sus_Fy_rr, \
	const real_Y &Sus_Fz_fl, const real_Y &Sus_Fz_fr, const real_Y &Sus_Fz_rl, const real_Y &Sus_Fz_rr, \
	const real_Y &Sus_Mx_fl, const real_Y &Sus_Mx_fr, const real_Y &Sus_Mx_rl, const real_Y &Sus_Mx_rr ,\
	const real_Y &Sus_My_fl, const real_Y &Sus_My_fr, const real_Y &Sus_My_rl, const real_Y &Sus_My_rr, \
	const real_Y &Sus_Mz_fl, const real_Y &Sus_Mz_fr, const real_Y &Sus_Mz_rl, const real_Y &Sus_Mz_rr, \
	const real_Y &Ext_Fx_ext, const real_Y &Ext_Fy_ext, const real_Y &Ext_Fz_ext, \
	const real_Y &Ext_Mx_ext, const real_Y &Ext_My_ext, const real_Y &Ext_Mz_ext);
	void push_fm () {};

	void push_drv (d_vec &derivatives);

private:
	//parameters
	//chassis
	real_Y m_mass, m_a, m_b, m_d, m_h, m_Ixx, m_Ixy, m_Ixz, \
	m_Iyx, m_Iyy, m_Iyz, m_Izx, m_Izy, m_Izz, \
	m_w_f, m_w_r;
	//inertial loads
	//z1
	real_Y m_z1m, m_z1R_x, m_z1R_y, m_z1R_z, m_z1I_xx, \
	m_z1I_xy, m_z1I_xz, m_z1I_yx, m_z1I_yy, m_z1I_yz, \
	m_z1I_zx, m_z1I_zy, m_z1I_zz;
	//z2
	real_Y m_z2m, m_z2R_x, m_z2R_y, m_z2R_z, m_z2I_xx, \
	m_z2I_xy, m_z2I_xz, m_z2I_yx, m_z2I_yy, m_z2I_yz, \
	m_z2I_zx, m_z2I_zy, m_z2I_zz;
	//z3
	real_Y m_z3m, m_z3R_x, m_z3R_y, m_z3R_z, m_z3I_xx, \
	m_z3I_xy, m_z3I_xz, m_z3I_yx, m_z3I_yy, m_z3I_yz, \
	m_z3I_zx, m_z3I_zy, m_z3I_zz;
	//z4
	real_Y m_z4m, m_z4R_x, m_z4R_y, m_z4R_z, m_z4I_xx, \
	m_z4I_xy, m_z4I_xz, m_z4I_yx, m_z4I_yy, m_z4I_yz, \
	m_z4I_zx, m_z4I_zy, m_z4I_zz;
	//z5
	real_Y m_z5m, m_z5R_x, m_z5R_y, m_z5R_z, m_z5I_xx, \
	m_z5I_xy, m_z5I_xz, m_z5I_yx, m_z5I_yy, m_z5I_yz, \
	m_z5I_zx, m_z5I_zy, m_z5I_zz;
	//z6
	real_Y m_z6m, m_z6R_x, m_z6R_y, m_z6R_z, m_z6I_xx, \
	m_z6I_xy, m_z6I_xz, m_z6I_yx, m_z6I_yy, m_z6I_yz, \
	m_z6I_zx, m_z6I_zy, m_z6I_zz;
	//z7
	real_Y m_z7m, m_z7R_x, m_z7R_y, m_z7R_z, m_z7I_xx, \
	m_z7I_xy, m_z7I_xz, m_z7I_yx, m_z7I_yy, m_z7I_yz, \
	m_z7I_zx, m_z7I_zy, m_z7I_zz;
	//aerodynamics
	real_Y m_Pabs, m_Cg, m_Af, m_Cd, m_Cl, m_Cpm; 
	d_vec m_beta_w, m_Cs, m_Cym;
	//simulation
	real_Y m_xdot_tol, m_longOff, m_latOff, m_vertOff;
	//inputs
	//WindXYZ in inertial coordinate
	real_Y m_Air_Wx = NaN, m_Air_Wy = NaN, m_Air_Wz = NaN;
	//AirTemp
	real_Y m_Air_Tair = NaN;
	//FSusp
	real_Y m_Sus_Fx_fl = NaN, m_Sus_Fx_fr = NaN, m_Sus_Fx_rl = NaN, m_Sus_Fx_rr = NaN, \
	m_Sus_Fy_fl = NaN, m_Sus_Fy_fr = NaN, m_Sus_Fy_rl = NaN, m_Sus_Fy_rr = NaN, \
	m_Sus_Fz_fl = NaN, m_Sus_Fz_fr = NaN, m_Sus_Fz_rl = NaN, m_Sus_Fz_rr = NaN;
	//MSusp
	real_Y m_Sus_Mx_fl = NaN, m_Sus_Mx_fr = NaN, m_Sus_Mx_rl = NaN, m_Sus_Mx_rr = NaN, \
	m_Sus_My_fl = NaN, m_Sus_My_fr = NaN, m_Sus_My_rl = NaN, m_Sus_My_rr = NaN, \
	m_Sus_Mz_fl = NaN, m_Sus_Mz_fr = NaN, m_Sus_Mz_rl = NaN, m_Sus_Mz_rr = NaN;
	//FExt
	real_Y m_Ext_Fx_ext = NaN, m_Ext_Fy_ext = NaN, m_Ext_Fz_ext = NaN;
	//MExt
	real_Y m_Ext_Mx_ext = NaN, m_Ext_My_ext = NaN, m_Ext_Mz_ext = NaN;
	//continuous states
	real_Y m_xe_x, m_xe_y, m_xe_z, \
	m_vb_x, m_vb_y, m_vb_z, \
	m_phai, m_theta, m_psi, \
	m_p, m_q, m_r;
	//continuous states derivative
	real_Y m_drv_xe_x, m_drv_xe_y, m_drv_xe_z, \
	m_drv_vb_x, m_drv_vb_y, m_drv_vb_z, \
	m_drv_phai, m_drv_theta, m_drv_psi, \
	m_drv_p, m_drv_q, m_drv_r;
	//middle variables
	real_Y m_cos_phai = NaN;
	real_Y m_cos_theta = NaN;
	real_Y m_cos_psi = NaN;
	real_Y m_sin_phai = NaN;
	real_Y m_sin_theta = NaN;
	real_Y m_sin_psi = NaN;
	real_Y m_ve_x = NaN;
	real_Y m_ve_y = NaN;
	real_Y m_ve_z = NaN;
	real_Y m_Mbar = NaN;
	real_Y m_Ibar_xx = NaN;
	real_Y m_Ibar_xy = NaN;
	real_Y m_Ibar_xz = NaN;
	real_Y m_Ibar_yx = NaN;
	real_Y m_Ibar_yy = NaN;
	real_Y m_Ibar_yz = NaN;
	real_Y m_Ibar_zx = NaN;
	real_Y m_Ibar_zy = NaN;
	real_Y m_Ibar_zz = NaN;
	real_Y m_Rbar_x = NaN;
	real_Y m_Rbar_y = NaN;
	real_Y m_Rbar_z = NaN;
	real_Y m_Xbar_a = NaN;
	real_Y m_Xbar_b = NaN;
	real_Y m_Xbar_h = NaN;
	real_Y m_Wbar_fl = NaN;
	real_Y m_Wbar_fr = NaN;
	real_Y m_Wbar_rl = NaN;
	real_Y m_Wbar_rr = NaN;
	real_Y m_HPbar_fl_x = NaN;
	real_Y m_HPbar_fr_x = NaN;
	real_Y m_HPbar_rl_x = NaN;
	real_Y m_HPbar_rr_x = NaN;
	real_Y m_HPbar_fl_y = NaN;
	real_Y m_HPbar_fr_y = NaN;
	real_Y m_HPbar_rl_y = NaN;
	real_Y m_HPbar_rr_y = NaN;
	real_Y m_HPbar_fl_z = NaN;
	real_Y m_HPbar_fr_z = NaN;
	real_Y m_HPbar_rl_z = NaN;
	real_Y m_HPbar_rr_z = NaN;

	real_Y m_invIbar_xx = NaN;
	real_Y m_invIbar_xy = NaN;
	real_Y m_invIbar_xz = NaN;
	real_Y m_invIbar_yx = NaN;
	real_Y m_invIbar_yy = NaN;
	real_Y m_invIbar_yz = NaN;
	real_Y m_invIbar_zx = NaN;
	real_Y m_invIbar_zy = NaN;
	real_Y m_invIbar_zz = NaN;

	//--suspension force in total
	real_Y m_F_VehiclB_x = NaN;
	real_Y m_F_VehiclB_y = NaN;
	real_Y m_F_VehiclB_z = NaN;
	//--suspension moment in total
	real_Y m_M_roll = NaN;
	real_Y m_M_pitch = NaN;
	real_Y m_M_yaw = NaN;
	//--gravity force 
	real_Y m_Fg_x = NaN;
	real_Y m_Fg_y = NaN;
	real_Y m_Fg_z = NaN;
	//--aero drag force and moment
	real_Y m_Fd_x = NaN;
	real_Y m_Fd_y = NaN;
	real_Y m_Fd_z = NaN;
	real_Y m_Md_x = NaN;
	real_Y m_Md_y = NaN;
	real_Y m_Md_z = NaN;
	//--total force and moment on vehicle body
	real_Y m_Fb_x = NaN;
	real_Y m_Fb_y = NaN;
	real_Y m_Fb_z = NaN;
	real_Y m_Mb_x = NaN;
	real_Y m_Mb_y = NaN;
	real_Y m_Mb_z = NaN;
	//--Cs Cym
	piece_wise_linear m_pwl_Cs = piece_wise_linear(m_beta_w,m_Cs);
	piece_wise_linear m_pwl_Cym = piece_wise_linear(m_beta_w,m_Cs);
	//outputs
	//--center of mass
	real_Y m_xe_x_c = NaN;
	real_Y m_xe_y_c = NaN;
	real_Y m_xe_z_c = NaN;
	real_Y m_ve_x_c = NaN;
	real_Y m_ve_y_c = NaN;
	real_Y m_ve_z_c = NaN;
	real_Y m_vb_x_c = NaN;
	real_Y m_vb_y_c = NaN;
	real_Y m_vb_z_c = NaN;
	real_Y m_phai_c = NaN;
	real_Y m_theta_c = NaN;
	real_Y m_psi_c = NaN;
	real_Y m_p_c = NaN;
	real_Y m_q_c = NaN;
	real_Y m_r_c = NaN;
	real_Y m_beta_c = NaN;

	real_Y m_DCM_00 = NaN;
	real_Y m_DCM_01 = NaN;
	real_Y m_DCM_02 = NaN;
	real_Y m_DCM_10 = NaN;
	real_Y m_DCM_11 = NaN;
	real_Y m_DCM_12 = NaN;
	real_Y m_DCM_20 = NaN;
	real_Y m_DCM_21 = NaN;
	real_Y m_DCM_22 = NaN;
	real_Y m_DCM_T_00 = NaN;
	real_Y m_DCM_T_01 = NaN;
	real_Y m_DCM_T_02 = NaN;
	real_Y m_DCM_T_10 = NaN;
	real_Y m_DCM_T_11 = NaN;
	real_Y m_DCM_T_12 = NaN;
	real_Y m_DCM_T_20 = NaN;
	real_Y m_DCM_T_21 = NaN;
	real_Y m_DCM_T_22 = NaN;
	//--offset poit
	real_Y m_xe_x_o = NaN;
	real_Y m_xe_y_o = NaN;
	real_Y m_xe_z_o = NaN;
	real_Y m_ve_x_o = NaN;
	real_Y m_ve_y_o = NaN;
	real_Y m_ve_z_o = NaN;
	real_Y m_xb_x_o = NaN;
	real_Y m_xb_y_o = NaN;
	real_Y m_xb_z_o = NaN;
	real_Y m_vb_x_o = NaN;
	real_Y m_vb_y_o = NaN;
	real_Y m_vb_z_o = NaN;
	real_Y m_beta_o = NaN;
	//--hard points
	real_Y m_xe_x_fl = NaN;
	real_Y m_xe_y_fl = NaN;
	real_Y m_xe_z_fl = NaN;
	real_Y m_ve_x_fl = NaN;
	real_Y m_ve_y_fl = NaN;
	real_Y m_ve_z_fl = NaN;
	real_Y m_xb_x_fl = NaN;
	real_Y m_xb_y_fl = NaN;
	real_Y m_xb_z_fl = NaN;
	real_Y m_vb_x_fl = NaN;
	real_Y m_vb_y_fl = NaN;
	real_Y m_vb_z_fl = NaN;
	
	real_Y m_xe_x_fr = NaN;
	real_Y m_xe_y_fr = NaN;
	real_Y m_xe_z_fr = NaN;
	real_Y m_ve_x_fr = NaN;
	real_Y m_ve_y_fr = NaN;
	real_Y m_ve_z_fr = NaN;
	real_Y m_xb_x_fr = NaN;
	real_Y m_xb_y_fr = NaN;
	real_Y m_xb_z_fr = NaN;
	real_Y m_vb_x_fr = NaN;
	real_Y m_vb_y_fr = NaN;
	real_Y m_vb_z_fr = NaN;
	
	real_Y m_xe_x_rl = NaN;
	real_Y m_xe_y_rl = NaN;
	real_Y m_xe_z_rl = NaN;
	real_Y m_ve_x_rl = NaN;
	real_Y m_ve_y_rl = NaN;
	real_Y m_ve_z_rl = NaN;
	real_Y m_xb_x_rl = NaN;
	real_Y m_xb_y_rl = NaN;
	real_Y m_xb_z_rl = NaN;
	real_Y m_vb_x_rl = NaN;
	real_Y m_vb_y_rl = NaN;
	real_Y m_vb_z_rl = NaN;
	
	real_Y m_xe_x_rr = NaN;
	real_Y m_xe_y_rr = NaN;
	real_Y m_xe_z_rr = NaN;
	real_Y m_ve_x_rr = NaN;
	real_Y m_ve_y_rr = NaN;
	real_Y m_ve_z_rr = NaN;
	real_Y m_xb_x_rr = NaN;
	real_Y m_xb_y_rr = NaN;
	real_Y m_xb_z_rr = NaN;
	real_Y m_vb_x_rr = NaN;
	real_Y m_vb_y_rr = NaN;
	real_Y m_vb_z_rr = NaN;
	//
	void calculate_bar();
	void calculate_gravity();
	void calculate_aero_drag();
};

}	//end of name space
#endif //VEHICLE_BODY_HPP
