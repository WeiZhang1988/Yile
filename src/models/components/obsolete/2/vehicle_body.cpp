#include "vehicle_body.hpp"

NMSPC::Vehicle_Body::Vehicle_Body(const d_vec &params, const d_vec &init_states, \
const double &t) {
	//parameters
	//chassis
	m_mass 		= params[0];
	m_a 		= params[1];
	m_b 		= params[2];
	m_d 		= params[3];
	m_h 		= params[4];
	m_Ixx 		= params[5];
	m_Ixy 		= params[6];
	m_Ixz 		= params[7];
	m_Iyx 		= params[8];
	m_Iyy 		= params[9];
	m_Iyz 		= params[10];
	m_Izx 		= params[11];
	m_Izy 		= params[12];
	m_Izz 		= params[13];
	m_w_f	= params[14];
	m_w_r	= params[15];
	//inertial loads
	//z1
	m_z1m 		= params[16];
	m_z1R_x 	= params[17];
	m_z1R_y 	= params[18];
	m_z1R_z		= params[19];
	m_z1I_xx 	= params[20];
	m_z1I_xy 	= params[21];
	m_z1I_xz 	= params[22];
	m_z1I_yx 	= params[23];
	m_z1I_yy 	= params[24];
	m_z1I_yz 	= params[25];
	m_z1I_zx 	= params[26];
	m_z1I_zy 	= params[27];
	m_z1I_zz 	= params[28];
	//z2
	m_z2m 		= params[29];
	m_z2R_x 	= params[30];
	m_z2R_y 	= params[31];
	m_z2R_z		= params[32];
	m_z2I_xx 	= params[33];
	m_z2I_xy 	= params[34];
	m_z2I_xz 	= params[35];
	m_z2I_yx 	= params[36];
	m_z2I_yy 	= params[37];
	m_z2I_yz 	= params[38];
	m_z2I_zx 	= params[39];
	m_z2I_zy 	= params[40];
	m_z2I_zz 	= params[41];
	//z3
	m_z3m 		= params[42];
	m_z3R_x 	= params[43];
	m_z3R_y 	= params[44];
	m_z3R_z		= params[45];
	m_z3I_xx 	= params[46];
	m_z3I_xy 	= params[47];
	m_z3I_xz 	= params[48];
	m_z3I_yx 	= params[49];
	m_z3I_yy 	= params[50];
	m_z3I_yz 	= params[51];
	m_z3I_zx 	= params[52];
	m_z3I_zy 	= params[53];
	m_z3I_zz 	= params[54];
	//z4
	m_z4m 		= params[55];
	m_z4R_x 	= params[56];
	m_z4R_y 	= params[57];
	m_z4R_z		= params[58];
	m_z4I_xx 	= params[59];
	m_z4I_xy 	= params[60];
	m_z4I_xz 	= params[61];
	m_z4I_yx 	= params[62];
	m_z4I_yy 	= params[63];
	m_z4I_yz 	= params[64];
	m_z4I_zx 	= params[65];
	m_z4I_zy 	= params[66];
	m_z4I_zz 	= params[67];
	//z5
	m_z5m 		= params[68];
	m_z5R_x 	= params[69];
	m_z5R_y 	= params[70];
	m_z5R_z		= params[71];
	m_z5I_xx 	= params[72];
	m_z5I_xy 	= params[73];
	m_z5I_xz 	= params[74];
	m_z5I_yx 	= params[75];
	m_z5I_yy 	= params[76];
	m_z5I_yz 	= params[77];
	m_z5I_zx 	= params[78];
	m_z5I_zy 	= params[79];
	m_z5I_zz 	= params[80];
	//z6
	m_z6m 		= params[81];
	m_z6R_x 	= params[82];
	m_z6R_y 	= params[83];
	m_z6R_z		= params[84];
	m_z6I_xx 	= params[85];
	m_z6I_xy 	= params[86];
	m_z6I_xz 	= params[87];
	m_z6I_yx 	= params[88];
	m_z6I_yy 	= params[89];
	m_z6I_yz 	= params[90];
	m_z6I_zx 	= params[91];
	m_z6I_zy 	= params[92];
	m_z6I_zz 	= params[93];
	//z7
	m_z7m 		= params[94];
	m_z7R_x 	= params[95];
	m_z7R_y 	= params[96];
	m_z7R_z		= params[97];
	m_z7I_xx 	= params[98];
	m_z7I_xy 	= params[99];
	m_z7I_xz 	= params[100];
	m_z7I_yx 	= params[101];
	m_z7I_yy 	= params[102];
	m_z7I_yz 	= params[103];
	m_z7I_zx 	= params[104];
	m_z7I_zy 	= params[105];
	m_z7I_zz 	= params[106];
	//aerodynamics
	m_Af  	 = params[107];
	m_Cd  	 = params[108]; 
	m_Cl  	 = params[109]; 
	m_Cpm 	 = params[110]; 
	m_beta_w = params[111];
	m_Cs 	 = params[112];
	m_Cym 	 = params[113];
	//simulation
	m_xdot_tol = params[114]; 
	m_longOff  = params[115]; 
	m_latOff   = params[116]; 
	m_vertOff  = params[117];
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
	//time
	m_t = t;
	//caculate bar
	calculate_bar();
	//feed vector of continuous states
	m_con_states[0] = m_xe_x;
	m_con_states[1] = m_xe_y;
	m_con_states[2] = m_xe_z;
	m_con_states[3] = m_vb_x;
	m_con_states[4] = m_vb_y;
	m_con_states[5] = m_vb_z;
	m_con_states[6] = m_phai;
	m_con_states[7] = m_theta;
	m_con_states[8] = m_psi;
	m_con_states[9] = m_p;
	m_con_states[10] = m_q;
	m_con_states[11] = m_r;
	//initialize kinematics outputs
	m_euler(0)	= m_phai;
	m_euler(1)	= m_theta;
	m_euler(2)	= m_psi;
	m_pqr(0)	= m_p;
	m_pqr(1)	= m_q;
	m_pqr(2)	= m_r;
	m_cos_phai 	= cos(m_phai);
	m_cos_theta	= cos(m_theta);
	m_cos_psi	= cos(m_psi);
	m_sin_phai 	= sin(m_phai);
	m_sin_theta	= sin(m_theta);
	m_sin_psi	= sin(m_psi);
	m_DCM(0,0)	= m_cos_theta * m_cos_phai;
	m_DCM(0,1)	= m_cos_theta * m_sin_phai;
	m_DCM(0,2)	= -m_sin_theta;
	m_DCM(1,0)	= m_sin_psi * m_sin_theta * m_cos_phai - \
	m_cos_psi * m_sin_phai;
	m_DCM(1,1)	= m_sin_psi * m_sin_theta * m_sin_phai + \
	m_cos_psi * m_cos_phai;
	m_DCM(1,2)	= m_sin_psi * m_cos_theta;
	m_DCM(2,0)	= m_cos_psi * m_sin_theta * m_cos_phai + \
	m_sin_psi * m_sin_phai;
	m_DCM(2,1)	= m_cos_psi * m_sin_theta * m_sin_phai - \
	m_sin_psi * m_cos_phai;
	m_DCM(2,2)	= m_cos_psi * m_cos_theta;
	m_xe(0) = m_xe_x;
	m_xe(1) = m_xe_y;
	m_xe(2) = m_xe_z;
	m_vb(0) = m_vb_x;
	m_vb(1) = m_vb_y;
	m_vb(2) = m_vb_z;
	m_ve = m_DCM.transpose() * m_vb;
	m_ve_x = m_ve(0);
	m_ve_y = m_ve(1);
	m_ve_z = m_ve(2);
	
}

void NMSPC::Vehicle_Body::preupdate (const d_vec &u, const double &t) {
	m_con_states[0]  = m_xe_x;
	m_con_states[1]  = m_xe_y;
	m_con_states[2]  = m_xe_z;
	m_con_states[3]  = m_vb_x;
	m_con_states[4]  = m_vb_y;
	m_con_states[5]  = m_vb_z;
	m_con_states[6]  = m_phai;
	m_con_states[7]  = m_theta;
	m_con_states[8]  = m_psi;
	m_con_states[9]  = m_p;
	m_con_states[10] = m_q;
	m_con_states[11] = m_r;
	//get system time
	m_t = t;
	//get inputs
    //FSusp
	m_Fx_fl = u[0];
	m_Fx_fr = u[1];
	m_Fx_rl = u[2];
	m_Fx_rr = u[3];
	m_Fy_fl = u[4];
	m_Fy_fr = u[5];
	m_Fy_rl = u[6];
	m_Fy_rr = u[7];
	m_Fz_fl = u[8];
	m_Fz_fr = u[9];
	m_Fz_rl = u[10];
	m_Fz_rr = u[11];
	//MSusp
	m_Mx_fl = u[12];
	m_Mx_fr = u[13];
	m_Mx_rl = u[14];
	m_Mx_rr = u[15];
	m_My_fl = u[16];
	m_My_fr = u[17];
	m_My_rl = u[18];
	m_My_rr = u[19];
	m_Mz_fl = u[20];
	m_Mz_fr = u[21];
	m_Mz_rl = u[22];
	m_Mz_rr = u[23];
	//FExt
	m_Fx_ext = u[24]; 
	m_Fy_ext = u[25]; 
	m_Fz_ext = u[26];
	//MExt
	m_Mx_ext = u[27];
	m_My_ext = u[28];
	m_Mz_ext = u[29];
	//WindXYZ in inertial coordinate
	m_Wx = u[30];
	m_Wy = u[31];
	m_Wz = u[32];
	//AirTemp
	m_Tair = u[33];
	//fill matrices and vectorers
	m_euler(0)	= m_phai;
	m_euler(1)	= m_theta;
	m_euler(2)	= m_psi;
	m_pqr(0)	= m_p;
	m_pqr(1)	= m_q;
	m_pqr(2)	= m_r;
	m_cos_phai 	= cos(m_phai);
	m_cos_theta	= cos(m_theta);
	m_cos_psi	= cos(m_psi);
	m_sin_phai 	= sin(m_phai);
	m_sin_theta	= sin(m_theta);
	m_sin_psi	= sin(m_psi);
	m_DCM(0,0)	= m_cos_theta * m_cos_phai;
	m_DCM(0,1)	= m_cos_theta * m_sin_phai;
	m_DCM(0,2)	= -m_sin_theta;
	m_DCM(1,0)	= m_sin_psi * m_sin_theta * m_cos_phai - \
	m_cos_psi * m_sin_phai;
	m_DCM(1,1)	= m_sin_psi * m_sin_theta * m_sin_phai + \
	m_cos_psi * m_cos_phai;
	m_DCM(1,2)	= m_sin_psi * m_cos_theta;
	m_DCM(2,0)	= m_cos_psi * m_sin_theta * m_cos_phai + \
	m_sin_psi * m_sin_phai;
	m_DCM(2,1)	= m_cos_psi * m_sin_theta * m_sin_phai - \
	m_sin_psi * m_cos_phai;
	m_DCM(2,2)	= m_cos_psi * m_cos_theta;
	m_xe(0) = m_xe_x;
	m_xe(1) = m_xe_y;
	m_xe(2) = m_xe_z;
	m_vb(0) = m_vb_x;
	m_vb(1) = m_vb_y;
	m_vb(2) = m_vb_z;
	m_ve = m_DCM.transpose() * m_vb;
	m_ve_x = m_ve(0);
	m_ve_y = m_ve(1);
	m_ve_z = m_ve(2);
	
	//
	m_F_VehiclB_x = m_Fx_fl + m_Fx_fr + m_Fx_rl + m_Fx_rr;
	m_F_VehiclB_y = m_Fy_fl + m_Fy_fr + m_Fy_rl + m_Fy_rr;
	m_F_VehiclB_z = m_Fz_fl + m_Fz_fr + m_Fz_rl + m_Fz_rr;
	m_M_roll = -m_Fz_fl * m_Wbar_fl + m_Fz_fr * m_Wbar_fr - \
	m_Fz_rl * m_Wbar_rl + m_Fz_rr * m_Wbar_rr - \
	m_F_VehiclB_y * m_Xbar_h;
	m_M_pitch = -(m_Fz_fl + m_Fz_fr) * m_Xbar_a + \
	(m_Fz_rl + m_Fz_rr) * m_Xbar_b + 
	m_F_VehiclB_x * m_Xbar_h;
	m_M_yaw = m_Fx_fl * m_Wbar_fl - m_Fx_fr *  m_Wbar_fr + \
	m_Fx_rl * m_Wbar_rl - m_Fx_rr * m_Wbar_rr + \
	(m_Fy_fl + m_Fy_fr) * m_Xbar_a - (m_Fy_rl + m_Fy_rr) * m_Xbar_b;
	
	calculate_gravity();
	calculate_aero_drag();
	m_Fb_x = m_Fx_ext + m_Fg_x + m_F_VehiclB_x - m_Fd_x;
	m_Fb_y = m_Fy_ext + m_Fg_y + m_F_VehiclB_y - m_Fd_y;
	m_Fb_z = m_Fz_ext + m_Fg_z + m_F_VehiclB_z - m_Fd_z;
	m_Mb_x = m_Mx_ext + m_M_roll + m_Mx_fl + m_Mx_fr + m_Mx_rl + m_Mx_rr - m_Md_x;
	m_Mb_y = m_My_ext + m_M_pitch + m_My_fl + m_My_fr + m_My_rl + m_My_rr - m_Md_y;
	m_Mb_z = m_Mz_ext + m_M_yaw + m_Mz_fl + m_Mz_fr + m_Mz_rl + m_Mz_rr - m_Md_z;

}

void NMSPC::Vehicle_Body::calculate_derivatives(double &drv_xe_x, double &drv_xe_y, \
double &drv_xe_z, double &drv_vb_x, double &drv_vb_y, \
double &drv_vb_z, double &drv_phai, double &drv_theta, \
double &drv_psi, double &drv_p, double &drv_q, double &drv_r) {
	
	drv_xe_x = m_ve_x;
	drv_xe_y = m_ve_y;
	drv_xe_z = m_ve_z;
	Eigen::Vector3d tmp_fom(m_Fb_x/m_Mbar, m_Fb_y/m_Mbar, m_Fb_z/m_Mbar);
	Eigen::Vector3d tmp_Ab = tmp_fom + m_vb.cross(m_pqr);
	drv_vb_x = tmp_Ab(0);
	drv_vb_y = tmp_Ab(1);
	drv_vb_z = tmp_Ab(2);
	
	drv_phai = m_p + (m_q * m_sin_phai + m_r * m_cos_phai) * \
	m_sin_theta / m_cos_theta;
	drv_theta = m_q * m_cos_phai - m_r * m_sin_phai;
	drv_psi = (m_q * m_sin_phai + m_r * m_cos_phai) / m_cos_theta;
	Eigen::Vector3d tmp_mb(m_Mb_x, m_Mb_y, m_Mb_z);
	Eigen::Vector3d tmp_Apqr = m_Ibar.inverse() * (tmp_mb - \
	m_pqr.cross(m_Ibar * m_pqr));
	drv_p = tmp_Apqr(0);
	drv_q = tmp_Apqr(1);
	drv_r = tmp_Apqr(2);
}

void NMSPC::Vehicle_Body::postupdate () {
	m_xe_x 	= m_con_states[0];
	m_xe_y 	= m_con_states[1];
	m_xe_z 	= m_con_states[2];
	m_vb_x 	= m_con_states[3];
	m_vb_y 	= m_con_states[4];
	m_vb_z 	= m_con_states[5];
	m_phai 	= m_con_states[6];
	m_theta = m_con_states[7];
	m_psi 	= m_con_states[8];
	m_p 	= m_con_states[9];
	m_q 	= m_con_states[10];
	m_r 	= m_con_states[11];
}

void NMSPC::Vehicle_Body::output (d_vec &outputs, Eigen::Matrix3d &DCM) {
	outputs[0]  = m_xe_x;
	outputs[1]  = m_xe_y;
	outputs[2]  = m_xe_z;
	outputs[3]  = m_vb_x;
	outputs[4]  = m_vb_y;
	outputs[5]  = m_vb_z;
	outputs[6]  = m_phai;
	outputs[7]  = m_theta;
	outputs[8]  = m_psi;
	outputs[9]  = m_p;
	outputs[10] = m_q;
	outputs[11] = m_r;
	outputs[12] = m_ve_x;
	outputs[13] = m_ve_y;
	outputs[14] = m_ve_z;
	DCM			= m_DCM;
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

	m_Rbar(0) = m_Rbar_x;
	m_Rbar(1) = m_Rbar_y;
	m_Rbar(2) = m_Rbar_z;
	m_Xbar(0) = m_Xbar_a;
	m_Xbar(1) = m_Xbar_b;
	m_Xbar(2) = m_Xbar_h;
	m_Wbar(0) = m_Wbar_fl;
	m_Wbar(1) = m_Wbar_fr;
	m_Wbar(2) = m_Wbar_rl;
	m_Wbar(3) = m_Wbar_rr;
	m_HPbar(0,0) = m_HPbar_fl_x;
	m_HPbar(0,1) = m_HPbar_fr_x;
	m_HPbar(0,2) = m_HPbar_rl_x;
	m_HPbar(0,3) = m_HPbar_rr_x;
	m_HPbar(1,0) = m_HPbar_fl_y;
	m_HPbar(1,1) = m_HPbar_fr_y;
	m_HPbar(1,2) = m_HPbar_rl_y;
	m_HPbar(1,3) = m_HPbar_rr_y;
	m_HPbar(2,0) = m_HPbar_fl_z;
	m_HPbar(2,1) = m_HPbar_fr_z;
	m_HPbar(2,2) = m_HPbar_rl_z;
	m_HPbar(2,3) = m_HPbar_rr_z;
	m_Ibar(0,0) = m_Ibar_xx;
	m_Ibar(0,1) = m_Ibar_xy;
	m_Ibar(0,2) = m_Ibar_xz;
	m_Ibar(1,0) = m_Ibar_yx;
	m_Ibar(1,1) = m_Ibar_yy;
	m_Ibar(1,2) = m_Ibar_yz;
	m_Ibar(2,0) = m_Ibar_zx;
	m_Ibar(2,1) = m_Ibar_zy;
	m_Ibar(2,2) = m_Ibar_zz;
}

void NMSPC::Vehicle_Body::calculate_gravity() {
	Eigen::Vector3d mg(0.0, 0.0, m_Mbar * g);
	Eigen::Vector3d Fg = m_DCM * mg;
	m_Fg_x = Fg(0);
	m_Fg_y = Fg(1);
	m_Fg_z = Fg(2);
}

void NMSPC::Vehicle_Body::calculate_aero_drag() {
	Eigen::Vector3d vb(m_vb_x, m_vb_y, m_vb_z);
	Eigen::Vector3d winde(m_Wx, m_Wy, m_Wz);
	Eigen::Vector3d windb = m_DCM * winde;
	m_Fd_x = 0.0;
	m_Fd_y = 0.0;
	m_Fd_z = 0.0;
	m_Md_x = 0.0;
	m_Md_y = 0.0;
	m_Md_z = 0.0;
}
