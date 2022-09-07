#ifndef WHEEL_DISK_TIRE_FIALA_HPP
#define WHEEL_DISK_TIRE_FIALA_HPP
#include "common.hpp"

namespace NMSPC{
class Wheel_Disk_Tire_Fiala {
public:
	static const int m_params_num = 26;						//amount of parameters
	static const int m_inputs_num = 10;						//amount of inputs
	static const int m_con_states_num = 4;					//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;	//amount of derivatives;
	static const int m_dis_states_num = 2;					//amount of discrete states;
	static const int m_outputs_num = 8;						//amount of outputs

	Wheel_Disk_Tire_Fiala (\
	//wheel
	double IYY=0.74, double br=1e-3, \
	double disk_abore=0.05, double num_pads=2.0, double Rm=0.177, \
	double mu_kinetic=0.2, double mu_static=0.3, \
	//tire
	double unloaded_radius=0.31, double Lrelx=0.05, double Lrely=0.15, \
	double alpha_min=-1.5708, double alpha_max=1.5708, double mu_min=0.8, \
	double mu_max=1.0, double aMy=8e-4, double bMy=1e-3, \
	double cMy=1.6e-4, double alphaMy=-3e-3, double betaMy=0.97, \
	double Fz_min=100.0, double Fz_max=1e4, double cKappa=1e7, \
	double cAlpha=4.5e4, double bMz=0.0, double width=0.209, \
	double cGamma=1e3, \
	//wheel
	double init_omega=0.0, bool init_locked_flag=false, \
	//tire
	double init_kappa=0.0, double init_alpha=0.0, \
	double init_Mroll=0.0, \
	double t=0.0) :
	//wheel
	m_IYY(IYY), m_br(br), m_disk_abore(disk_abore), \
	m_num_pads(num_pads), m_Rm(Rm), \
	m_mu_kinetic(mu_kinetic), m_mu_static(mu_static), \
	//tire
	m_unloaded_radius(unloaded_radius), m_Lrelx(Lrelx), \
	m_Lrely(Lrely), m_alpha_min(alpha_min), m_alpha_max(alpha_max), \
	m_mu_min(mu_min), m_mu_max(mu_max), m_aMy(aMy), m_bMy(bMy), \
	m_cMy(cMy), m_alphaMy(alphaMy), m_betaMy(betaMy), m_Fz_min(Fz_min), \
	m_Fz_max(Fz_max), m_cKappa(cKappa), m_cAlpha(cAlpha), m_bMz(bMz), \
	m_width(width), m_cGamma(cGamma), \
	//wheel
	m_unlocked_omega(init_omega), m_unlocked_omega_pre(init_omega), \
	m_locked_flag(init_locked_flag), \
	//tire
	m_kappa(init_kappa), m_alpha_prime(init_alpha), \
	m_Mroll(init_Mroll), \
	m_t(t) \
	{
		//feed vector of continuous states
		//wheel
		m_con_states[0] = init_omega;
		//tire
		m_con_states[1] = init_kappa;
		m_con_states[2] = init_alpha;
		m_con_states[3] = init_Mroll;
		//initialize kinematics outputs
		m_output_omega = init_omega;
	}
	
	Wheel_Disk_Tire_Fiala (const d_vec &params, \
	const d_vec &init_states, const bool &init_locked_flag, \
	const double &t);

	//functionalities of preupdate:
	//1, to update continous state vector to be current state
	//2, to grasp inputs and system time
	//3, to calculate middle ravriables and outputs
	void preupdate (const d_vec &u, const double &t);
	//functionalities of calculate_accs:
	//just to calculate rhs of a wheel system 
	//(maybe acc is not a good name)
	void calculate_derivatives(double &drv_omega, \
	double &drv_kappa, double &drv_alpha_prime, double &drv_Mroll);
	//functionalities of postupdate:
	//to update discrete state which is based on continuous update
	void postupdate ();
	//functionalities of output:
	//to update output based on updated system states
	void output (d_vec &outputs);

private:
	//built in table
	const int m_truth_table[8] = {0,1,0,0,1,1,1,0};	//for lock logic
	//parameters
	//wheel
	double m_IYY, m_br, m_disk_abore, m_num_pads, \
	m_Rm, m_mu_kinetic, m_mu_static;
	//tire
	double m_unloaded_radius, m_Lrelx, m_Lrely, m_alpha_min, \
	m_alpha_max, m_mu_min, m_mu_max, m_aMy, m_bMy, m_cMy, \
	m_alphaMy, m_betaMy, m_Fz_min, m_Fz_max, m_cKappa, m_cAlpha, \
	m_bMz, m_width, m_cGamma;
	//inputs
	//wheel
	double m_Brake_Pressure = NaN;
	double m_Axl_trq = NaN;
	//tire
	double m_vx = NaN;
	double m_vy = NaN;
	double m_susF = NaN;
	double m_gamma = NaN;
	double m_psidot = NaN;
	double m_tirePrs = NaN;
	double m_Gnd = NaN;
	double m_scale = NaN;
	//continuous states
	double m_unlocked_omega, m_kappa, m_alpha_prime, m_Mroll;
	//discrete states
	double m_unlocked_omega_pre;
	bool m_locked_flag;
	//middle variables
	//wheel
	double m_M_y = NaN;
	double m_F_x = NaN;
	double m_Re = NaN;
	double m_Tout = NaN;
	double m_Tfmaxk = NaN;
	double m_Tfmaxs = NaN;
	//tire
	double m_z = NaN;
	double m_z_dot = NaN;
	double m_sat_Fz = NaN;
	double m_rhoz = NaN;
	//---
	double m_My = NaN;
	//---
	double m_mu = NaN;
	double m_kappa_critical = NaN;
	double m_Fx1 = NaN;
	double m_Fx2 = NaN;
	double m_Fx_stick = NaN;
	double m_Fx_slide = NaN;
	//---
	double m_alpha = NaN;
	double m_tan_alpha = NaN;
	double m_alpha_critical = NaN;
	double m_H = NaN;
	double m_Mz_stick = NaN;
	double m_Fy_stick = NaN;
	double m_Mz_slide = NaN;
	double m_Fy_slide = NaN;
	//outputs
	//wheel
	double m_output_omega = NaN;
	//tire
	double m_Fx = NaN;
	double m_Fy = NaN;
	double m_Fz = NaN;
	double m_Mx = NaN;
	//double m_Mroll = NaN;	also defined as continuous state
	double m_Mz = NaN;
	//double m_Re = NaN;	also defined as middle variables
public:
	double m_t;	//component time
	//for states communication with class System
	d_vec m_con_states = d_vec(m_con_states_num, NaN);
};

}	//end of name space
#endif //WHEEL_DISKDRUM_TIRE_FIALA_HPP
