#ifndef TIRE_FIALA_HPP
#define TIRE_FIALA_HPP
#include "common.hpp"

class Tire_Fiala {
public:
	static const int m_params_num = 19;	//amount of parameters
	static const int m_inputs_num = 9;	//amount of inputs
	static const int m_con_states_num = 3;	//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_outputs_num = 7;	//amount of outputs


	Tire_Fiala (double unloaded_radius, double Lrelx, double Lrely, \
	double alpha_min, double alpha_max, double mu_min, \
	double mu_max, double aMy, double bMy, double cMy, \
	double alphaMy, double betaMy, double Fz_min, double Fz_max, \
	double cKappa, double cAlpha, double bMz, double width, \
	double cGamma, double init_kappa, double init_alpha, \
	double init_Mroll, double t) : \
	m_unloaded_radius(unloaded_radius), m_Lrelx(Lrelx), \
	m_Lrely(Lrely), m_alpha_min(alpha_min), m_alpha_max(alpha_max), \
	m_mu_min(mu_min), m_mu_max(mu_max), m_aMy(aMy), m_bMy(bMy), \
	m_cMy(cMy), m_alphaMy(alphaMy), m_betaMy(betaMy), m_Fz_min(Fz_min), \
	m_Fz_max(Fz_max), m_cKappa(cKappa), m_cAlpha(cAlpha), m_bMz(bMz), \
	m_width(width), m_cGamma(cGamma), \
	m_kappa(init_kappa), m_alpha_prime(init_alpha), \
	m_Mroll(init_Mroll), m_t(t) \
	{
		m_con_states[0] = init_kappa;
		m_con_states[1] = init_alpha;
		m_con_states[2] = init_Mroll;
	}
	
	Tire_Fiala (const d_vec &params, const d_vec &init_states, \
	const double &t);
	
	//functionalities of preupdate:
	//1, to update continous state vector to be current state
	//2, to grasp inputs and system time
	//3, to calculate middle ravriables and outputs
	void preupdate (const d_vec &u, const double &t);
	//functionalities of calculate_accs:
	//just to calculate rhs of a wheel system 
	//(maybe acc is not a good name)
	void calculate_derivatives(double &drv_kappa, \
	double &drv_alpha_prime, double &drv_Mroll);
	//functionalities of postupdate:
	//to update discrete state which is based on continuous update
	void postupdate ();
	//functionalities of output:
	//to update output based on updated system states
	void output (d_vec &outputs);

private:
	//parameters	
	double m_unloaded_radius, m_Lrelx, m_Lrely, m_alpha_min, m_alpha_max,\
	m_mu_min, m_mu_max, m_aMy, m_bMy, m_cMy, m_alphaMy, m_betaMy, \
	m_Fz_min, m_Fz_max, m_cKappa, m_cAlpha, m_bMz, m_width, \
	m_cGamma;
	//inputs
	double m_omega = NaN;
	double m_vx = NaN;
	double m_vy = NaN;
	double m_susF = NaN;
	double m_gamma = NaN;
	double m_psidot = NaN;
	double m_tirePrs = NaN;
	double m_Gnd = NaN;
	double m_scale = NaN;
	//continuous states
	double m_kappa, m_alpha_prime, m_Mroll;
	//outputs
	double m_Fx = NaN;
	double m_Fy = NaN;
	double m_Fz = NaN;
	double m_Mx = NaN;
	//double m_Mroll = NaN;
	double m_Mz = NaN;
	double m_Re = NaN;
	//middle variables
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
	
public:
	double m_t;	//wheel time
	//for states communication with class System
	d_vec m_con_states = d_vec(m_con_states_num, NaN);
};

#endif //TIRE_FIALA_HPP
