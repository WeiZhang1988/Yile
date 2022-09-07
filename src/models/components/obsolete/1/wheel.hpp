#ifndef WHEEL_HPP
#define WHEEL_HPP
#include "common.hpp"

class Wheel {
public:
	static const int m_params_num = 7;	//amount of parameters
	static const int m_inputs_num = 5;	//amount of inputs
	static const int m_con_states_num = 1;	//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = 2;	//amount of discrete states;
	static const int m_outputs_num = 1;	//amount of outputs



	Wheel (double IYY, double br, double disk_abore, \
	double num_pads, double Rm, double mu_kinetic, \
	double mu_static, double init_omega, bool init_locked_flag, \
	double t) :
	m_IYY(IYY), m_br(br), m_disk_abore(disk_abore), \
	m_num_pads(num_pads), m_Rm(Rm), \
	m_mu_kinetic(mu_kinetic), m_mu_static(mu_static), \
	m_unlocked_omega(init_omega), m_unlocked_omega_pre(init_omega), \
	m_locked_flag(init_locked_flag), m_t(t) \
	{
		//feed vector of continuous states
		m_con_states[0] = init_omega;
		//initialize kinematics outputs
		m_output_omega = init_omega;
	}
	
	Wheel (const d_vec &params, const d_vec &init_states, const bool \
	&init_locked_flag, const double &t);

	//functionalities of preupdate:
	//1, to update continous state vector to be current state
	//2, to grasp inputs and system time
	//3, to calculate middle ravriables and outputs
	void preupdate (const d_vec &u, const double &t);
	//functionalities of calculate_accs:
	//just to calculate rhs of a wheel system 
	//(maybe acc is not a good name)
	void calculate_derivatives(double &drv_omega);
	//functionalities of postupdate:
	//to update discrete state which is based on continuous update
	void postupdate ();
	//functionalities of output:
	//to update output based on updated system states
	void output (d_vec &outputs);

private:
	//parameters
	double m_IYY, m_br, m_disk_abore, m_num_pads, \
	m_Rm, m_mu_kinetic, m_mu_static;
	//inputs
	double m_Brake_Pressure = NaN;
	double m_Axl_trq = NaN;
	double m_M_y = NaN;
	double m_F_x = NaN;
	double m_Re = NaN;
	//continuous states
	double m_unlocked_omega;
	//discrete states
	double m_unlocked_omega_pre;
	bool m_locked_flag;
	//outputs
	double m_output_omega = NaN;
	//built in table
	const int m_truth_table[8] = {0,1,0,0,1,1,1,0};	//for lock logic
	//middle variables
	double m_Tout = NaN;
	double m_Tfmaxk = NaN;
	double m_Tfmaxs = NaN;
	

public:
	double m_t;	//wheel time
	//for states communication with class System
	d_vec m_con_states = d_vec(m_con_states_num, NaN);
};

#endif //WHEELS_HPP
