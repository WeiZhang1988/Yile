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
#ifndef WHEEL_DISK_HPP
#define WHEEL_DISK_HPP
#include "common.hpp"

namespace NMSPC{
typedef struct {
	double Gnd_Pz;
	double Axl_Trq;
	double Brk_Prs;
	double Tir_Fx;
	double Tir_My;
	double Sus_Fz;	//Fext
	double Tir_Fz;	//Gnd Fz
}Wheel_Inputs;

typedef struct {
	double Tir_omega;
	double Tir_Re;
	double Tir_rhoz;
	double Tir_Brk_Trq;
}Wheel_Outputs;

typedef struct {
	double unlocked_omega;
	double Tir_Pz;
	double Tir_Vz;
	double Sus_lpf_Fz = 0.0;
}Wheel_Continuous_States;

typedef struct {
	double unlocked_omega_pre;
	bool   locked_flag;
}Wheel_Discrete_States;

typedef struct {
	double unlocked_omega;
	double Tir_Pz;
	double Tir_Vz;
	double Sus_lpf_Fz = 0.0;
}Wheel_States_Derivative;

typedef struct {
	double unloaded_radius; 
	double IYY;
	double br;
	double disk_abore;
	double num_pads;
	double Rm;
	double mu_kinetic; 
	double mu_static;
}Wheel_Parameter;

class Wheel_Disk {
public:
	static const int m_params_num = 8;										//amount of parameters
	static const int m_pv_inputs_num = 1;									//amount of pv inputs
	static const int m_fm_inputs_num = 6;									//amount of fm inputs
	static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;		//amount of inputs
	static const int m_con_states_num = 4;									//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;					//amount of derivatives;
	static const int m_dis_states_num = 2;									//amount of discrete states;
	static const int m_pv_outputs_num = 5;									//amount of pv outputs
	static const int m_fm_outputs_num = 1;									//amount of fm outputs
	static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;	//amount of outputs

	// Wheel_Disk (\
	// double unloaded_radius=0.31, double IYY=0.74, double br=1e-3, \
	// double disk_abore=0.05, double num_pads=2.0, double Rm=0.177, \
	// double mu_kinetic=0.2, double mu_static=0.3, \
	// double init_omega=0.0, double init_Pz=0.0, double init_Vz=0.0, \
	// bool init_locked_flag=false) :
	// m_unloaded_radius(unloaded_radius),m_IYY(IYY), m_br(br), \
	// m_disk_abore(disk_abore), m_num_pads(num_pads), m_Rm(Rm), \
	// m_mu_kinetic(mu_kinetic), m_mu_static(mu_static), \
	// m_unlocked_omega(init_omega), m_unlocked_omega_pre(init_omega), \
	// m_Tir_Pz(init_Pz), m_Tir_Vz(init_Vz), \
	// m_locked_flag(init_locked_flag) {};

	Wheel_Disk (Wheel_Parameter P, Wheel_Continuous_States X, Wheel_Discrete_States D) :
	m_Wheel_P(P),
	m_Wheel_X(X),
	m_Wheel_D(D)
	{
		//nothing
	};


	void push_con_states (d_vec &con_states);
	void pull_con_states (const d_vec &con_states);
	void update_pv (const d_vec &inputs, d_vec &outputs);
	void update_fm (const d_vec &inputs, d_vec &outputs);
	void update_drv (d_vec &outputs);
private:
	//built in table
	const int m_truth_table[8] = {0,1,0,0,1,1,1,0};	//for lock logic
	//parameters
	Wheel_Parameter m_Wheel_P;

	//inputs
	Wheel_Inputs m_Wheel_I;

	//continuous states
	Wheel_Continuous_States m_Wheel_X;

	//continuous states derivative
	Wheel_States_Derivative m_Wheel_XDot;

	//discrete states
	Wheel_Discrete_States m_Wheel_D;

	//middle variables
	double m_Tout = NaN;
	double m_Tfmaxk = NaN;
	double m_Tfmaxs = NaN;
	//outputs
	Wheel_Outputs m_Wheel_O;
};

}	//end of name space
#endif //WHEEL_DISKDRUM_TIRE_FIALA_HPP
