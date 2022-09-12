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
#ifndef LOW_PASS_FILTER_HPP
#define LOW_PASS_FILTER_HPP
#include "common.hpp"

namespace NMSPC{
class Low_Pass_Filter {
public:
	static const int m_params_num = 1;										//amount of parameters
	static const int m_pv_inputs_num = 0;									//amount of pv inputs
	static const int m_fm_inputs_num = 12;									//amount of fm inputs
	static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;		//amount of inputs
	static const int m_con_states_num = 12;									//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;					//amount of derivatives;
	static const int m_dis_states_num = 0;									//amount of discrete states;
	static const int m_pv_outputs_num = 12;									//amount of pv outputs
	static const int m_fm_outputs_num = 0;									//amount of fm outputs
	static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;	//amount of outputs

	Low_Pass_Filter (double wc=100.0*2.0*pi, \
	double init_toe_fl=0.0, double init_toe_fr=0.0,\
	double init_toe_rl=0.0, double init_toe_rr=0.0,\
	double init_camber_fl=0.0, double init_camber_fr=0.0,\
	double init_camber_rl=0.0, double init_camber_rr=0.0,\
	double init_Fz_fl=0.0, double init_Fz_fr=0.0,\
	double init_Fz_rl=0.0, double init_Fz_rr=0.0
	) :
	m_wc(wc), \
	m_toe_fl(init_toe_fl), m_toe_fr(init_toe_fr), \
	m_toe_rl(init_toe_rl), m_toe_rr(init_toe_rr), \
	m_camber_fl(init_camber_fl), m_camber_fr(init_camber_fr), \
	m_camber_rl(init_camber_rl), m_camber_rr(init_camber_rr), \
	m_Fz_fl(init_Fz_fl), m_Fz_fr(init_Fz_fr), \
	m_Fz_rl(init_Fz_rl), m_Fz_rr(init_Fz_rr) {};

	void push_con_states (d_vec &con_states);
	
	void pull_con_states (d_vec &con_states);
	void update_pv (const d_vec &inputs, d_vec &outputs);
	void update_fm (const d_vec &inputs, d_vec &outputs);
	void update_drv (d_vec &outputs);
private:
	//parameters
	double m_wc;
	//inputs
	double m_input_Sus_toe_fl = NaN;
	double m_input_Sus_toe_fr = NaN;
	double m_input_Sus_toe_rl = NaN;
	double m_input_Sus_toe_rr = NaN;
	double m_input_Sus_camber_fl = NaN;
	double m_input_Sus_camber_fr = NaN;
	double m_input_Sus_camber_rl = NaN;
	double m_input_Sus_camber_rr = NaN;
	double m_input_Sus_Fz_fl = NaN;
	double m_input_Sus_Fz_fr = NaN;
	double m_input_Sus_Fz_rl = NaN;
	double m_input_Sus_Fz_rr = NaN;
	//continuous states
	double m_toe_fl = NaN;
	double m_toe_fr = NaN;
	double m_toe_rl = NaN;
	double m_toe_rr = NaN;
	double m_camber_fl = NaN;
	double m_camber_fr = NaN;
	double m_camber_rl = NaN;
	double m_camber_rr = NaN;
	double m_Fz_fl = NaN;
	double m_Fz_fr = NaN;
	double m_Fz_rl = NaN;
	double m_Fz_rr = NaN;
	//continuous states derivative
	double m_drv_toe_fl = NaN;
	double m_drv_toe_fr = NaN;
	double m_drv_toe_rl = NaN;
	double m_drv_toe_rr = NaN;
	double m_drv_camber_fl = NaN;
	double m_drv_camber_fr = NaN;
	double m_drv_camber_rl = NaN;
	double m_drv_camber_rr = NaN;
	double m_drv_Fz_fl = NaN;
	double m_drv_Fz_fr = NaN;
	double m_drv_Fz_rl = NaN;
	double m_drv_Fz_rr = NaN;
	//outputs
	double m_output_LPF_toe_fl = NaN;
	double m_output_LPF_toe_fr = NaN;
	double m_output_LPF_toe_rl = NaN;
	double m_output_LPF_toe_rr = NaN;
	double m_output_LPF_camber_fl = NaN;
	double m_output_LPF_camber_fr = NaN;
	double m_output_LPF_camber_rl = NaN;
	double m_output_LPF_camber_rr = NaN;
	double m_output_LPF_Fz_fl = NaN;
	double m_output_LPF_Fz_fr = NaN;
	double m_output_LPF_Fz_rl = NaN;
	double m_output_LPF_Fz_rr = NaN;
};

}	//end of name space
#endif //LOW_PASS_FILTER_HPP
