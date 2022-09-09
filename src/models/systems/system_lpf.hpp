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
#ifndef SYSTEM_LPF_HPP
#define SYSTEM_LPF_HPP
#include "components/common.hpp"
#include "components/low_pass_filter.hpp"
#include "interfaces/interface_lpf.hpp"

namespace NMSPC{
class Sys_LPF {
public:
    static const int m_pv_inters_num = 0;
	static const int m_fm_inters_num = 0;

	static const int m_pv_inputs_num = Low_Pass_Filter::m_pv_inputs_num - m_pv_inters_num;
	static const int m_fm_inputs_num = Low_Pass_Filter::m_fm_inputs_num - m_fm_inters_num;
	static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;
    static const int m_external_inputs_num = m_inputs_num - m_pv_inters_num - m_fm_inters_num;
	static const int m_con_states_num = Low_Pass_Filter::m_con_states_num;						
	static const int m_derivatives_num = Low_Pass_Filter::m_derivatives_num;
	static const int m_dis_states_num = Low_Pass_Filter::m_dis_states_num;
	static const int m_pv_outputs_num = Low_Pass_Filter::m_pv_outputs_num;
	static const int m_fm_outputs_num = Low_Pass_Filter::m_fm_outputs_num;
	static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;
	
	void add_lpf(std::shared_ptr<Low_Pass_Filter> sptr_lpf) \
    {m_sptr_lpf = sptr_lpf;}
	
	void push_con_states(d_vec &con_states);
	void pull_con_states(const d_vec &con_states);
	void pull_external_inputs(const d_vec &inputs);
	void update_pv();
	void update_fm();
	void update_drv();
	void operator() (const d_vec &x, d_vec &dxdt, const double &t);
	

private:
	std::shared_ptr<Low_Pass_Filter> m_sptr_lpf;

	//inputs
	//continuous states
	d_vec m_lpf_con_states = d_vec(Low_Pass_Filter::m_con_states_num,NaN);
	//continuous states derivatives
	d_vec m_lpf_drvs = d_vec(Low_Pass_Filter::m_derivatives_num,NaN);
	d_vec m_drvs = d_vec(m_derivatives_num,NaN);
	//outputs
	
public:
	//continuous states
	d_vec m_con_states = d_vec(m_con_states_num,NaN);
	//interface 
    //stop here
	Int_LPF interface = Int_LPF(m_external_inputs_num);
};

}	//end of name space
#endif //SYSTEM_LPF_HPP
