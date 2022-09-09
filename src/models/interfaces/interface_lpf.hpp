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
#ifndef INTERFACE_LPF_HPP
#define INTERFACE_LPF_HPP
#include "components/common.hpp"
#include "components/low_pass_filter.hpp"

namespace NMSPC{
class Int_LPF {
public:
	Int_LPF(int ext_inputs_num);
	
	d_vec m_external_inputs;	// 
	
    d_vec  m_lpf_pv_inputs = d_vec(Low_Pass_Filter::m_pv_inputs_num, NaN);
    d_vec  m_lpf_fm_inputs = d_vec(Low_Pass_Filter::m_fm_inputs_num, NaN);
    
    d_vec  m_lpf_pv_outputs = d_vec(Low_Pass_Filter::m_pv_outputs_num, NaN);
    d_vec  m_lpf_fm_outputs = d_vec(Low_Pass_Filter::m_fm_outputs_num, NaN);

	void map_lpf_pv();
	void map_lpf_fm();
};

}//end of name space 
#endif //INTERFACE_LPF_HPP
