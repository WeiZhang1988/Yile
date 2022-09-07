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
#ifndef INTERFACE_SUS_2IND_HPP
#define INTERFACE_SUS_2IND_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_sus_2ind.hpp"

namespace NMSPC{
class Int_Sus_2Ind {
public:
	Int_Sus_2Ind(int ext_inputs_num);
	
	d_vec m_external_inputs;	
	
    d_vec  m_sub_sus_2ind_pv_inputs = d_vec(Subsys_Sus_2Ind::m_pv_inputs_num, NaN);
    d_vec  m_sub_sus_2ind_fm_inputs = d_vec(Subsys_Sus_2Ind::m_fm_inputs_num, NaN);
    
    d_vec  m_sub_sus_2ind_pv_outputs = d_vec(Subsys_Sus_2Ind::m_pv_outputs_num, NaN);
    d_vec  m_sub_sus_2ind_fm_outputs = d_vec(Subsys_Sus_2Ind::m_fm_outputs_num, NaN);

	void map_sub_sus_2ind_pv();
	void map_sub_sus_2ind_fm();
};

}//end of name space 
#endif //INTERFACE_SUS_2IND_HPP
