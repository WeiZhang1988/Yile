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
#ifndef INTERFACE_TIRE_4FIALA_HPP
#define INTERFACE_TIRE_4FIALA_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_tir_4fiala.hpp"

namespace NMSPC{
class Int_Tir_4Fiala {
public:
	Int_Tir_4Fiala(int ext_inputs_num);
	
	d_vec m_external_inputs;	//omega vx vy gamma(camber) psidot FzSus Scale TirPrs Tamb 
	
    d_vec  m_sub_tir_4fiala_pv_inputs = d_vec(Subsys_Tire_4Fiala::m_pv_inputs_num, NaN);
    d_vec  m_sub_tir_4fiala_fm_inputs = d_vec(Subsys_Tire_4Fiala::m_fm_inputs_num, NaN);
    
    d_vec  m_sub_tir_4fiala_pv_outputs = d_vec(Subsys_Tire_4Fiala::m_pv_outputs_num, NaN);
    d_vec  m_sub_tir_4fiala_fm_outputs = d_vec(Subsys_Tire_4Fiala::m_fm_outputs_num, NaN);

	void map_sub_tir_4fiala_pv();
	void map_sub_tir_4fiala_fm();
};

}//end of name space 
#endif //INTERFACE_TIRE_4FIALA_HPP
