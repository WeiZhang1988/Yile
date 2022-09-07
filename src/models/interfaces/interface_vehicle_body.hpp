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
#ifndef INTERFACE_VEHICLE_BODY_HPP
#define INTERFACE_VEHICLE_BODY_HPP
#include "components/common.hpp"
#include "components/vehicle_body.hpp"

namespace NMSPC{
class Int_Vehicle_Body {
public:
	Int_Vehicle_Body(int ext_inputs_num);
	
	d_vec m_external_inputs;	
	
    d_vec  m_vhl_bdy_pv_inputs = d_vec(Vehicle_Body::m_pv_inputs_num, NaN);
    d_vec  m_vhl_bdy_fm_inputs = d_vec(Vehicle_Body::m_fm_inputs_num, NaN);
    
    d_vec  m_vhl_bdy_pv_outputs = d_vec(Vehicle_Body::m_pv_outputs_num, NaN);
    d_vec  m_vhl_bdy_fm_outputs = d_vec(Vehicle_Body::m_fm_outputs_num, NaN);

	void map_vhl_bdy_pv();
	void map_vhl_bdy_fm();
};

}//end of name space 
#endif //INTERFACE_VEHICLE_BODY_HPP
