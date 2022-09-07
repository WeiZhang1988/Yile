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
#include "interface_vehicle_body.hpp"

NMSPC::Int_Vehicle_Body::Int_Vehicle_Body(int ext_inputs_num) {
	m_external_inputs = d_vec(ext_inputs_num, NaN);
}

void NMSPC::Int_Vehicle_Body::map_vhl_bdy_pv() {
	
}

void NMSPC::Int_Vehicle_Body::map_vhl_bdy_fm() {
	std::copy(m_external_inputs.begin(),m_external_inputs.end(),m_vhl_bdy_fm_inputs.begin());
}

