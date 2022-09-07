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
#ifndef INTERFACE_WHEEL_DISK_TIRE_FIALA_HPP
#define INTERFACE_WHEEL_DISK_TIRE_FIALA_HPP
#include "components/common.hpp"
#include "components/wheel_disk.hpp"
#include "components/tire_fiala.hpp"

namespace NMSPC{
class Int_Whl_Disk_Tir_Fiala {
public:
	Int_Whl_Disk_Tir_Fiala(int ext_inputs_num);
	
	d_vec m_external_inputs;	//Gnd vx vy gamma(camber) psidot AxlTrq BrkPrs FzSus Scale TirPrs Tamb 

	d_vec  m_whl_pv_inputs = d_vec(Wheel_Disk::m_pv_inputs_num, NaN);
	d_vec  m_whl_fm_inputs = d_vec(Wheel_Disk::m_fm_inputs_num, NaN);
	d_vec  m_tir_pv_inputs = d_vec(Tire_Fiala::m_pv_inputs_num, NaN);
	d_vec  m_tir_fm_inputs = d_vec(Tire_Fiala::m_fm_inputs_num, NaN);
	
	d_vec  m_whl_pv_outputs = d_vec(Wheel_Disk::m_pv_outputs_num, NaN);
	d_vec  m_whl_fm_outputs = d_vec(Wheel_Disk::m_fm_outputs_num, NaN);
	d_vec  m_tir_pv_outputs = d_vec(Tire_Fiala::m_pv_outputs_num, NaN);
	d_vec  m_tir_fm_outputs = d_vec(Tire_Fiala::m_fm_outputs_num, NaN);
	
	void map_whl_pv();
	void map_tir_pv();
	void map_whl_fm();
	void map_tir_fm();
};

}//end of name space 
#endif //INTERFACE_WHEEL_DISK_TIRE_FIALA_HPP
