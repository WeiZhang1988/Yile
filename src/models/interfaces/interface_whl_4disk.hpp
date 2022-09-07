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
#ifndef INTERFACE_WHEEL_4DISK_HPP
#define INTERFACE_WHEEL_4DISK_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_whl_4disk.hpp"

namespace NMSPC{
class Int_Whl_4Disk {
public:
	Int_Whl_4Disk(int ext_inputs_num);
	
	d_vec m_external_inputs;	// 
	
    d_vec  m_sub_whl_4disk_pv_inputs = d_vec(Subsys_Wheel_4Disk::m_pv_inputs_num, NaN);
    d_vec  m_sub_whl_4disk_fm_inputs = d_vec(Subsys_Wheel_4Disk::m_fm_inputs_num, NaN);
    
    d_vec  m_sub_whl_4disk_pv_outputs = d_vec(Subsys_Wheel_4Disk::m_pv_outputs_num, NaN);
    d_vec  m_sub_whl_4disk_fm_outputs = d_vec(Subsys_Wheel_4Disk::m_fm_outputs_num, NaN);

	void map_sub_whl_4disk_pv();
	void map_sub_whl_4disk_fm();
};

}//end of name space 
#endif //INTERFACE_WHEEL_4DISK_HPP
