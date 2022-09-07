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
#ifndef INTERFACE_ENTIRE_DISK_FIALA_HPP
#define INTERFACE_ENTIRE_DISK_FIALA_HPP
#include "components/common.hpp"
#include "components/vehicle_body.hpp"
#include "subsystems/subsystem_sus_2ind.hpp"
#include "subsystems/subsystem_whl_4disk.hpp"
#include "subsystems/subsystem_tir_4fiala.hpp"

namespace NMSPC{
class Int_Entire_Disk_Fiala {
public:
	Int_Entire_Disk_Fiala(int ext_inputs_num);
	
	d_vec m_external_inputs;

	d_vec  m_vhl_bdy_pv_inputs = d_vec(Vehicle_Body::m_pv_inputs_num, NaN);
    d_vec  m_vhl_bdy_pv_outputs = d_vec(Vehicle_Body::m_pv_outputs_num, NaN);
    d_vec  m_sub_whl_4disk_pv_inputs = d_vec(Subsys_Wheel_4Disk::m_pv_inputs_num, NaN);
    d_vec  m_sub_whl_4disk_pv_outputs = d_vec(Subsys_Wheel_4Disk::m_pv_outputs_num, NaN);
    d_vec  m_sub_sus_2ind_pv_inputs = d_vec(Subsys_Sus_2Ind::m_pv_inputs_num, NaN);
    d_vec  m_sub_sus_2ind_pv_outputs = d_vec(Subsys_Sus_2Ind::m_pv_outputs_num, NaN);
    d_vec  m_sub_tir_4fiala_pv_inputs = d_vec(Subsys_Tire_4Fiala::m_pv_inputs_num, NaN);
    d_vec  m_sub_tir_4fiala_pv_outputs = d_vec(Subsys_Tire_4Fiala::m_pv_outputs_num, NaN);
    d_vec  m_sub_tir_4fiala_fm_inputs = d_vec(Subsys_Tire_4Fiala::m_fm_inputs_num, NaN);
    d_vec  m_sub_tir_4fiala_fm_outputs = d_vec(Subsys_Tire_4Fiala::m_fm_outputs_num, NaN);
    d_vec  m_sub_sus_2ind_fm_inputs = d_vec(Subsys_Sus_2Ind::m_fm_inputs_num, NaN);
    d_vec  m_sub_sus_2ind_fm_outputs = d_vec(Subsys_Sus_2Ind::m_fm_outputs_num, NaN);
    d_vec  m_sub_whl_4disk_fm_inputs = d_vec(Subsys_Wheel_4Disk::m_fm_inputs_num, NaN);
    d_vec  m_sub_whl_4disk_fm_outputs = d_vec(Subsys_Wheel_4Disk::m_fm_outputs_num, NaN);
    d_vec  m_vhl_bdy_fm_inputs = d_vec(Vehicle_Body::m_fm_inputs_num, NaN);
    d_vec  m_vhl_bdy_fm_outputs = d_vec(Vehicle_Body::m_fm_outputs_num, NaN);

	void map_vhl_bdy_pv();
    void map_sub_whl_4disk_pv();
    void map_sub_sus_2ind_pv();
    void map_sub_tir_4fiala_pv();
    void map_sub_tir_4fiala_fm();
    void map_sub_sus_2ind_fm();
	void map_sub_whl_4disk_fm();
	void map_vhl_bdy_fm();
};

}//end of name space 
#endif //INTERFACE_ENTIRE_DISK_FIALA_HPP
