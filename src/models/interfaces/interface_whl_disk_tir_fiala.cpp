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
#include "interface_whl_disk_tir_fiala.hpp"

NMSPC::Int_Whl_Disk_Tir_Fiala::Int_Whl_Disk_Tir_Fiala(int ext_inputs_num) {
	m_external_inputs = d_vec(ext_inputs_num, NaN);
}

void NMSPC::Int_Whl_Disk_Tir_Fiala::map_whl_pv() {
	m_whl_pv_inputs[0] = m_external_inputs[0];
}

void NMSPC::Int_Whl_Disk_Tir_Fiala::map_tir_pv() {
	m_tir_pv_inputs[0] = m_whl_pv_outputs[0];
	m_tir_pv_inputs[1] = m_external_inputs[1];
	m_tir_pv_inputs[2] = m_external_inputs[2];
	m_tir_pv_inputs[3] = m_external_inputs[3];
	m_tir_pv_inputs[4] = m_external_inputs[4];
	m_tir_pv_inputs[5] = m_whl_pv_outputs[1];
	m_tir_pv_inputs[6] = m_whl_pv_outputs[4];
}

void NMSPC::Int_Whl_Disk_Tir_Fiala::map_whl_fm() {
	m_whl_fm_inputs[0] = m_external_inputs[5]; 
	m_whl_fm_inputs[1] = m_external_inputs[6]; 
	m_whl_fm_inputs[2] = m_tir_fm_outputs[0]; 
	m_whl_fm_inputs[3] = m_tir_fm_outputs[4]; 
	m_whl_fm_inputs[4] = m_external_inputs[7];
	m_whl_fm_inputs[5] = m_tir_fm_outputs[2];
}

void NMSPC::Int_Whl_Disk_Tir_Fiala::map_tir_fm() {
	m_tir_fm_inputs[0] = m_external_inputs[7];
	m_tir_fm_inputs[1] = m_external_inputs[8];
	m_tir_fm_inputs[2] = m_external_inputs[9];
	m_tir_fm_inputs[3] = m_external_inputs[10];
}
