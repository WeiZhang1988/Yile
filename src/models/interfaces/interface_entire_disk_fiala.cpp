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
#include "interface_entire_disk_fiala.hpp"

NMSPC::Int_Entire_Disk_Fiala::Int_Entire_Disk_Fiala(int ext_inputs_num) {
	m_external_inputs = d_vec(ext_inputs_num, NaN);
}

void NMSPC::Int_Entire_Disk_Fiala::map_vhl_bdy_pv() {

}

void NMSPC::Int_Entire_Disk_Fiala::map_sub_whl_4disk_pv() {

}

void NMSPC::Int_Entire_Disk_Fiala::map_sub_sus_2ind_pv() {

}

void NMSPC::Int_Entire_Disk_Fiala::map_sub_tir_4fiala_pv() {

}

void NMSPC::Int_Entire_Disk_Fiala::map_sub_tir_4fiala_fm() {

}

void NMSPC::Int_Entire_Disk_Fiala::map_sub_sus_2ind_fm() {

}

void NMSPC::Int_Entire_Disk_Fiala::map_sub_whl_4disk_fm() {

}

void NMSPC::Int_Entire_Disk_Fiala::map_vhl_bdy_fm() {

    //inputs
	//WindXYZ in inertial coordinate
	double m_Wx = NaN, m_Wy = NaN, m_Wz = NaN;
	//AirTemp
	double m_Tair = NaN;
	//FSusp
	double m_Fx_fl = NaN, m_Fx_fr = NaN, m_Fx_rl = NaN, m_Fx_rr = NaN, \
	m_Fy_fl = NaN, m_Fy_fr = NaN, m_Fy_rl = NaN, m_Fy_rr = NaN, \
	m_Fz_fl = NaN, m_Fz_fr = NaN, m_Fz_rl = NaN, m_Fz_rr = NaN;
	//MSusp
	double m_Mx_fl = NaN, m_Mx_fr = NaN, m_Mx_rl = NaN, m_Mx_rr = NaN, \
	m_My_fl = NaN, m_My_fr = NaN, m_My_rl = NaN, m_My_rr = NaN, \
	m_Mz_fl = NaN, m_Mz_fr = NaN, m_Mz_rl = NaN, m_Mz_rr = NaN;
	//FExt
	double m_Fx_ext = NaN, m_Fy_ext = NaN, m_Fz_ext = NaN;
	//MExt
	double m_Mx_ext = NaN, m_My_ext = NaN, m_Mz_ext = NaN;
}
