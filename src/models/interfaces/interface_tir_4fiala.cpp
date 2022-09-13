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
#include "interface_tir_4fiala.hpp"

NMSPC::Int_Tir_4Fiala::Int_Tir_4Fiala(int ext_inputs_num) {
	m_external_inputs = d_vec(ext_inputs_num, NaN);
}

void NMSPC::Int_Tir_4Fiala::map_sub_tir_4fiala_pv() {
	m_sub_tir_4fiala_pv_inputs[0] = m_external_inputs[0];
    m_sub_tir_4fiala_pv_inputs[1] = m_external_inputs[1];
    m_sub_tir_4fiala_pv_inputs[2] = m_external_inputs[2];
    m_sub_tir_4fiala_pv_inputs[3] = m_external_inputs[3];
    m_sub_tir_4fiala_pv_inputs[4] = m_external_inputs[4];
    m_sub_tir_4fiala_pv_inputs[5] = m_external_inputs[5];
    m_sub_tir_4fiala_pv_inputs[6] = m_external_inputs[6];
    m_sub_tir_4fiala_pv_inputs[7] = m_external_inputs[7];
    m_sub_tir_4fiala_pv_inputs[8] = m_external_inputs[8];

    m_sub_tir_4fiala_pv_inputs[9] = m_external_inputs[13];
    m_sub_tir_4fiala_pv_inputs[10] = m_external_inputs[14];
    m_sub_tir_4fiala_pv_inputs[11] = m_external_inputs[15];
    m_sub_tir_4fiala_pv_inputs[12] = m_external_inputs[16];
    m_sub_tir_4fiala_pv_inputs[13] = m_external_inputs[17];
    m_sub_tir_4fiala_pv_inputs[14] = m_external_inputs[18];
    m_sub_tir_4fiala_pv_inputs[15] = m_external_inputs[19];
    m_sub_tir_4fiala_pv_inputs[16] = m_external_inputs[20];
    m_sub_tir_4fiala_pv_inputs[17] = m_external_inputs[21];

    m_sub_tir_4fiala_pv_inputs[18] = m_external_inputs[26];
    m_sub_tir_4fiala_pv_inputs[19] = m_external_inputs[27];
    m_sub_tir_4fiala_pv_inputs[20] = m_external_inputs[28];
    m_sub_tir_4fiala_pv_inputs[21] = m_external_inputs[29];
    m_sub_tir_4fiala_pv_inputs[22] = m_external_inputs[30];
    m_sub_tir_4fiala_pv_inputs[23] = m_external_inputs[31];
    m_sub_tir_4fiala_pv_inputs[24] = m_external_inputs[32];
    m_sub_tir_4fiala_pv_inputs[25] = m_external_inputs[33];
    m_sub_tir_4fiala_pv_inputs[26] = m_external_inputs[34];

    m_sub_tir_4fiala_pv_inputs[27] = m_external_inputs[39];
    m_sub_tir_4fiala_pv_inputs[28] = m_external_inputs[40];
    m_sub_tir_4fiala_pv_inputs[29] = m_external_inputs[41];
    m_sub_tir_4fiala_pv_inputs[30] = m_external_inputs[42];
    m_sub_tir_4fiala_pv_inputs[31] = m_external_inputs[43];
    m_sub_tir_4fiala_pv_inputs[32] = m_external_inputs[44];
    m_sub_tir_4fiala_pv_inputs[33] = m_external_inputs[45];
    m_sub_tir_4fiala_pv_inputs[34] = m_external_inputs[46];
    m_sub_tir_4fiala_pv_inputs[35] = m_external_inputs[47];
}

void NMSPC::Int_Tir_4Fiala::map_sub_tir_4fiala_fm() {
	m_sub_tir_4fiala_fm_inputs[0] = m_external_inputs[9];
    m_sub_tir_4fiala_fm_inputs[1] = m_external_inputs[10];
    m_sub_tir_4fiala_fm_inputs[2] = m_external_inputs[11];
    m_sub_tir_4fiala_fm_inputs[3] = m_external_inputs[12];

    m_sub_tir_4fiala_fm_inputs[4] = m_external_inputs[22];
    m_sub_tir_4fiala_fm_inputs[5] = m_external_inputs[23];
    m_sub_tir_4fiala_fm_inputs[6] = m_external_inputs[24];
    m_sub_tir_4fiala_fm_inputs[7] = m_external_inputs[25];

    m_sub_tir_4fiala_fm_inputs[8]  = m_external_inputs[35];
    m_sub_tir_4fiala_fm_inputs[9]  = m_external_inputs[36];
    m_sub_tir_4fiala_fm_inputs[10] = m_external_inputs[37];
    m_sub_tir_4fiala_fm_inputs[11] = m_external_inputs[38];

    m_sub_tir_4fiala_fm_inputs[12] = m_external_inputs[48];
    m_sub_tir_4fiala_fm_inputs[13] = m_external_inputs[49];
    m_sub_tir_4fiala_fm_inputs[14] = m_external_inputs[50];
    m_sub_tir_4fiala_fm_inputs[15] = m_external_inputs[51];
}

