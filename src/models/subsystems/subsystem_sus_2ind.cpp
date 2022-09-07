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
#include "subsystem_sus_2ind.hpp"

void NMSPC::Subsys_Sus_2Ind::update_pv(const d_vec &inputs, d_vec &outputs) {
    std::copy(inputs.begin(), inputs.begin() + Sus_Ind_2Tracks::m_pv_inputs_num, m_sus_pv_inputs_f.begin());
    std::copy(inputs.begin() + Sus_Ind_2Tracks::m_pv_inputs_num, inputs.end()  , m_sus_pv_inputs_r.begin());
    m_sptr_sus_f->update_pv(m_sus_pv_inputs_f, m_sus_pv_outputs_f);
    m_sptr_sus_r->update_pv(m_sus_pv_inputs_r, m_sus_pv_outputs_r);
    std::copy(m_sus_pv_outputs_f.begin(),m_sus_pv_outputs_f.end(),outputs.begin());
    std::copy(m_sus_pv_outputs_r.begin(),m_sus_pv_outputs_r.end(),outputs.begin() + Sus_Ind_2Tracks::m_pv_outputs_num);
}

void NMSPC::Subsys_Sus_2Ind::update_fm(const d_vec &inputs, d_vec &outputs) {
    std::copy(inputs.begin(), inputs.begin() + Sus_Ind_2Tracks::m_fm_inputs_num, m_sus_fm_inputs_f.begin());
    std::copy(inputs.begin() + Sus_Ind_2Tracks::m_fm_inputs_num, inputs.end()  , m_sus_fm_inputs_r.begin());
    m_sptr_sus_f->update_pv(m_sus_fm_inputs_f, m_sus_fm_outputs_f);
    m_sptr_sus_r->update_pv(m_sus_fm_inputs_r, m_sus_fm_outputs_r);
    std::copy(m_sus_fm_outputs_f.begin(),m_sus_fm_outputs_f.end(),outputs.begin());
    std::copy(m_sus_fm_outputs_r.begin(),m_sus_fm_outputs_r.end(),outputs.begin() + Sus_Ind_2Tracks::m_fm_outputs_num);
}