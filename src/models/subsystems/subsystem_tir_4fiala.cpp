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
#include "subsystem_tir_4fiala.hpp"

void NMSPC::Subsys_Tire_4Fiala::push_con_states(d_vec &con_states) {
    m_sptr_tir_fl->push_con_states(m_tir_con_states_fl);
    m_sptr_tir_fr->push_con_states(m_tir_con_states_fr);
    m_sptr_tir_rl->push_con_states(m_tir_con_states_rl);
    m_sptr_tir_rr->push_con_states(m_tir_con_states_rr);
    std::copy(m_tir_con_states_fl.begin(),m_tir_con_states_fl.end(),con_states.begin() + 0 * Tire_Fiala::m_con_states_num);
    std::copy(m_tir_con_states_fr.begin(),m_tir_con_states_fr.end(),con_states.begin() + 1 * Tire_Fiala::m_con_states_num);
    std::copy(m_tir_con_states_rl.begin(),m_tir_con_states_rl.end(),con_states.begin() + 2 * Tire_Fiala::m_con_states_num);
    std::copy(m_tir_con_states_rr.begin(),m_tir_con_states_rr.end(),con_states.begin() + 3 * Tire_Fiala::m_con_states_num);
}

void NMSPC::Subsys_Tire_4Fiala::pull_con_states(const d_vec &con_states) {
    std::copy(con_states.begin() + 0 * Tire_Fiala::m_con_states_num,con_states.begin() + 1 * Tire_Fiala::m_con_states_num,m_tir_con_states_fl.begin());
    std::copy(con_states.begin() + 1 * Tire_Fiala::m_con_states_num,con_states.begin() + 2 * Tire_Fiala::m_con_states_num,m_tir_con_states_fr.begin());
    std::copy(con_states.begin() + 2 * Tire_Fiala::m_con_states_num,con_states.begin() + 3 * Tire_Fiala::m_con_states_num,m_tir_con_states_rl.begin());
    std::copy(con_states.begin() + 3 * Tire_Fiala::m_con_states_num,con_states.end(),m_tir_con_states_rr.begin());
    m_sptr_tir_fl->pull_con_states(m_tir_con_states_fl);
    m_sptr_tir_fr->pull_con_states(m_tir_con_states_fr);
    m_sptr_tir_rl->pull_con_states(m_tir_con_states_rl);
    m_sptr_tir_rr->pull_con_states(m_tir_con_states_rr);
}

void NMSPC::Subsys_Tire_4Fiala::update_pv(const d_vec &inputs, d_vec &outputs) {
    std::copy(inputs.begin() + 0 * Tire_Fiala::m_pv_inputs_num, inputs.begin() + 1 * Tire_Fiala::m_pv_inputs_num, m_tir_pv_inputs_fl.begin());
    std::copy(inputs.begin() + 1 * Tire_Fiala::m_pv_inputs_num, inputs.begin() + 2 * Tire_Fiala::m_pv_inputs_num, m_tir_pv_inputs_fr.begin());
    std::copy(inputs.begin() + 2 * Tire_Fiala::m_pv_inputs_num, inputs.begin() + 3 * Tire_Fiala::m_pv_inputs_num, m_tir_pv_inputs_rl.begin());
    std::copy(inputs.begin() + 3 * Tire_Fiala::m_pv_inputs_num, inputs.end(), m_tir_pv_inputs_rr.begin());
    m_sptr_tir_fl->update_pv(m_tir_pv_inputs_fl, m_tir_pv_outputs_fl);
    m_sptr_tir_fr->update_pv(m_tir_pv_inputs_fr, m_tir_pv_outputs_fr);
    m_sptr_tir_rl->update_pv(m_tir_pv_inputs_rl, m_tir_pv_outputs_rl);
    m_sptr_tir_rr->update_pv(m_tir_pv_inputs_rr, m_tir_pv_outputs_rr);
    std::copy(m_tir_pv_outputs_fl.begin(),m_tir_pv_outputs_fl.end(),outputs.begin() + 0 * Tire_Fiala::m_pv_outputs_num);
    std::copy(m_tir_pv_outputs_fr.begin(),m_tir_pv_outputs_fr.end(),outputs.begin() + 1 * Tire_Fiala::m_pv_outputs_num);
    std::copy(m_tir_pv_outputs_rl.begin(),m_tir_pv_outputs_rl.end(),outputs.begin() + 2 * Tire_Fiala::m_pv_outputs_num);
    std::copy(m_tir_pv_outputs_rr.begin(),m_tir_pv_outputs_rr.end(),outputs.begin() + 3 * Tire_Fiala::m_pv_outputs_num);
}

void NMSPC::Subsys_Tire_4Fiala::update_fm(const d_vec &inputs, d_vec &outputs) {
    std::copy(inputs.begin() + 0 * Tire_Fiala::m_fm_inputs_num, inputs.begin() + 1 * Tire_Fiala::m_fm_inputs_num, m_tir_fm_inputs_fl.begin());
    std::copy(inputs.begin() + 1 * Tire_Fiala::m_fm_inputs_num, inputs.begin() + 2 * Tire_Fiala::m_fm_inputs_num, m_tir_fm_inputs_fr.begin());
    std::copy(inputs.begin() + 2 * Tire_Fiala::m_fm_inputs_num, inputs.begin() + 3 * Tire_Fiala::m_fm_inputs_num, m_tir_fm_inputs_rl.begin());
    std::copy(inputs.begin() + 3 * Tire_Fiala::m_fm_inputs_num, inputs.end(), m_tir_fm_inputs_rr.begin());
    m_sptr_tir_fl->update_fm(m_tir_fm_inputs_fl, m_tir_fm_outputs_fl);
    m_sptr_tir_fr->update_fm(m_tir_fm_inputs_fr, m_tir_fm_outputs_fr);
    m_sptr_tir_rl->update_fm(m_tir_fm_inputs_rl, m_tir_fm_outputs_rl);
    m_sptr_tir_rr->update_fm(m_tir_fm_inputs_rr, m_tir_fm_outputs_rr);
    std::copy(m_tir_fm_outputs_fl.begin(),m_tir_fm_outputs_fl.end(),outputs.begin() + 0 * Tire_Fiala::m_fm_outputs_num);
    std::copy(m_tir_fm_outputs_fr.begin(),m_tir_fm_outputs_fr.end(),outputs.begin() + 1 * Tire_Fiala::m_fm_outputs_num);
    std::copy(m_tir_fm_outputs_rl.begin(),m_tir_fm_outputs_rl.end(),outputs.begin() + 2 * Tire_Fiala::m_fm_outputs_num);
    std::copy(m_tir_fm_outputs_rr.begin(),m_tir_fm_outputs_rr.end(),outputs.begin() + 3 * Tire_Fiala::m_fm_outputs_num);
}

void NMSPC::Subsys_Tire_4Fiala::update_drv(d_vec &outputs) {
    m_sptr_tir_fl->update_drv(m_tir_drvs_fl);
    m_sptr_tir_fr->update_drv(m_tir_drvs_fr);
    m_sptr_tir_rl->update_drv(m_tir_drvs_rl);
    m_sptr_tir_rr->update_drv(m_tir_drvs_rr);
    std::copy(m_tir_drvs_fl.begin(),m_tir_drvs_fl.end(),outputs.begin() + 0 * Tire_Fiala::m_derivatives_num);
    std::copy(m_tir_drvs_fr.begin(),m_tir_drvs_fr.end(),outputs.begin() + 1 * Tire_Fiala::m_derivatives_num);
    std::copy(m_tir_drvs_rl.begin(),m_tir_drvs_rl.end(),outputs.begin() + 2 * Tire_Fiala::m_derivatives_num);
    std::copy(m_tir_drvs_rr.begin(),m_tir_drvs_rr.end(),outputs.begin() + 3 * Tire_Fiala::m_derivatives_num);
}