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
#include "subsystem_whl_4disk.hpp"

void NMSPC::Subsys_Wheel_4Disk::push_con_states(d_vec &con_states) {
    m_sptr_whl_fl->push_con_states(m_whl_con_states_fl);
    m_sptr_whl_fr->push_con_states(m_whl_con_states_fr);
    m_sptr_whl_rl->push_con_states(m_whl_con_states_rl);
    m_sptr_whl_rr->push_con_states(m_whl_con_states_rr);
    std::copy(m_whl_con_states_fl.begin(),m_whl_con_states_fl.end(),con_states.begin() + 0 * Wheel_Disk::m_con_states_num);
    std::copy(m_whl_con_states_fr.begin(),m_whl_con_states_fr.end(),con_states.begin() + 1 * Wheel_Disk::m_con_states_num);
    std::copy(m_whl_con_states_rl.begin(),m_whl_con_states_rl.end(),con_states.begin() + 2 * Wheel_Disk::m_con_states_num);
    std::copy(m_whl_con_states_rr.begin(),m_whl_con_states_rr.end(),con_states.begin() + 3 * Wheel_Disk::m_con_states_num);
}

void NMSPC::Subsys_Wheel_4Disk::pull_con_states(const d_vec &con_states) {
    std::copy(con_states.begin() + 0 * Wheel_Disk::m_con_states_num,con_states.begin() + 1 * Wheel_Disk::m_con_states_num,m_whl_con_states_fl.begin());
    std::copy(con_states.begin() + 1 * Wheel_Disk::m_con_states_num,con_states.begin() + 2 * Wheel_Disk::m_con_states_num,m_whl_con_states_fr.begin());
    std::copy(con_states.begin() + 2 * Wheel_Disk::m_con_states_num,con_states.begin() + 3 * Wheel_Disk::m_con_states_num,m_whl_con_states_rl.begin());
    std::copy(con_states.begin() + 3 * Wheel_Disk::m_con_states_num,con_states.end(),m_whl_con_states_rr.begin());
    m_sptr_whl_fl->pull_con_states(m_whl_con_states_fl);
    m_sptr_whl_fr->pull_con_states(m_whl_con_states_fr);
    m_sptr_whl_rl->pull_con_states(m_whl_con_states_rl);
    m_sptr_whl_rr->pull_con_states(m_whl_con_states_rr);
}

void NMSPC::Subsys_Wheel_4Disk::update_pv(const d_vec &inputs, d_vec &outputs) {
    std::copy(inputs.begin() + 0 * Wheel_Disk::m_pv_inputs_num, inputs.begin() + 1 * Wheel_Disk::m_pv_inputs_num, m_whl_pv_inputs_fl.begin());
    std::copy(inputs.begin() + 1 * Wheel_Disk::m_pv_inputs_num, inputs.begin() + 2 * Wheel_Disk::m_pv_inputs_num, m_whl_pv_inputs_fr.begin());
    std::copy(inputs.begin() + 2 * Wheel_Disk::m_pv_inputs_num, inputs.begin() + 3 * Wheel_Disk::m_pv_inputs_num, m_whl_pv_inputs_rl.begin());
    std::copy(inputs.begin() + 3 * Wheel_Disk::m_pv_inputs_num, inputs.end(), m_whl_pv_inputs_rr.begin());
    m_sptr_whl_fl->update_pv(m_whl_pv_inputs_fl, m_whl_pv_outputs_fl);
    m_sptr_whl_fr->update_pv(m_whl_pv_inputs_fr, m_whl_pv_outputs_fr);
    m_sptr_whl_rl->update_pv(m_whl_pv_inputs_rl, m_whl_pv_outputs_rl);
    m_sptr_whl_rr->update_pv(m_whl_pv_inputs_rr, m_whl_pv_outputs_rr);
    std::copy(m_whl_pv_outputs_fl.begin(),m_whl_pv_outputs_fl.end(),outputs.begin() + 0 * Wheel_Disk::m_pv_outputs_num);
    std::copy(m_whl_pv_outputs_fr.begin(),m_whl_pv_outputs_fr.end(),outputs.begin() + 1 * Wheel_Disk::m_pv_outputs_num);
    std::copy(m_whl_pv_outputs_rl.begin(),m_whl_pv_outputs_rl.end(),outputs.begin() + 2 * Wheel_Disk::m_pv_outputs_num);
    std::copy(m_whl_pv_outputs_rr.begin(),m_whl_pv_outputs_rr.end(),outputs.begin() + 3 * Wheel_Disk::m_pv_outputs_num);
}

void NMSPC::Subsys_Wheel_4Disk::update_fm(const d_vec &inputs, d_vec &outputs) {
    std::copy(inputs.begin() + 0 * Wheel_Disk::m_fm_inputs_num, inputs.begin() + 1 * Wheel_Disk::m_fm_inputs_num, m_whl_fm_inputs_fl.begin());
    std::copy(inputs.begin() + 1 * Wheel_Disk::m_fm_inputs_num, inputs.begin() + 2 * Wheel_Disk::m_fm_inputs_num, m_whl_fm_inputs_fr.begin());
    std::copy(inputs.begin() + 2 * Wheel_Disk::m_fm_inputs_num, inputs.begin() + 3 * Wheel_Disk::m_fm_inputs_num, m_whl_fm_inputs_rl.begin());
    std::copy(inputs.begin() + 3 * Wheel_Disk::m_fm_inputs_num, inputs.end(), m_whl_fm_inputs_rr.begin());
    m_sptr_whl_fl->update_fm(m_whl_fm_inputs_fl, m_whl_fm_outputs_fl);
    m_sptr_whl_fr->update_fm(m_whl_fm_inputs_fr, m_whl_fm_outputs_fr);
    m_sptr_whl_rl->update_fm(m_whl_fm_inputs_rl, m_whl_fm_outputs_rl);
    m_sptr_whl_rr->update_fm(m_whl_fm_inputs_rr, m_whl_fm_outputs_rr);
    std::copy(m_whl_fm_outputs_fl.begin(),m_whl_fm_outputs_fl.end(),outputs.begin() + 0 * Wheel_Disk::m_fm_outputs_num);
    std::copy(m_whl_fm_outputs_fr.begin(),m_whl_fm_outputs_fr.end(),outputs.begin() + 1 * Wheel_Disk::m_fm_outputs_num);
    std::copy(m_whl_fm_outputs_rl.begin(),m_whl_fm_outputs_rl.end(),outputs.begin() + 2 * Wheel_Disk::m_fm_outputs_num);
    std::copy(m_whl_fm_outputs_rr.begin(),m_whl_fm_outputs_rr.end(),outputs.begin() + 3 * Wheel_Disk::m_fm_outputs_num);
}

void NMSPC::Subsys_Wheel_4Disk::update_drv(d_vec &outputs) {
    m_sptr_whl_fl->update_drv(m_whl_drvs_fl);
    m_sptr_whl_fr->update_drv(m_whl_drvs_fr);
    m_sptr_whl_rl->update_drv(m_whl_drvs_rl);
    m_sptr_whl_rr->update_drv(m_whl_drvs_rr);
    std::copy(m_whl_drvs_fl.begin(),m_whl_drvs_fl.end(),outputs.begin() + 0 * Wheel_Disk::m_derivatives_num);
    std::copy(m_whl_drvs_fr.begin(),m_whl_drvs_fr.end(),outputs.begin() + 1 * Wheel_Disk::m_derivatives_num);
    std::copy(m_whl_drvs_rl.begin(),m_whl_drvs_rl.end(),outputs.begin() + 2 * Wheel_Disk::m_derivatives_num);
    std::copy(m_whl_drvs_rr.begin(),m_whl_drvs_rr.end(),outputs.begin() + 3 * Wheel_Disk::m_derivatives_num);
}