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
#ifndef SUBSYSTEM_SUS_2INDEPENDENT_HPP
#define SUBSYSTEM_SUS_2INDEPENDENT_HPP
#include "components/common.hpp"
#include "components/sus_ind_2tracks.hpp"

namespace NMSPC{
class Subsys_Sus_2Ind {
public:
    static const int m_pv_inputs_num = Sus_Ind_2Tracks::m_pv_inputs_num + Sus_Ind_2Tracks::m_pv_inputs_num;
    static const int m_fm_inputs_num = Sus_Ind_2Tracks::m_fm_inputs_num + Sus_Ind_2Tracks::m_fm_inputs_num;
    static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;
    static const int m_con_states_num = Sus_Ind_2Tracks::m_con_states_num + Sus_Ind_2Tracks::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = Sus_Ind_2Tracks::m_dis_states_num + Sus_Ind_2Tracks::m_dis_states_num;
    static const int m_pv_outputs_num = Sus_Ind_2Tracks::m_pv_outputs_num + Sus_Ind_2Tracks::m_pv_outputs_num;
    static const int m_fm_outputs_num = Sus_Ind_2Tracks::m_fm_outputs_num + Sus_Ind_2Tracks::m_fm_outputs_num;
    static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;

    void add_suses(std::shared_ptr<Sus_Ind_2Tracks> sptr_sus_f, std::shared_ptr<Sus_Ind_2Tracks> sptr_sus_r) {m_sptr_sus_f=sptr_sus_f; m_sptr_sus_r=sptr_sus_r;}

    void push_con_states(d_vec &con_states) {};
	void pull_con_states(const d_vec &con_states) {};
	void update_pv(const d_vec &inputs, d_vec &outputs);
	void update_fm(const d_vec &inputs, d_vec &outputs);
	void update_drv(d_vec &outputs) {};

private:
    std::shared_ptr<Sus_Ind_2Tracks> m_sptr_sus_f;
    std::shared_ptr<Sus_Ind_2Tracks> m_sptr_sus_r;
    d_vec m_sus_pv_inputs_f = d_vec(Sus_Ind_2Tracks::m_pv_inputs_num, NaN);
    d_vec m_sus_pv_inputs_r = d_vec(Sus_Ind_2Tracks::m_pv_inputs_num, NaN);
    d_vec m_sus_pv_outputs_f = d_vec(Sus_Ind_2Tracks::m_pv_outputs_num, NaN);
    d_vec m_sus_pv_outputs_r = d_vec(Sus_Ind_2Tracks::m_pv_outputs_num, NaN);
    d_vec m_sus_fm_inputs_f = d_vec(Sus_Ind_2Tracks::m_fm_inputs_num, NaN);
    d_vec m_sus_fm_inputs_r = d_vec(Sus_Ind_2Tracks::m_fm_inputs_num, NaN);
    d_vec m_sus_fm_outputs_f = d_vec(Sus_Ind_2Tracks::m_fm_outputs_num, NaN);
    d_vec m_sus_fm_outputs_r = d_vec(Sus_Ind_2Tracks::m_fm_outputs_num, NaN);
};
} //end of name space
#endif //SUBSYSTEM_SUS_2INDEPENDENT_HPP