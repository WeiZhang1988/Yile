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
#ifndef SUBSYSTEM_TIRE_4FIALA_HPP
#define SUBSYSTEM_TIRE_4FIALA_HPP
#include "components/common.hpp"
#include "components/tire_fiala.hpp"
namespace NMSPC{
class Subsys_Tire_4Fiala {
public:
    static const int m_pv_inputs_num = Tire_Fiala::m_pv_inputs_num + Tire_Fiala::m_pv_inputs_num + Tire_Fiala::m_pv_inputs_num + Tire_Fiala::m_pv_inputs_num;
    static const int m_fm_inputs_num = Tire_Fiala::m_fm_inputs_num + Tire_Fiala::m_fm_inputs_num + Tire_Fiala::m_fm_inputs_num + Tire_Fiala::m_fm_inputs_num;
    static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;
    static const int m_con_states_num = Tire_Fiala::m_con_states_num + Tire_Fiala::m_con_states_num + Tire_Fiala::m_con_states_num + Tire_Fiala::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = Tire_Fiala::m_dis_states_num + Tire_Fiala::m_dis_states_num + Tire_Fiala::m_dis_states_num + Tire_Fiala::m_dis_states_num;
    static const int m_pv_outputs_num = Tire_Fiala::m_pv_outputs_num + Tire_Fiala::m_pv_outputs_num + Tire_Fiala::m_pv_outputs_num + Tire_Fiala::m_pv_outputs_num;
    static const int m_fm_outputs_num = Tire_Fiala::m_fm_outputs_num + Tire_Fiala::m_fm_outputs_num + Tire_Fiala::m_fm_outputs_num + Tire_Fiala::m_fm_outputs_num;
    static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;

    void add_tirs(std::shared_ptr<Tire_Fiala> sptr_tir_fl, std::shared_ptr<Tire_Fiala> sptr_tir_fr,\
    std::shared_ptr<Tire_Fiala> sptr_tir_rl, std::shared_ptr<Tire_Fiala> sptr_tir_rr) 
    {m_sptr_tir_fl=sptr_tir_fl; m_sptr_tir_fr=sptr_tir_fr;m_sptr_tir_rl=sptr_tir_rl; m_sptr_tir_rr=sptr_tir_rr;}

    void push_con_states(d_vec &con_states);
	void pull_con_states(const d_vec &con_states);
	void update_pv(const d_vec &inputs, d_vec &outputs);
	void update_fm(const d_vec &inputs, d_vec &outputs);
	void update_drv(d_vec &outputs);

    private:
    std::shared_ptr<Tire_Fiala> m_sptr_tir_fl, m_sptr_tir_fr, m_sptr_tir_rl, m_sptr_tir_rr;
    d_vec m_tir_con_states_fl = d_vec(Tire_Fiala::m_con_states_num, NaN);
    d_vec m_tir_con_states_fr = d_vec(Tire_Fiala::m_con_states_num, NaN);
    d_vec m_tir_con_states_rl = d_vec(Tire_Fiala::m_con_states_num, NaN);
    d_vec m_tir_con_states_rr = d_vec(Tire_Fiala::m_con_states_num, NaN);

    d_vec m_tir_drvs_fl = d_vec(Tire_Fiala::m_derivatives_num, NaN);
    d_vec m_tir_drvs_fr = d_vec(Tire_Fiala::m_derivatives_num, NaN);
    d_vec m_tir_drvs_rl = d_vec(Tire_Fiala::m_derivatives_num, NaN);
    d_vec m_tir_drvs_rr = d_vec(Tire_Fiala::m_derivatives_num, NaN);

    d_vec m_tir_pv_inputs_fl = d_vec(Tire_Fiala::m_pv_inputs_num, NaN);
    d_vec m_tir_pv_inputs_fr = d_vec(Tire_Fiala::m_pv_inputs_num, NaN);
    d_vec m_tir_pv_inputs_rl = d_vec(Tire_Fiala::m_pv_inputs_num, NaN);
    d_vec m_tir_pv_inputs_rr = d_vec(Tire_Fiala::m_pv_inputs_num, NaN);

    d_vec m_tir_pv_outputs_fl = d_vec(Tire_Fiala::m_pv_outputs_num, NaN);
    d_vec m_tir_pv_outputs_fr = d_vec(Tire_Fiala::m_pv_outputs_num, NaN);
    d_vec m_tir_pv_outputs_rl = d_vec(Tire_Fiala::m_pv_outputs_num, NaN);
    d_vec m_tir_pv_outputs_rr = d_vec(Tire_Fiala::m_pv_outputs_num, NaN);

    d_vec m_tir_fm_inputs_fl = d_vec(Tire_Fiala::m_fm_inputs_num, NaN);
    d_vec m_tir_fm_inputs_fr = d_vec(Tire_Fiala::m_fm_inputs_num, NaN);
    d_vec m_tir_fm_inputs_rl = d_vec(Tire_Fiala::m_fm_inputs_num, NaN);
    d_vec m_tir_fm_inputs_rr = d_vec(Tire_Fiala::m_fm_inputs_num, NaN);

    d_vec m_tir_fm_outputs_fl = d_vec(Tire_Fiala::m_fm_outputs_num, NaN);
    d_vec m_tir_fm_outputs_fr = d_vec(Tire_Fiala::m_fm_outputs_num, NaN);
    d_vec m_tir_fm_outputs_rl = d_vec(Tire_Fiala::m_fm_outputs_num, NaN);
    d_vec m_tir_fm_outputs_rr = d_vec(Tire_Fiala::m_fm_outputs_num, NaN);
};

} //end of name space
#endif //SUBSYSTEM_TIRE_4FIALA_HPP