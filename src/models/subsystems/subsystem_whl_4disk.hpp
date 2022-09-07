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
#ifndef SUBSYSTEM_WHEEL_4DISK_HPP
#define SUBSYSTEM_WHEEL_4DISK_HPP
#include "components/common.hpp"
#include "components/wheel_disk.hpp"
namespace NMSPC{
class Subsys_Wheel_4Disk {
public:
    static const int m_pv_inputs_num = Wheel_Disk::m_pv_inputs_num + Wheel_Disk::m_pv_inputs_num + Wheel_Disk::m_pv_inputs_num + Wheel_Disk::m_pv_inputs_num;
    static const int m_fm_inputs_num = Wheel_Disk::m_fm_inputs_num + Wheel_Disk::m_fm_inputs_num + Wheel_Disk::m_fm_inputs_num + Wheel_Disk::m_fm_inputs_num;
    static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;
    static const int m_con_states_num = Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num + Wheel_Disk::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num + Wheel_Disk::m_dis_states_num;
    static const int m_pv_outputs_num = Wheel_Disk::m_pv_outputs_num + Wheel_Disk::m_pv_outputs_num + Wheel_Disk::m_pv_outputs_num + Wheel_Disk::m_pv_outputs_num;
    static const int m_fm_outputs_num = Wheel_Disk::m_fm_outputs_num + Wheel_Disk::m_fm_outputs_num + Wheel_Disk::m_fm_outputs_num + Wheel_Disk::m_fm_outputs_num;
    static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;

    void add_whls(std::shared_ptr<Wheel_Disk> sptr_whl_fl, std::shared_ptr<Wheel_Disk> sptr_whl_fr,\
    std::shared_ptr<Wheel_Disk> sptr_whl_rl, std::shared_ptr<Wheel_Disk> sptr_whl_rr) 
    {m_sptr_whl_fl=sptr_whl_fl; m_sptr_whl_fr=sptr_whl_fr;m_sptr_whl_rl=sptr_whl_rl; m_sptr_whl_rr=sptr_whl_rr;}

    void push_con_states(d_vec &con_states);
	void pull_con_states(const d_vec &con_states);
	void update_pv(const d_vec &inputs, d_vec &outputs);
	void update_fm(const d_vec &inputs, d_vec &outputs);
	void update_drv(d_vec &outputs);

    private:
    std::shared_ptr<Wheel_Disk> m_sptr_whl_fl, m_sptr_whl_fr, m_sptr_whl_rl, m_sptr_whl_rr;
    d_vec m_whl_con_states_fl = d_vec(Wheel_Disk::m_con_states_num, NaN);
    d_vec m_whl_con_states_fr = d_vec(Wheel_Disk::m_con_states_num, NaN);
    d_vec m_whl_con_states_rl = d_vec(Wheel_Disk::m_con_states_num, NaN);
    d_vec m_whl_con_states_rr = d_vec(Wheel_Disk::m_con_states_num, NaN);

    d_vec m_whl_drvs_fl = d_vec(Wheel_Disk::m_derivatives_num, NaN);
    d_vec m_whl_drvs_fr = d_vec(Wheel_Disk::m_derivatives_num, NaN);
    d_vec m_whl_drvs_rl = d_vec(Wheel_Disk::m_derivatives_num, NaN);
    d_vec m_whl_drvs_rr = d_vec(Wheel_Disk::m_derivatives_num, NaN);

    d_vec m_whl_pv_inputs_fl = d_vec(Wheel_Disk::m_pv_inputs_num, NaN);
    d_vec m_whl_pv_inputs_fr = d_vec(Wheel_Disk::m_pv_inputs_num, NaN);
    d_vec m_whl_pv_inputs_rl = d_vec(Wheel_Disk::m_pv_inputs_num, NaN);
    d_vec m_whl_pv_inputs_rr = d_vec(Wheel_Disk::m_pv_inputs_num, NaN);

    d_vec m_whl_pv_outputs_fl = d_vec(Wheel_Disk::m_pv_outputs_num, NaN);
    d_vec m_whl_pv_outputs_fr = d_vec(Wheel_Disk::m_pv_outputs_num, NaN);
    d_vec m_whl_pv_outputs_rl = d_vec(Wheel_Disk::m_pv_outputs_num, NaN);
    d_vec m_whl_pv_outputs_rr = d_vec(Wheel_Disk::m_pv_outputs_num, NaN);

    d_vec m_whl_fm_inputs_fl = d_vec(Wheel_Disk::m_fm_inputs_num, NaN);
    d_vec m_whl_fm_inputs_fr = d_vec(Wheel_Disk::m_fm_inputs_num, NaN);
    d_vec m_whl_fm_inputs_rl = d_vec(Wheel_Disk::m_fm_inputs_num, NaN);
    d_vec m_whl_fm_inputs_rr = d_vec(Wheel_Disk::m_fm_inputs_num, NaN);

    d_vec m_whl_fm_outputs_fl = d_vec(Wheel_Disk::m_fm_outputs_num, NaN);
    d_vec m_whl_fm_outputs_fr = d_vec(Wheel_Disk::m_fm_outputs_num, NaN);
    d_vec m_whl_fm_outputs_rl = d_vec(Wheel_Disk::m_fm_outputs_num, NaN);
    d_vec m_whl_fm_outputs_rr = d_vec(Wheel_Disk::m_fm_outputs_num, NaN);
};

} //end of name space
#endif //SUBSYSTEM_WHEEL_4DISK_HPP