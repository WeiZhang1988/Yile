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
#ifndef SYSTEM_VEHICLE_BODY_HPP
#define SYSTEM_VEHICLE_BODY_HPP
#include "components/common.hpp"
#include "components/vehicle_body.hpp"
#include "interfaces/interface_vehicle_body.hpp"

namespace NMSPC{
class Sys_Vehicle_Body {
public:
    static const int m_pv_inters_num = 0;
	static const int m_fm_inters_num = 0;

	static const int m_pv_inputs_num = Vehicle_Body::m_pv_inputs_num - m_pv_inters_num;
	static const int m_fm_inputs_num = Vehicle_Body::m_fm_inputs_num - m_fm_inters_num;
	static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;
    static const int m_external_inputs_num = m_inputs_num - m_pv_inters_num - m_fm_inters_num;
	static const int m_con_states_num = Vehicle_Body::m_con_states_num;						
	static const int m_derivatives_num = Vehicle_Body::m_derivatives_num;
	static const int m_dis_states_num = Vehicle_Body::m_dis_states_num;
	static const int m_pv_outputs_num = Vehicle_Body::m_pv_outputs_num;
	static const int m_fm_outputs_num = Vehicle_Body::m_fm_outputs_num;
	static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;
	
	void add_vhl_bdy(std::shared_ptr<Vehicle_Body> sptr_vhl_bdy) \
    {m_sptr_vhl_bdy = sptr_vhl_bdy;}
	
	void push_con_states(d_vec &con_states);
	void pull_con_states(const d_vec &con_states);
	void pull_external_inputs(const d_vec &inputs);
	void update_pv();
	void update_fm();
	void update_drv();
	void operator() (const d_vec &x, d_vec &dxdt, const double &t);
	

private:
	std::shared_ptr<Vehicle_Body> m_sptr_vhl_bdy;

	//inputs
	//continuous states
	d_vec m_vhl_bdy_con_states = d_vec(Vehicle_Body::m_con_states_num,NaN);
	//continuous states derivatives
	d_vec m_vhl_bdy_drvs = d_vec(Vehicle_Body::m_derivatives_num,NaN);
	d_vec m_drvs = d_vec(m_derivatives_num,NaN);
	//outputs
	
public:
	//continuous states
	d_vec m_con_states = d_vec(m_con_states_num,NaN);
	//interface 
	Int_Vehicle_Body interface = Int_Vehicle_Body(m_external_inputs_num);
};

}	//end of name space
#endif //SYSTEM_VEHICLE_BODY_HPP
