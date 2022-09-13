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
#ifndef SYSTEM_WHEEL_DISK_TIRE_FIALA_HPP
#define SYSTEM_WHEEL_DISK_TIRE_FIALA_HPP
#include "components/common.hpp"
#include "components/wheel_disk.hpp"
#include "components/tire_fiala.hpp"
#include "interfaces/interface_whl_disk_tir_fiala.hpp"

namespace NMSPC{
class Sys_Whl_Disk_Tir_Fiala {
public:
	static const int m_external_inputs_num = 11;
	
	static const int m_pv_inters_num = 3;
	static const int m_fm_inters_num = 3;

	static const int m_pv_inputs_num = Wheel_Disk::m_pv_inputs_num + Tire_Fiala::m_pv_inputs_num - m_pv_inters_num;
	static const int m_fm_inputs_num = Wheel_Disk::m_fm_inputs_num + Tire_Fiala::m_fm_inputs_num - m_fm_inters_num;
	static const int m_inputs_num = m_pv_inputs_num + m_fm_inputs_num;
	static const int m_con_states_num = Wheel_Disk::m_con_states_num + Tire_Fiala::m_con_states_num;						
	static const int m_derivatives_num = Wheel_Disk::m_derivatives_num + Tire_Fiala::m_derivatives_num;
	static const int m_dis_states_num = Wheel_Disk::m_dis_states_num + Tire_Fiala::m_dis_states_num;
	static const int m_pv_outputs_num = Wheel_Disk::m_pv_outputs_num + Tire_Fiala::m_pv_outputs_num;
	static const int m_fm_outputs_num = Wheel_Disk::m_fm_outputs_num + Tire_Fiala::m_fm_outputs_num;
	static const int m_outputs_num = m_pv_outputs_num + m_fm_outputs_num;
	
	void add_whl(std::shared_ptr<Wheel_Disk> sptr_whl) {m_sptr_whl=sptr_whl;}
	void add_tir(std::shared_ptr<Tire_Fiala> sptr_tir) {m_sptr_tir=sptr_tir;}
	void add_interface(std::shared_ptr<Int_Whl_Disk_Tir_Fiala> sptr_interface) {m_sptr_interface=sptr_interface;}
	
	void push_con_states(d_vec &con_states);
	void pull_con_states(const d_vec &con_states);
	void pull_external_inputs(const d_vec &inputs);
	void update_pv();
	void update_fm();
	void update_drv();
	void operator() (const d_vec &x, d_vec &dxdt, const double &t);
	

private:
	std::shared_ptr<Wheel_Disk> m_sptr_whl;
	std::shared_ptr<Tire_Fiala> m_sptr_tir;
	std::shared_ptr<Int_Whl_Disk_Tir_Fiala> m_sptr_interface;

	//inputs
	//external inputs are
	//Gnd vx vy gamma psidot AxlTrq BrkPrs SusFz scale TirPrs Tamb
	//continuous states
	d_vec m_whl_con_states = d_vec(Wheel_Disk::m_con_states_num,NaN);
	d_vec m_tir_con_states = d_vec(Tire_Fiala::m_con_states_num,NaN);
	//continuous states derivatives
	d_vec m_whl_drv_states = d_vec(Wheel_Disk::m_derivatives_num,NaN);
	d_vec m_tir_drv_states = d_vec(Tire_Fiala::m_derivatives_num,NaN);
	d_vec m_drvs = d_vec(m_derivatives_num,NaN);
	//outputs
	//external outputs are 
	//omega Re Pz Vz rhoz BrkTrq TirFx TirFy TirFz TirMx TirMy TirMz
	
public:
	//continuous states
	d_vec m_con_states = d_vec(m_con_states_num,NaN);
};

}	//end of name space
#endif //SYSTEM_WHEEL_DISK_TIRE_FIALA_HPP
