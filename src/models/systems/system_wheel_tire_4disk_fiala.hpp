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
#ifndef SYSTEM_WHEEL_TIRE_4DISK_FIALA_HPP
#define SYSTEM_WHEEL_TIRE_4DISK_FIALA_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_whl_4disk.hpp"
#include "subsystems/subsystem_tir_4fiala.hpp"
#include "interfaces/interface_chassis_2ind_disk_fiala.hpp"

namespace NMSPC{
class Sys_Wheel_Tire_4Disk_Fiala {
public:
	static const int m_con_states_num  = Subsys_Wheel_4Disk::m_con_states_num + \
										 Subsys_Tire_4Fiala::m_con_states_num;						
	static const int m_derivatives_num = Subsys_Wheel_4Disk::m_derivatives_num + \
										 Subsys_Tire_4Fiala::m_derivatives_num;
	static const int m_dis_states_num  = Subsys_Wheel_4Disk::m_dis_states_num + \
										 Subsys_Tire_4Fiala::m_dis_states_num;
	
	void add_subsys_whl_4disk(std::shared_ptr<Subsys_Wheel_4Disk> sptr_subsys_whl_4disk) 	{m_sptr_subsys_whl_4disk = sptr_subsys_whl_4disk;}
	void add_subsys_tir_4fiala(std::shared_ptr<Subsys_Tire_4Fiala> sptr_subsys_tir_4fiala) 	{m_sptr_subsys_tir_4fiala = sptr_subsys_tir_4fiala;}
	void add_interface(std::shared_ptr<Int_Chassis_2Ind_Disk_Fiala> sptr_interface) 		{m_sptr_interface = sptr_interface;}
	void add_store(std::shared_ptr<d_v_vec> sptr_store) 									{m_sptr_store=sptr_store;}
	
	void push_con_states(d_vec &con_states);
	void push_con_states_omega_only(d_vec &con_states);
	void pull_con_states(const d_vec &con_states);

	void update_pv();
	void update_fm();

	void update_drv();

	void store_data();

	void operator() (const d_vec &x, d_vec &dxdt, const double &t);
	

private:
	std::shared_ptr<Subsys_Wheel_4Disk> 			m_sptr_subsys_whl_4disk;
	std::shared_ptr<Subsys_Tire_4Fiala> 			m_sptr_subsys_tir_4fiala;
	std::shared_ptr<Int_Chassis_2Ind_Disk_Fiala> 	m_sptr_interface;
	std::shared_ptr<d_v_vec> 						m_sptr_store;

	//inputs
	//continuous states 
	d_vec m_subsys_whl_4disk_con_states  = d_vec(Subsys_Wheel_4Disk::m_con_states_num,NaN);
	d_vec m_subsys_tir_4fiala_con_states = d_vec(Subsys_Tire_4Fiala::m_con_states_num,NaN);
	//continuous states derivatives
	d_vec m_subsys_whl_4disk_drvs 		 = d_vec(Subsys_Wheel_4Disk::m_derivatives_num,NaN);
	d_vec m_subsys_tir_4fiala_drvs 		 = d_vec(Subsys_Tire_4Fiala::m_derivatives_num,NaN);
	d_vec m_drvs = d_vec(m_derivatives_num,NaN);
	
public:
	//continuous states
	d_vec m_con_states = d_vec(m_con_states_num,NaN);
};

}	//end of name space
#endif //SYSTEM_WHEEL_TIRE_4DISK_FIALA_HPP
