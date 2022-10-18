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
#ifndef SYSTEM_CHASSIS_2IND_DISK_FIALA_HPP
#define SYSTEM_CHASSIS_2IND_DISK_FIALA_HPP
#include "components/common.hpp"
#include "components/vehicle_body.hpp"
#include "subsystems/subsystem_sus_2ind.hpp"
#include "subsystems/subsystem_whl_4disk.hpp"
#include "subsystems/subsystem_tir_4fiala.hpp"
#include "interfaces/interface_chassis_2ind_disk_fiala.hpp"

namespace NMSPC{
class Sys_Chassis_2Ind_Disk_Fiala {
public:
	static const int m_con_states_num  = Vehicle_Body::m_con_states_num + \
										 Subsys_Wheel_4Disk::m_con_states_num + \
										 Subsys_Sus_2Ind::m_con_states_num + \
										 Subsys_Tire_4Fiala::m_con_states_num;						
	static const int m_derivatives_num = Vehicle_Body::m_derivatives_num + \
										 Subsys_Wheel_4Disk::m_derivatives_num + \
										 Subsys_Sus_2Ind::m_derivatives_num + \
										 Subsys_Tire_4Fiala::m_derivatives_num;
	static const int m_dis_states_num  = Vehicle_Body::m_dis_states_num + \
										 Subsys_Wheel_4Disk::m_dis_states_num + \
										 Subsys_Sus_2Ind::m_dis_states_num + \
										 Subsys_Tire_4Fiala::m_dis_states_num;
	
	void add_vhl_bdy(std::shared_ptr<Vehicle_Body> sptr_vhl_bdy) 							{m_sptr_vhl_bdy = sptr_vhl_bdy;}
	void add_subsys_whl_4disk(std::shared_ptr<Subsys_Wheel_4Disk> sptr_subsys_whl_4disk) 	{m_sptr_subsys_whl_4disk = sptr_subsys_whl_4disk;}
	void add_subsys_sus_2ind(std::shared_ptr<Subsys_Sus_2Ind> sptr_subsys_sus_2ind) 		{m_sptr_subsys_sus_2ind = sptr_subsys_sus_2ind;}
	void add_subsys_tir_4fiala(std::shared_ptr<Subsys_Tire_4Fiala> sptr_subsys_tir_4fiala) 	{m_sptr_subsys_tir_4fiala = sptr_subsys_tir_4fiala;}
	void add_interface(std::shared_ptr<Int_Chassis_2Ind_Disk_Fiala> sptr_interface) 		{m_sptr_interface = sptr_interface;}
	void add_store(std::shared_ptr<d_v_vec> sptr_store) 									{m_sptr_store=sptr_store;}
	
	void push_con_states(d_vec &con_states);
	void push_con_states_whl_only(d_vec &con_states);
	void pull_con_states(const d_vec &con_states);
	void pull_con_states_whl_only(const d_vec &con_states);

	void update_pv();
	void update_fm();

	void update_drv();

	void store_data();

	void operator() (const d_vec &x, d_vec &dxdt, const real_Y &t);
	

private:
	std::shared_ptr<Vehicle_Body> 					m_sptr_vhl_bdy;
	std::shared_ptr<Subsys_Wheel_4Disk> 			m_sptr_subsys_whl_4disk;
	std::shared_ptr<Subsys_Sus_2Ind> 				m_sptr_subsys_sus_2ind;
	std::shared_ptr<Subsys_Tire_4Fiala> 			m_sptr_subsys_tir_4fiala;
	std::shared_ptr<Int_Chassis_2Ind_Disk_Fiala> 	m_sptr_interface;
	std::shared_ptr<d_v_vec> 						m_sptr_store;

	//inputs
	//continuous states 
	d_vec m_vhl_bdy_con_states 			 = d_vec(Vehicle_Body::m_con_states_num,NaN);
	d_vec m_subsys_whl_4disk_con_states  = d_vec(Subsys_Wheel_4Disk::m_con_states_num,NaN);
	d_vec m_subsys_sus_2ind_con_states   = d_vec(Subsys_Sus_2Ind::m_con_states_num,NaN);
	d_vec m_subsys_tir_4fiala_con_states = d_vec(Subsys_Tire_4Fiala::m_con_states_num,NaN);
	//continuous states derivatives
	d_vec m_vhl_bdy_drvs 				 = d_vec(Vehicle_Body::m_derivatives_num,NaN);
	d_vec m_subsys_whl_4disk_drvs 		 = d_vec(Subsys_Wheel_4Disk::m_derivatives_num,NaN);
	d_vec m_subsys_sus_2ind_drvs 		 = d_vec(Subsys_Sus_2Ind::m_derivatives_num,NaN);
	d_vec m_subsys_tir_4fiala_drvs 		 = d_vec(Subsys_Tire_4Fiala::m_derivatives_num,NaN);
	d_vec m_drvs = d_vec(m_derivatives_num,NaN);
	
public:
	//continuous states
	d_vec m_con_states = d_vec(m_con_states_num,NaN);
};

}	//end of name space
#endif //SYSTEM_CHASSIS_2IND_DISK_FIALA_HPP
