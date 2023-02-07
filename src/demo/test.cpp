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
#include <string>
#include <fstream>
#include "simulators/simulator_whl_4disk.hpp"
#include "simulators/simulator_tir_4fiala.hpp"
#include "simulators/simulator_sus_2ind.hpp"
#include "simulators/simulator_vehicle_body.hpp"
#include "simulators/simulator_chassis_2ind_disk_fiala.hpp"
#include "simulators/simulator_wheel_tire_4disk_fiala.hpp"
#include "simulators/simulator_sus_wheel_tire_2ind_disk_fiala.hpp"
#include "simulator_pass14dof.hpp"

using namespace Yile;

void run_Whl_4Disk() {
	std::ofstream of;
	Simulator_Whl_4Disk sim1 = Simulator_Whl_4Disk(0.0,100.0,1e-4);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_whl_4disk.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (auto items : *sim1.m_sptr_store) {
		for (auto item : items) {
			of<<std::setprecision(15)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();
}

void run_Tir_4Fiala() {
	std::ofstream of;
	Simulator_Tir_4Fiala sim1 = Simulator_Tir_4Fiala(0.0,100.0,1e-3);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_tir_4fiala.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (auto items : *sim1.m_sptr_store) {
		for (auto item : items) {
			of<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();
	
}

void run_Sus_2Ind() {
	std::ofstream of;
	Simulator_Sus_2Ind sim1 = Simulator_Sus_2Ind(0.0,100,1e-3);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_sus_2ind.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (auto items : *sim1.m_sptr_store) {
		for (auto item : items) {
			of<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();	
}

void run_Vehicle_Body() {
	std::ofstream of;
	Simulator_Vehicle_Body sim1 = Simulator_Vehicle_Body(0.0,100,5e-4);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_veh_bdy.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (auto items : *sim1.m_sptr_store) {
		for (auto item : items) {
			of<<std::setw(30)<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();

	of.open("res_veh_bdy_states.csv");
	for (auto items : sim1.m_sptr_store_states) {
		for (auto item : items) {
			of<<std::setw(30)<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();
	
	
}

void run_Chassis_2Ind_Disk_Fiala() {
	std::ofstream of;
	Simulator_Chassis_2Ind_Disk_Fiala sim1 = Simulator_Chassis_2Ind_Disk_Fiala(0.0,100.0,5e-4);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_chassis_2ind_disk_fiala.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (auto items : *sim1.m_sptr_store) {
		for (auto item : items) {
			of<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();
}

void run_Wheel_Tire_4Disk_Fiala() {
	std::ofstream of;
	Simulator_Wheel_Tire_4Disk_Fiala sim1 = Simulator_Wheel_Tire_4Disk_Fiala(0.0,100.0,1e-3);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_wheel_tire_4disk_fiala.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (auto items : *sim1.m_sptr_store) {
		for (auto item : items) {
			of<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();
}


void run_Sus_Wheel_Tire_2Ind_Disk_Fiala() {
	std::ofstream of;
	Simulator_Sus_Wheel_Tire_2Ind_Disk_Fiala sim1 = Simulator_Sus_Wheel_Tire_2Ind_Disk_Fiala(0.0,100.0,1e-3);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_sus_wheel_tire_2ind_disk_fiala.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (auto items : *sim1.m_sptr_store) {
		for (auto item : items) {
			of<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();
}

void run_Pass14DOF(string par_file,string output_file) {
	std::ofstream of;
	Simulator_Pass14DOF sim1 = Simulator_Pass14DOF(0);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open(output_file);
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (auto items : *sim1.m_sptr_store) {
		for (auto item : items) {
			of<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();
}

int main(int argc, char *argv[]){
	string par_file = "data/parameters/parameters_0.json";
	string output_file = "data/outputs/res_pass14dof_0.csv";
	if (argc>2) {
		par_file = argv[1];
		output_file = argv[2];
	} else {
		cout<<"arguments less than 2 so the default is used."<<endl;
	}
	//run_Whl_4Disk();
	//run_Tir_4Fiala();
	//run_Sus_2Ind();
	//run_Vehicle_Body();
	//run_Wheel_Tire_4Disk_Fiala();
	//run_Sus_Wheel_Tire_2Ind_Disk_Fiala();
	//run_Chassis_2Ind_Disk_Fiala();
	run_Pass14DOF(par_file,output_file);
	return 0;
}