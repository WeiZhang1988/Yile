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
#include <fstream>
#include "simulators/simulator_whl_disk_tir_fiala.hpp"
#include "simulators/simulator_tir_4fiala.hpp"
#include "simulators/simulator_sus_2ind.hpp"
#include "simulators/simulator_vehicle_body.hpp"
#include "simulators/simulator_whl_4disk.hpp"
#include "simulators/simulator_lpf.hpp"

using namespace Yile;


void run_Whl_Disk_Tir_Fiala() {
	std::ofstream of;
	Simulator_Whl_Disk_Tir_Fiala sim1 = Simulator_Whl_Disk_Tir_Fiala(0.0,10.0,1e-4);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_wheels_tires.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (size_t i=0;i<sim1.m_times.size();i++) {
		of<<std::setprecision(7)<<sim1.m_times[i]<<',';
		for (auto item : sim1.m_outputs[i]) {
			of<<std::setw(30)<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();

}

void run_Tir_4Fiala() {
	std::ofstream of;
	Simulator_Tir_4Fiala sim1 = Simulator_Tir_4Fiala(0.0,10.0,1e-4);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_tire_4fiala.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (size_t i=0;i<sim1.m_times.size();i++) {
		of<<std::setprecision(7)<<sim1.m_times[i]<<',';
		for (auto item : sim1.m_outputs[i]) {
			of<<std::setw(30)<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();

}

void run_Sus_2Ind() {
	std::ofstream of;
	Simulator_Sus_2Ind sim1 = Simulator_Sus_2Ind(0.0,10.0,1e-4);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_sus_2ind.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (size_t i=0;i<sim1.m_times.size();i++) {
		of<<std::setprecision(7)<<sim1.m_times[i]<<',';
		for (auto item : sim1.m_outputs[i]) {
			of<<std::setw(30)<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();

}

void run_Vehicle_Body() {
	std::ofstream of;
	Simulator_Vehicle_Body sim1 = Simulator_Vehicle_Body(0.0,10.0,1e-4);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_vehicle_body.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (size_t i=0;i<sim1.m_times.size();i++) {
		of<<std::setprecision(7)<<sim1.m_times[i]<<',';
		for (auto item : sim1.m_outputs[i]) {
			of<<std::setw(30)<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();

}

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
	for (size_t i=0;i<sim1.m_times.size();i++) {
		of<<std::setprecision(7)<<sim1.m_times[i]<<',';
		for (auto item : sim1.m_outputs[i]) {
			of<<std::setw(30)<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();
	
}

void run_LPF() {
	std::ofstream of;
	Simulator_LPF sim1 = Simulator_LPF(0.0,10.0,1e-4);
	steady_clock::time_point start = steady_clock::now();
	sim1.run();
	steady_clock::time_point end = steady_clock::now();
	milliseconds dur= duration_cast<milliseconds>(end - start);
	cout<<"time elapsed in milliseconds: "<<dur.count()<<"ms"<<endl;
	
	of.open("res_lpf.csv");
	std::cout<<"total: "<<sim1.m_steps<<" steps"<<std::endl;
	for (size_t i=0;i<sim1.m_times.size();i++) {
		of<<std::setprecision(7)<<sim1.m_times[i]<<',';
		for (auto item : sim1.m_outputs[i]) {
			of<<std::setw(30)<<std::setprecision(7)<<item<<',';
		}
		of<<std::endl;
	}
	of.close();
	
}


int main(){
	run_Whl_Disk_Tir_Fiala();
	//run_Tir_4Fiala();
	//run_Sus_2Ind();
	//run_Vehicle_Body();
	//run_Whl_4Disk();
	//run_LPF();
	return 0;
}
