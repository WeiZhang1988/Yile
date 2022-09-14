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
#include "simulators/simulator_whl_4disk.hpp"

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
	//run_Whl_Disk_Tir_Fiala();
	//run_Tir_4Fiala();
	//run_Sus_2Ind();
	//run_Vehicle_Body();
	run_Whl_4Disk();
	//run_LPF();
	return 0;
}
