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
#include <filesystem>
#include <thread>
#include <iostream>
#include <vector>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <boost/property_tree/json_parser.hpp>
#include "simulator_pass14dof.hpp"

using namespace Yile;

void run_Pass14DOF() {
    shared_ptr<Simulator_Pass14DOF> sim = make_shared<Simulator_Pass14DOF>();
    sim->run();
}

int main(int argc, char *argv[]){
	run_Pass14DOF();
	return 0;
}
