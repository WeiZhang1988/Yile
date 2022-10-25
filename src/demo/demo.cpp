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
#include <cstdlib>
#include <unistd.h>  
#include <thread>

void invoke_server() {
    int result = std::system("./demo_server");
}

void invoke_client() {
    int result = std::system("./demo_client");
}

int main ()  {
    std::thread t1(invoke_server);
    usleep(1e4);
	std::thread t2(invoke_client);
	t1.join();
	t2.join();
}