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

class Thread_Pool {
public:
    Thread_Pool(int thrd_num) : m_running_flag(false) {
        //arg: number of threads
        for (int i=0;i<thrd_num;i++) {
            m_threads.emplace_back(&Thread_Pool::work, this);
        }
    }
    ~Thread_Pool() {
        stop();
    }
    void start() {
        m_running_flag = true;
    }
    void stop() {
        m_running_flag = false;
        m_cv.notify_all();
        for (auto &th : m_threads) {
            if (th.joinable()) {
                th.join();
            }
        }
    }
    void push_task(Task *task) {
        std::unique_lock<std::mutex> locker(this->m_mu);
        this->m_tasks.push(task);
        locker.unlock();
        this->m_cv.notify_one();
    }
private:
    void work() {
        while(m_running_flag) {
            Task *task = nullptr;
            {
                std::unique_lock<std::mutex> locker(this->m_mu);
                while (this->m_tasks.empty()) {
                    m_cv.wait(locker);
                }
                if (m_running_flag) {
                    task = this->m_tasks.front();
                    this->m_tasks.pop();
                }
            }
            if (task!=nullptr) {
                task->run();
            }
        }
    }
    std::vector<std::thread> m_threads;
    std::queue<Task*> m_tasks;
    std::mutex m_mu;
    std::atomic_bool m_running_flag;
    std::condition_variable m_cv;
};

void run_Pass14DOF(int thrd_num=1) {
	Thread_Pool pool(thrd_num);
    pool.start();
    for (int i=0;i<thrd_num;i++) {
		Simulator_Pass14DOF *sim = new Simulator_Pass14DOF(i);
        pool.push_task(sim);
    }
	getchar();
}

int main(int argc, char *argv[]){
	int car_num = 1;
	if (argc>1) {
		car_num = atoi(argv[1]);
		car_num = car_num>1?car_num:1;
		car_num = car_num<3?car_num:3;
		cout<< car_num <<" cars are created."<<endl;
	} else {
		cout<<"1 car is created."<<endl;
	}
	run_Pass14DOF(car_num);
	return 0;
}
