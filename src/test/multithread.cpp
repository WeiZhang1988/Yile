#include <thread>
#include <iostream>
#include <unistd.h>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <deque>
#include "simulator_pass14dof.hpp"

std::vector<double> zero_inputs{	eps,eps,eps,eps,  //1-StrgAng 0000
									eps,eps,eps,eps,  //2-AxlTrq 0000
									eps,eps,eps,eps,  //3-BrkPrs 0000
									eps,eps,eps,	  //4-WindXYZ 000
									eps,eps,eps,eps,  //5-Ground 0000
									1.0,1.0,1.0,1.0,  //6-Friction 111
									2.2e5,2.2e5,2.2e5,2.2e5, //Tire Pressure 220000
									273.0,// Air temperature Constant: Tair=273
									eps,eps,eps,eps,// Tire temperature Constant: Tamb=0
									eps,eps,eps,//Extern Fx, Fy, Fz
									eps,eps,eps};//Extern Mx, My, Mz
std::deque<std::vector<double> > inputs;
std::mutex mu;
std::condition_variable cond;

bool sim_over = false;

void inputs_pthread()
{
	int count = 0;
	while(!sim_over)
	{
		std::unique_lock<std::mutex> locker(mu,std::defer_lock);
		locker.lock();
		inputs.push_back(zero_inputs);
		cond.notify_one();
		locker.unlock();
	}
}

void vehicle_pthread()
{
	
	double time_start = 0.0;
	double time_end   = 100.0;
	double time_step  = 5e-4;
	int num_step   = (time_end-time_start)/time_step;
	Simulator_Pass14DOF sim1(time_start,time_end,time_step);
	sim1.m_sptr_sys->push_con_states(sim1.m_sptr_sys->m_con_states);
	while(1)
	{
		std::unique_lock<std::mutex> locker(mu);	
		//从inputs中读取数据
		
		cond.wait(locker,[](){return !inputs.empty();}); //解锁
		
		sim1.step(inputs.front());
		inputs.pop_front();
		locker.unlock();
		printf("vbx = %10.8lf,vby = %10.8lf,vbz = %10.8lf\n",(*sim1.m_sptr_store).back().at(0),(*sim1.m_sptr_store).back().at(1),(*sim1.m_sptr_store).back().at(2));
		if(sim1.m_t_current>=time_end) 
		{
			printf("Simulation is over...\n");
			sim_over = true;
			break;
		}
	}
}
/*
void carla_pthread()
{
	while(1)
	{
		std::unique_lock<std::mutex> locker(mu);
		//从inputs中读取数据
		while(inputs.empty())
		{
			cond.wait(locker); //解锁
		}
		inputs.push_back({1,2,3,4,5,6,7,8,9,10,11,...,38});
		cond.notify_one();
		locker.unlock();
	}
}
*/





int main()
{	
	std::thread t1(inputs_pthread);
	std::thread t2(vehicle_pthread);
	/*std::thread t3(carla_pthread);*/

	t1.join();
	t2.join();
	//t3.join();
	return 0;
}