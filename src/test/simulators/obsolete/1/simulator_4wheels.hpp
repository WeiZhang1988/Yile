#ifndef SIMULATOR_4WHEELS_HPP
#define SIMULATOR_4WHEELS_HPP

#include <boost/numeric/odeint.hpp>
#include <iostream>
#include "components/common.hpp"
#include "components/wheel.hpp"
#include "systems/system_4wheels.hpp"

using namespace boost::numeric::odeint;
using namespace std;

class Simulator_4Wheels {
public:
	Simulator_4Wheels(double t_start, double t_end, double t_step);
	void run ();

    shared_ptr<System_4Wheels> m_sptr_sys;
    
    shared_ptr<Wheel> m_sptr_whl_fl;
    shared_ptr<Wheel> m_sptr_whl_fr;
    shared_ptr<Wheel> m_sptr_whl_rl;
    shared_ptr<Wheel> m_sptr_whl_rr;
    
    shared_ptr<d_vec> m_sptr_inputs_whl_fl;
    shared_ptr<d_vec> m_sptr_inputs_whl_fr;
    shared_ptr<d_vec> m_sptr_inputs_whl_rl;
    shared_ptr<d_vec> m_sptr_inputs_whl_rr;
    
    size_t m_steps;
    d_v_vec m_states;
    d_vec m_times;
	
private:
	double m_t_start, m_t_end, m_t_step;
    runge_kutta4<d_vec> m_stepper;

};

#endif //SIMULATOR_4WHEELS_HPP
