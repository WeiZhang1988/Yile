#ifndef SIMULATOR_CONNECTED_HPP
#define SIMULATOR_CONNECTED_HPP

#include <boost/numeric/odeint.hpp>
#include <iostream>
#include "components/common.hpp"
#include "components/wheel.hpp"
#include "components/tire_fiala.hpp"
#include "systems/system_connected.hpp"

using namespace boost::numeric::odeint;
using namespace std;

class Simulator_Connected {
public:
	Simulator_Connected(double t_start, double t_end, double t_step);
	void run ();

    shared_ptr<System_Connected> m_sptr_sys;
    
    shared_ptr<Wheel> m_sptr_whl_fl;
    shared_ptr<Wheel> m_sptr_whl_fr;
    shared_ptr<Wheel> m_sptr_whl_rl;
    shared_ptr<Wheel> m_sptr_whl_rr;
    
    shared_ptr<Tire_Fiala> m_sptr_tr_fl;
    shared_ptr<Tire_Fiala> m_sptr_tr_fr;
    shared_ptr<Tire_Fiala> m_sptr_tr_rl;
    shared_ptr<Tire_Fiala> m_sptr_tr_rr;
    
    shared_ptr<d_vec> m_sptr_inputs_connected_fl;
    shared_ptr<d_vec> m_sptr_inputs_connected_fr;
    shared_ptr<d_vec> m_sptr_inputs_connected_rl;
    shared_ptr<d_vec> m_sptr_inputs_connected_rr;
    
    size_t m_steps;
    d_v_vec m_states;
    d_vec m_times;
	
private:
	double m_t_start, m_t_end, m_t_step;
    runge_kutta4<d_vec> m_stepper;

};

#endif //SIMULATOR_CONNECTED_HPP
