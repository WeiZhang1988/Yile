#ifndef SIMULATOR_4TIRES_HPP
#define SIMULATOR_4TIRES_HPP

#include <boost/numeric/odeint.hpp>
#include <iostream>
#include "components/common.hpp"
#include "components/tire_fiala.hpp"
#include "systems/system_4tires.hpp"

using namespace boost::numeric::odeint;
using namespace std;

class Simulator_4Tires {
public:
	Simulator_4Tires(double t_start, double t_end, double t_step);
	void run ();

    shared_ptr<System_4Tires> m_sptr_sys;
    
    shared_ptr<Tire_Fiala> m_sptr_tr_fl;
    shared_ptr<Tire_Fiala> m_sptr_tr_fr;
    shared_ptr<Tire_Fiala> m_sptr_tr_rl;
    shared_ptr<Tire_Fiala> m_sptr_tr_rr;
    
    shared_ptr<d_vec> m_sptr_inputs_tr_fl;
    shared_ptr<d_vec> m_sptr_inputs_tr_fr;
    shared_ptr<d_vec> m_sptr_inputs_tr_rl;
    shared_ptr<d_vec> m_sptr_inputs_tr_rr;
    
    size_t m_steps;
    d_v_vec m_states;
    d_vec m_times;
	
private:
	double m_t_start, m_t_end, m_t_step;
    runge_kutta4<d_vec> m_stepper;

};

#endif //SIMULATOR_4TIRES_HPP
