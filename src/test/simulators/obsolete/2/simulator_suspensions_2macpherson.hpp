#ifndef SIMULATOR_SUSPENSIONS_2MACPHERSON_HPP
#define SIMULATOR_SUSPENSIONS_2MACPHERSON_HPP

#include <boost/numeric/odeint.hpp>
#include <iostream>
#include "components/common.hpp"
#include "components/sus_macpherson_2tracks.hpp"
#include "subsystems/subsystem_suspensions_2macpherson.hpp"
#include "systems/system_suspensions_2macpherson.hpp"

using namespace boost::numeric::odeint;
using namespace std;
using namespace name_space;

class Simulator_Suspensions_2Macpherson {
public:
	Simulator_Suspensions_2Macpherson(double t_start, double t_end, double t_step);
	void run ();

    shared_ptr<System_Suspensions_2Macpherson> m_sptr_sys;
    shared_ptr<Subsystem_Suspensions_2Macpherson> m_sptr_sb2mpsn;
    shared_ptr<Sus_Macpherson_2Tracks> m_sptr_sus_f, m_sptr_sus_r;

    size_t m_steps;
    d_v_vec m_states, m_outputs;
    d_vec m_times;
	
private:
	double m_t_start, m_t_end, m_t_step;
    runge_kutta4<d_vec> m_stepper;

};

#endif //SIMULATOR_SUSPENSIONS_2MACPHERSON_HPP
