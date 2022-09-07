#ifndef SIMULATOR_VEHICLE_BODY_HPP
#define SIMULATOR_VEHICLE_BODY_HPP

#include <boost/numeric/odeint.hpp>
#include <iostream>
#include "components/common.hpp"
#include "components/vehicle_body.hpp"
#include "systems/system_vehicle_body.hpp"

using namespace boost::numeric::odeint;
using namespace std;
using namespace name_space;

class Simulator_Vehicle_Body {
public:
	Simulator_Vehicle_Body(double t_start, double t_end, double t_step);
	void run ();

    shared_ptr<System_Vehicle_Body> m_sptr_sys;
    shared_ptr<Vehicle_Body> m_sptr_vhlbdy;

    size_t m_steps;
    d_v_vec m_states, m_outputs;
    d_vec m_times;
	
private:
	double m_t_start, m_t_end, m_t_step;
    runge_kutta4<d_vec> m_stepper;

};

#endif //SIMULATOR_VEHICLE_BODY_HPP
