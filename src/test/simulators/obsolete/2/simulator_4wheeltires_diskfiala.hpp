#ifndef SIMULATOR_4WHEELTIRES_DISKFIALA_HPP
#define SIMULATOR_4WHEELTIRES_DISKFIALA_HPP

#include <boost/numeric/odeint.hpp>
#include <iostream>
#include "components/common.hpp"
#include "components/wheel_disk_tire_fiala.hpp"
#include "subsystems/subsystem_4wheeltires_diskfiala.hpp"
#include "systems/system_4wheeltires_diskfiala.hpp"

using namespace boost::numeric::odeint;
using namespace std;
using namespace name_space;

class Simulator_4WheelTires_DiskFiala {
public:
	Simulator_4WheelTires_DiskFiala(double t_start, double t_end, double t_step);
	void run ();

    shared_ptr<System_4WheelTires_DiskFiala> m_sptr_sys;
    shared_ptr<Subsystem_4WheelTires_DiskFiala> m_sptr_sb4whltr;
    shared_ptr<Wheel_Disk_Tire_Fiala> m_sptr_whl_tr_fl;
    shared_ptr<Wheel_Disk_Tire_Fiala> m_sptr_whl_tr_fr;
    shared_ptr<Wheel_Disk_Tire_Fiala> m_sptr_whl_tr_rl;
    shared_ptr<Wheel_Disk_Tire_Fiala> m_sptr_whl_tr_rr;

    size_t m_steps;
    d_v_vec m_states, m_outputs;
    d_vec m_times;
	
private:
	double m_t_start, m_t_end, m_t_step;
    runge_kutta4<d_vec> m_stepper;

};

#endif //SIMULATOR_WHEELS_TIRES_HPP
