#include "simulator_4wheeltires_diskfiala.hpp"

Simulator_4WheelTires_DiskFiala::Simulator_4WheelTires_DiskFiala(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;

	m_sptr_whl_tr_fl = make_shared<Wheel_Disk_Tire_Fiala>();
    m_sptr_whl_tr_fr = make_shared<Wheel_Disk_Tire_Fiala>();
    m_sptr_whl_tr_rl = make_shared<Wheel_Disk_Tire_Fiala>();
    m_sptr_whl_tr_rr = make_shared<Wheel_Disk_Tire_Fiala>();
    m_sptr_sb4whltr = make_shared<Subsystem_4WheelTires_DiskFiala>(t_start);
    m_sptr_sys = make_shared<System_4WheelTires_DiskFiala>(t_start);
    
    m_sptr_sb4whltr->add_wheeltires(m_sptr_whl_tr_fl, \
    m_sptr_whl_tr_fr, m_sptr_whl_tr_rl, m_sptr_whl_tr_rr);
    m_sptr_sys->add_subsystems(m_sptr_sb4whltr);
}

void Simulator_4WheelTires_DiskFiala::run () {

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
        
	double t = m_t_start;
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		
		m_sptr_sys->preprocess(t);
		m_sptr_sys->collect_con_states();
		m_sptr_sys->output();
		m_outputs.push_back(m_sptr_sys->m_system_outputs);
		m_sptr_sys->calculate_and_collect_derivatives();
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_system_con_states,t,m_t_step);
		m_sptr_sys->distribute_con_states();
 		m_sptr_sys->postprocess();
		
		t += m_t_step;
	}
	m_times.push_back(t);
	m_sptr_sys->preprocess(t);
	m_sptr_sys->output();
	m_outputs.push_back(m_sptr_sys->m_system_outputs);
}
