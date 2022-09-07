#include "simulator_vehicle_body.hpp"

Simulator_Vehicle_Body::Simulator_Vehicle_Body(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;

	m_sptr_vhlbdy = make_shared<Vehicle_Body>();
    m_sptr_sys = make_shared<System_Vehicle_Body>(t_start);
    
    m_sptr_sys->add_vehicle_body(m_sptr_vhlbdy);
}

void Simulator_Vehicle_Body::run () {

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
