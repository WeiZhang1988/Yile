#include "simulator_suspensions_2macpherson.hpp"

Simulator_Suspensions_2Macpherson::Simulator_Suspensions_2Macpherson(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;

	m_sptr_sus_f = make_shared<Sus_Macpherson_2Tracks>();
    m_sptr_sus_r = make_shared<Sus_Macpherson_2Tracks>();

    m_sptr_sb2mpsn = make_shared<Subsystem_Suspensions_2Macpherson>(t_start);
    m_sptr_sys = make_shared<System_Suspensions_2Macpherson>(t_start);
    
    m_sptr_sb2mpsn->add_suspensions(m_sptr_sus_f, m_sptr_sus_r);
    m_sptr_sys->add_subsystems(m_sptr_sb2mpsn);
}

void Simulator_Suspensions_2Macpherson::run () {

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
