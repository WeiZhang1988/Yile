#include "simulator_4wheels.hpp"

Simulator_4Wheels::Simulator_4Wheels(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	
	m_sptr_sys = make_shared<System_4Wheels>(t_start);
	
	m_sptr_whl_fl = make_shared<Wheel>(0.9, 1e-3, 0.05, 2.0, \
	0.177, 0.2, 0.3, 0.0, false, t_start);
    m_sptr_whl_fr = make_shared<Wheel>(0.9, 1e-3, 0.05, 2.0, \
    0.177, 0.2, 0.3, 0.0, false, t_start);
    m_sptr_whl_rl = make_shared<Wheel>(0.9, 1e-3, 0.05, 2.0, \
    0.177, 0.2, 0.3, 0.0, false, t_start);
    m_sptr_whl_rr = make_shared<Wheel>(0.9, 1e-3, 0.05, 2.0, \
    0.177, 0.2, 0.3, 0.0, false, t_start);
    
    d_vec inputs_whl_fl = {0.0, 10.0, 0.0, 0.0, 0.0};
    d_vec inputs_whl_fr = {0.0, 50.0, 0.0, 0.0, 0.0};
    d_vec inputs_whl_rl = {0.0, 100.0, 0.0, 0.0, 0.0};
    d_vec inputs_whl_rr = {0.0, 200.0, 0.0, 0.0, 0.0};
	
	m_sptr_inputs_whl_fl = make_shared<d_vec>(inputs_whl_fl);
    m_sptr_inputs_whl_fr = make_shared<d_vec>(inputs_whl_fr);
    m_sptr_inputs_whl_rl = make_shared<d_vec>(inputs_whl_rl);
    m_sptr_inputs_whl_rr = make_shared<d_vec>(inputs_whl_rr);

	m_sptr_sys->add_wheels(m_sptr_whl_fl, m_sptr_whl_fr, m_sptr_whl_rl, m_sptr_whl_rr);
	m_sptr_sys->add_wheels_inputs(m_sptr_inputs_whl_fl, m_sptr_inputs_whl_fr, m_sptr_inputs_whl_rl, m_sptr_inputs_whl_rr);
}

void Simulator_4Wheels::run () {

	int steps_num = static_cast<int>((m_t_end - m_t_start) / m_t_step);
        
	double t = m_t_start;
	for (int i=0; i<steps_num; i++) {
		m_steps++;	
		m_times.push_back(t);
		
		m_sptr_sys->preprocess(t);
		m_sptr_sys->collect_con_states();
		m_states.push_back(m_sptr_sys->m_system_con_states);
		m_sptr_sys->calculate_and_collect_derivatives();
		m_stepper.do_step(*m_sptr_sys,m_sptr_sys->m_system_con_states,t,m_t_step);
		m_sptr_sys->distribute_con_states();
 		m_sptr_sys->postprocess();
		
		t += m_t_step;
	}
	m_times.push_back(t);
	m_states.push_back(m_sptr_sys->m_system_con_states);
}
