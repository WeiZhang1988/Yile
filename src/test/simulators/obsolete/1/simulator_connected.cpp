#include "simulator_connected.hpp"

Simulator_Connected::Simulator_Connected(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	
	m_sptr_sys = make_shared<System_Connected>(t_start);	//m_sptr_sys = new System_Connected(t_start)
	
	m_sptr_whl_fl = make_shared<Wheel>(0.9, 1e-3, 0.05, 2.0, \
	0.177, 0.2, 0.3, 0.0, false, t_start);
    m_sptr_whl_fr = make_shared<Wheel>(0.9, 1e-3, 0.05, 2.0, \
    0.177, 0.2, 0.3, 0.0, false, t_start);
    m_sptr_whl_rl = make_shared<Wheel>(0.9, 1e-3, 0.05, 2.0, \
    0.177, 0.2, 0.3, 0.0, false, t_start);
    m_sptr_whl_rr = make_shared<Wheel>(0.9, 1e-3, 0.05, 2.0, \
    0.177, 0.2, 0.3, 0.0, false, t_start);
    
    m_sptr_tr_fl = \
    make_shared<Tire_Fiala>(0.31,0.05,0.15,-1.5,1.5,0.5,0.8,8e-4,1e-3,1.6e-4,-3e-3,0.97,10.0,1e4,1e7,4.5e4,0.0,0.21,1e3,0.0,0.0,0.0,t_start);
    m_sptr_tr_fr = \
    make_shared<Tire_Fiala>(0.31,0.05,0.15,-1.5,1.5,0.5,0.8,8e-4,1e-3,1.6e-4,-3e-3,0.97,10.0,1e4,1e7,4.5e4,0.0,0.21,1e3,0.0,0.0,0.0,t_start);
    m_sptr_tr_rl = \
    make_shared<Tire_Fiala>(0.31,0.05,0.15,-1.5,1.5,0.5,0.8,8e-4,1e-3,1.6e-4,-3e-3,0.97,10.0,1e4,1e7,4.5e4,0.0,0.21,1e3,0.0,0.0,0.0,t_start);
    m_sptr_tr_rr = \
    make_shared<Tire_Fiala>(0.31,0.05,0.15,-1.5,1.5,0.5,0.8,8e-4,1e-3,1.6e-4,-3e-3,0.97,10.0,1e4,1e7,4.5e4,0.0,0.21,1e3,0.0,0.0,0.0,t_start);
    
    d_vec inputs_connected_fl = {0.0, 10.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 220000.0, 0.0, 1.0};
    d_vec inputs_connected_fr = {0.0, 50.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 220000.0, 0.0, 1.0};
    d_vec inputs_connected_rl = {0.0, 100.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 220000.0, 0.0, 1.0};
    d_vec inputs_connected_rr = {0.0, 200.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 220000.0, 0.0, 1.0};
	
	m_sptr_inputs_connected_fl = make_shared<d_vec>(inputs_connected_fl);
    m_sptr_inputs_connected_fr = make_shared<d_vec>(inputs_connected_fr);
    m_sptr_inputs_connected_rl = make_shared<d_vec>(inputs_connected_rl);
    m_sptr_inputs_connected_rr = make_shared<d_vec>(inputs_connected_rr);

	m_sptr_sys->add_wheels(m_sptr_whl_fl, m_sptr_whl_fr, m_sptr_whl_rl, m_sptr_whl_rr);
	m_sptr_sys->add_tires(m_sptr_tr_fl, m_sptr_tr_fr, m_sptr_tr_rl, m_sptr_tr_rr);
	m_sptr_sys->add_connected_inputs(m_sptr_inputs_connected_fl, m_sptr_inputs_connected_fr, m_sptr_inputs_connected_rl, m_sptr_inputs_connected_rr);
}

void Simulator_Connected::run () {

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
