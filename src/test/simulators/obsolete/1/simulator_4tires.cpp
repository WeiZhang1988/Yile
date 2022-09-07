#include "simulator_4tires.hpp"

Simulator_4Tires::Simulator_4Tires(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	
	m_sptr_sys = make_shared<System_4Tires>(t_start);
	
	m_sptr_tr_fl = make_shared<Tire_Fiala>(1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,t_start);
    m_sptr_tr_fr = make_shared<Tire_Fiala>(1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,t_start);
    m_sptr_tr_rl = make_shared<Tire_Fiala>(1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,t_start);
    m_sptr_tr_rr = make_shared<Tire_Fiala>(1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,t_start);
    
    d_vec inputs_tr_fl = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    d_vec inputs_tr_fr = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    d_vec inputs_tr_rl = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    d_vec inputs_tr_rr = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	
	m_sptr_inputs_tr_fl = make_shared<d_vec>(inputs_tr_fl);
    m_sptr_inputs_tr_fr = make_shared<d_vec>(inputs_tr_fr);
    m_sptr_inputs_tr_rl = make_shared<d_vec>(inputs_tr_rl);
    m_sptr_inputs_tr_rr = make_shared<d_vec>(inputs_tr_rr);

	m_sptr_sys->add_tires(m_sptr_tr_fl, m_sptr_tr_fr, m_sptr_tr_rl, m_sptr_tr_rr);
	m_sptr_sys->add_tires_inputs(m_sptr_inputs_tr_fl, m_sptr_inputs_tr_fr, m_sptr_inputs_tr_rl, m_sptr_inputs_tr_rr);
}

void Simulator_4Tires::run () {

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
