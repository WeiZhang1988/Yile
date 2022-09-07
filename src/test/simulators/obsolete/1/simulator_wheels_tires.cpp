#include "simulator_wheels_tires.hpp"

Simulator_Wheels_Tires::Simulator_Wheels_Tires(double t_start, double t_end, double t_step) {
	m_steps = 0;
	m_t_start = t_start;
	m_t_end = t_end;
	m_t_step = t_step;
	
	m_sptr_sys = make_shared<System_Wheels_Tires>(t_start);
	
	m_sptr_whl_tr_fl = make_shared<Wheel_Disk_Tire_Fiala>();
    m_sptr_whl_tr_fr = make_shared<Wheel_Disk_Tire_Fiala>();
    m_sptr_whl_tr_rl = make_shared<Wheel_Disk_Tire_Fiala>();
    m_sptr_whl_tr_rr = make_shared<Wheel_Disk_Tire_Fiala>();
    
    d_vec inputs_whl_tr_fl = {0.0, 10.0, 0.0, 0.0, 1000.0, \
    0.0, 0.0, 220000.0,0.0, 1.0};
    d_vec inputs_whl_tr_fr = {0.0, 50.0, 0.0, 0.0, 1000.0, \
    0.0, 0.0, 220000.0,0.0, 1.0};
    d_vec inputs_whl_tr_rl = {0.0, 100.0, 0.0, 0.0, 1000.0, \
    0.0, 0.0, 220000.0,0.0, 1.0};
    d_vec inputs_whl_tr_rr = {0.0, 200.0, 0.0, 0.0, 1000.0, \
    0.0, 0.0, 220000.0,0.0, 1.0};
	
	m_sptr_inputs_whl_tr_fl = make_shared<d_vec>(inputs_whl_tr_fl);
    m_sptr_inputs_whl_tr_fr = make_shared<d_vec>(inputs_whl_tr_fr);
    m_sptr_inputs_whl_tr_rl = make_shared<d_vec>(inputs_whl_tr_rl);
    m_sptr_inputs_whl_tr_rr = make_shared<d_vec>(inputs_whl_tr_rr);

	m_sptr_sys->add_wheel_tire(m_sptr_whl_tr_fl, \
	m_sptr_whl_tr_fr, \
	m_sptr_whl_tr_rl, \
	m_sptr_whl_tr_rr);
	m_sptr_sys->add_wheel_tire_inputs(m_sptr_inputs_whl_tr_fl, \
	m_sptr_inputs_whl_tr_fr, \
	m_sptr_inputs_whl_tr_rl, \
	m_sptr_inputs_whl_tr_rr);
}

void Simulator_Wheels_Tires::run () {

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
