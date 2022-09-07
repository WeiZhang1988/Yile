#include "system_4wheels.hpp"

void System_4Wheels::add_wheels(\
std::shared_ptr<Wheel> sptr_whl_fl, \
std::shared_ptr<Wheel> sptr_whl_fr, \
std::shared_ptr<Wheel> sptr_whl_rl, \
std::shared_ptr<Wheel> sptr_whl_rr){
	m_sptr_whl_fl = sptr_whl_fl;
	m_sptr_whl_fr = sptr_whl_fr;
	m_sptr_whl_rl = sptr_whl_rl;
	m_sptr_whl_rr = sptr_whl_rr;
}
	
void System_4Wheels::add_wheels_inputs(\
std::shared_ptr<d_vec> sptr_inputs_whl_fl, \
std::shared_ptr<d_vec> sptr_inputs_whl_fr, \
std::shared_ptr<d_vec> sptr_inputs_whl_rl, \
std::shared_ptr<d_vec> sptr_inputs_whl_rr) {
	m_sptr_inputs_whl_fl = sptr_inputs_whl_fl;
	m_sptr_inputs_whl_fr = sptr_inputs_whl_fr;
	m_sptr_inputs_whl_rl = sptr_inputs_whl_rl;
	m_sptr_inputs_whl_rr = sptr_inputs_whl_rr;
}


void System_4Wheels::preprocess(const double &t) {
	m_t = t;
	
	m_sptr_whl_fl->preupdate(*m_sptr_inputs_whl_fl, m_t);
	m_sptr_whl_fr->preupdate(*m_sptr_inputs_whl_fr, m_t);
	m_sptr_whl_rl->preupdate(*m_sptr_inputs_whl_rl, m_t);
	m_sptr_whl_rr->preupdate(*m_sptr_inputs_whl_rr, m_t);
}

void System_4Wheels::collect_con_states() {
	m_system_con_states[0] = m_sptr_whl_fl->m_con_states[0];
	m_system_con_states[1] = m_sptr_whl_fr->m_con_states[0];
	m_system_con_states[2] = m_sptr_whl_rl->m_con_states[0];
	m_system_con_states[3] = m_sptr_whl_rr->m_con_states[0];
}

void System_4Wheels::calculate_and_collect_derivatives() {
	m_sptr_whl_fl->calculate_derivatives(m_system_drvs[0]);
	m_sptr_whl_fr->calculate_derivatives(m_system_drvs[1]);
	m_sptr_whl_rl->calculate_derivatives(m_system_drvs[2]);
	m_sptr_whl_rr->calculate_derivatives(m_system_drvs[3]);
}

void System_4Wheels::distribute_con_states() {
	m_sptr_whl_fl->m_con_states[0] = m_system_con_states[0];
	m_sptr_whl_fr->m_con_states[0] = m_system_con_states[1];
	m_sptr_whl_rl->m_con_states[0] = m_system_con_states[2];
	m_sptr_whl_rr->m_con_states[0] = m_system_con_states[3];
}

void System_4Wheels::postprocess() {
	m_sptr_whl_fl->postupdate();
	m_sptr_whl_fr->postupdate();
	m_sptr_whl_rl->postupdate();
	m_sptr_whl_rr->postupdate();
}

void System_4Wheels::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
void System_4Wheels::derivative (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
