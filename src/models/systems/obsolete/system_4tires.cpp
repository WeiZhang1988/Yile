#include "system_4tires.hpp"

void System_4Tires::add_tires(\
std::shared_ptr<Tire_Fiala> sptr_tr_fl, \
std::shared_ptr<Tire_Fiala> sptr_tr_fr, \
std::shared_ptr<Tire_Fiala> sptr_tr_rl, \
std::shared_ptr<Tire_Fiala> sptr_tr_rr){
	m_sptr_tr_fl = sptr_tr_fl;
	m_sptr_tr_fr = sptr_tr_fr;
	m_sptr_tr_rl = sptr_tr_rl;
	m_sptr_tr_rr = sptr_tr_rr;
}
	
void System_4Tires::add_tires_inputs(\
std::shared_ptr<d_vec> sptr_inputs_tr_fl, \
std::shared_ptr<d_vec> sptr_inputs_tr_fr, \
std::shared_ptr<d_vec> sptr_inputs_tr_rl, \
std::shared_ptr<d_vec> sptr_inputs_tr_rr) {
	m_sptr_inputs_tr_fl = sptr_inputs_tr_fl;
	m_sptr_inputs_tr_fr = sptr_inputs_tr_fr;
	m_sptr_inputs_tr_rl = sptr_inputs_tr_rl;
	m_sptr_inputs_tr_rr = sptr_inputs_tr_rr;
}


void System_4Tires::preprocess(const double &t) {
	m_t = t;
	
	m_sptr_tr_fl->preupdate(*m_sptr_inputs_tr_fl, m_t);
	m_sptr_tr_fr->preupdate(*m_sptr_inputs_tr_fr, m_t);
	m_sptr_tr_rl->preupdate(*m_sptr_inputs_tr_rl, m_t);
	m_sptr_tr_rr->preupdate(*m_sptr_inputs_tr_rr, m_t);
}

void System_4Tires::collect_con_states() {
	m_system_con_states[0] = m_sptr_tr_fl->m_con_states[0];
	m_system_con_states[1] = m_sptr_tr_fl->m_con_states[1];
	m_system_con_states[2] = m_sptr_tr_fl->m_con_states[2];
	
	m_system_con_states[3] = m_sptr_tr_fr->m_con_states[0];
	m_system_con_states[4] = m_sptr_tr_fr->m_con_states[1];
	m_system_con_states[5] = m_sptr_tr_fr->m_con_states[2];
	
	m_system_con_states[6] = m_sptr_tr_rl->m_con_states[0];
	m_system_con_states[7] = m_sptr_tr_rl->m_con_states[1];
	m_system_con_states[8] = m_sptr_tr_rl->m_con_states[2];
	
	m_system_con_states[9]  = m_sptr_tr_rr->m_con_states[0];
	m_system_con_states[10] = m_sptr_tr_rr->m_con_states[1];
	m_system_con_states[11] = m_sptr_tr_rr->m_con_states[2];
}

void System_4Tires::calculate_and_collect_derivatives() {
	m_sptr_tr_fl->calculate_derivatives(m_system_drvs[0], \
	m_system_drvs[1], \
	m_system_drvs[2]);
	m_sptr_tr_fr->calculate_derivatives(m_system_drvs[3],\
	m_system_drvs[4], \
	m_system_drvs[5]);
	m_sptr_tr_rl->calculate_derivatives(m_system_drvs[6], \
	m_system_drvs[7], \
	m_system_drvs[8]);
	m_sptr_tr_rr->calculate_derivatives(m_system_drvs[9], \
	m_system_drvs[10], \
	m_system_drvs[11]);
}

void System_4Tires::distribute_con_states() {
	m_sptr_tr_fl->m_con_states[0] = m_system_con_states[0];
	m_sptr_tr_fl->m_con_states[1] = m_system_con_states[1];
	m_sptr_tr_fl->m_con_states[2] = m_system_con_states[2];
	
	m_sptr_tr_fr->m_con_states[0] = m_system_con_states[3];
	m_sptr_tr_fr->m_con_states[1] = m_system_con_states[4];
	m_sptr_tr_fr->m_con_states[2] = m_system_con_states[5];
	
	m_sptr_tr_rl->m_con_states[0] = m_system_con_states[6];
	m_sptr_tr_rl->m_con_states[1] = m_system_con_states[7];
	m_sptr_tr_rl->m_con_states[2] = m_system_con_states[8];
	
	m_sptr_tr_rr->m_con_states[0] = m_system_con_states[9];
	m_sptr_tr_rr->m_con_states[1] = m_system_con_states[10];
	m_sptr_tr_rr->m_con_states[2] = m_system_con_states[11];
}

void System_4Tires::postprocess() {
	m_sptr_tr_fl->postupdate();
	m_sptr_tr_fr->postupdate();
	m_sptr_tr_rl->postupdate();
	m_sptr_tr_rr->postupdate();
}

void System_4Tires::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
void System_4Tires::derivative (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
