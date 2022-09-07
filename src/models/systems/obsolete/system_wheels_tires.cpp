#include "system_wheels_tires.hpp"

void System_Wheels_Tires::add_wheel_tire(\
std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whl_tr_fl, \
std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whl_tr_fr, \
std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whl_tr_rl, \
std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whl_tr_rr){
	m_sptr_whl_tr_fl = sptr_whl_tr_fl;
	m_sptr_whl_tr_fr = sptr_whl_tr_fr;
	m_sptr_whl_tr_rl = sptr_whl_tr_rl;
	m_sptr_whl_tr_rr = sptr_whl_tr_rr;
}
	
void System_Wheels_Tires::add_wheel_tire_inputs(\
std::shared_ptr<d_vec> sptr_inputs_whl_tr_fl, \
std::shared_ptr<d_vec> sptr_inputs_whl_tr_fr, \
std::shared_ptr<d_vec> sptr_inputs_whl_tr_rl, \
std::shared_ptr<d_vec> sptr_inputs_whl_tr_rr) {
	m_sptr_inputs_whl_tr_fl = sptr_inputs_whl_tr_fl;
	m_sptr_inputs_whl_tr_fr = sptr_inputs_whl_tr_fr;
	m_sptr_inputs_whl_tr_rl = sptr_inputs_whl_tr_rl;
	m_sptr_inputs_whl_tr_rr = sptr_inputs_whl_tr_rr;
}


void System_Wheels_Tires::preprocess(const double &t) {
	m_t = t;
	
	m_sptr_whl_tr_fl->preupdate(*m_sptr_inputs_whl_tr_fl, m_t);
	m_sptr_whl_tr_fr->preupdate(*m_sptr_inputs_whl_tr_fr, m_t);
	m_sptr_whl_tr_rl->preupdate(*m_sptr_inputs_whl_tr_rl, m_t);
	m_sptr_whl_tr_rr->preupdate(*m_sptr_inputs_whl_tr_rr, m_t);
}

void System_Wheels_Tires::collect_con_states() {
	//front left
	std::copy(m_sptr_whl_tr_fl->m_con_states.begin(), \
	m_sptr_whl_tr_fl->m_con_states.end(), \
	m_system_con_states.begin());
	//front right
	std::copy(m_sptr_whl_tr_fr->m_con_states.begin(), \
	m_sptr_whl_tr_fr->m_con_states.end(), \
	m_system_con_states.begin() + \
	m_total_con_states_num / m_whl_tr_nums);
	//rear left
	std::copy(m_sptr_whl_tr_rl->m_con_states.begin(), \
	m_sptr_whl_tr_rl->m_con_states.end(), \
	m_system_con_states.begin() + \
	2 * m_total_con_states_num / m_whl_tr_nums);
	//rear right
	std::copy(m_sptr_whl_tr_rr->m_con_states.begin(), \
	m_sptr_whl_tr_rr->m_con_states.end(), \
	m_system_con_states.begin() + \
	3 * m_total_con_states_num / m_whl_tr_nums);
}

void System_Wheels_Tires::calculate_and_collect_derivatives() {
	m_sptr_whl_tr_fl->calculate_derivatives(m_system_drvs[0], \
	m_system_drvs[1], m_system_drvs[2], m_system_drvs[3]);
	m_sptr_whl_tr_fr->calculate_derivatives(m_system_drvs[4], \
	m_system_drvs[5], m_system_drvs[6], m_system_drvs[7]);
	m_sptr_whl_tr_rl->calculate_derivatives(m_system_drvs[8], \
	m_system_drvs[9], m_system_drvs[10], m_system_drvs[11]);
	m_sptr_whl_tr_rr->calculate_derivatives(m_system_drvs[12], \
	m_system_drvs[13], m_system_drvs[14], m_system_drvs[15]);
}

void System_Wheels_Tires::distribute_con_states() {
	//front left
	std::copy(m_system_con_states.begin(), \
	m_system_con_states.begin() + \
	m_total_con_states_num / m_whl_tr_nums, \
	m_sptr_whl_tr_fl->m_con_states.begin());
	//front right
	std::copy(m_system_con_states.begin() + \
	m_total_con_states_num / m_whl_tr_nums, \
	m_system_con_states.begin() + \
	2 * m_total_con_states_num / m_whl_tr_nums, \
	m_sptr_whl_tr_fr->m_con_states.begin());
	//rear left
	std::copy(m_system_con_states.begin() + \
	2 * m_total_con_states_num / m_whl_tr_nums, \
	m_system_con_states.begin() + \
	3 * m_total_con_states_num / m_whl_tr_nums, \
	m_sptr_whl_tr_rl->m_con_states.begin());
	//rear right
	std::copy(m_system_con_states.begin() + \
	3 * m_total_con_states_num / m_whl_tr_nums, \
	m_system_con_states.end(), \
	m_sptr_whl_tr_rr->m_con_states.begin());
}

void System_Wheels_Tires::postprocess() {
	m_sptr_whl_tr_fl->postupdate();
	m_sptr_whl_tr_fr->postupdate();
	m_sptr_whl_tr_rl->postupdate();
	m_sptr_whl_tr_rr->postupdate();
}

void System_Wheels_Tires::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
void System_Wheels_Tires::derivative (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
