#include "system_suspensions_2macpherson.hpp"

void NMSPC::System_Suspensions_2Macpherson::feed_inputs(const d_vec &u) {
	m_sb2mpsn_inputs = u;
}

void NMSPC::System_Suspensions_2Macpherson::preprocess(const double &t) {
	m_t = t;
	m_sptr_sb2mpsn->preprocess(m_sb2mpsn_inputs, t);
}

void NMSPC::System_Suspensions_2Macpherson::collect_con_states() {
	m_sptr_sb2mpsn->collect_con_states();

	std::copy(m_sptr_sb2mpsn->m_con_states.begin(), \
	m_sptr_sb2mpsn->m_con_states.end(), \
	m_system_con_states.begin());
}

void NMSPC::System_Suspensions_2Macpherson::calculate_and_collect_derivatives() {
	m_sptr_sb2mpsn->calculate_and_collect_derivatives();
	
	std::copy(m_sptr_sb2mpsn->m_drvs.begin(), \
	m_sptr_sb2mpsn->m_drvs.end(), \
	m_system_drvs.begin());
}

void NMSPC::System_Suspensions_2Macpherson::distribute_con_states() {
	std::copy(m_system_con_states.begin(), \
	m_system_con_states.end(), \
	m_sptr_sb2mpsn->m_con_states.begin());
	
	m_sptr_sb2mpsn->distribute_con_states();
}

void NMSPC::System_Suspensions_2Macpherson::postprocess() {
	m_sptr_sb2mpsn->postprocess();
}

void NMSPC::System_Suspensions_2Macpherson::output() {
	m_sptr_sb2mpsn->output(m_system_outputs);
}

void NMSPC::System_Suspensions_2Macpherson::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
void NMSPC::System_Suspensions_2Macpherson::derivative (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
