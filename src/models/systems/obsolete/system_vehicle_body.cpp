#include "system_vehicle_body.hpp"

void NMSPC::System_Vehicle_Body::feed_inputs(const d_vec &u) {
	m_vhlbdy_inputs = u;
}

void NMSPC::System_Vehicle_Body::preprocess(const double &t) {
	m_t = t;
	m_sptr_vhlbdy->preupdate(m_vhlbdy_inputs, t);
}

void NMSPC::System_Vehicle_Body::collect_con_states() {
	std::copy(m_sptr_vhlbdy->m_con_states.begin(), \
	m_sptr_vhlbdy->m_con_states.end(), \
	m_system_con_states.begin());
}

void NMSPC::System_Vehicle_Body::calculate_and_collect_derivatives() {
	m_sptr_vhlbdy->calculate_derivatives(m_system_drvs[0], m_system_drvs[1], \
	m_system_drvs[2], m_system_drvs[3], m_system_drvs[4], \
	m_system_drvs[5], m_system_drvs[6], m_system_drvs[7], \
	m_system_drvs[8], m_system_drvs[9], m_system_drvs[10], m_system_drvs[11]);
}

void NMSPC::System_Vehicle_Body::distribute_con_states() {
	std::copy(m_system_con_states.begin(), \
	m_system_con_states.end(), \
	m_sptr_vhlbdy->m_con_states.begin());
}

void NMSPC::System_Vehicle_Body::postprocess() {
	m_sptr_vhlbdy->postupdate();
}

void NMSPC::System_Vehicle_Body::output() {
	m_sptr_vhlbdy->output(m_system_outputs, m_output_DCM);
}

void NMSPC::System_Vehicle_Body::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
void NMSPC::System_Vehicle_Body::derivative (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
