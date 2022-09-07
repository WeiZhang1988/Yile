#include "system_4wheeltires_diskfiala.hpp"

void NMSPC::System_4WheelTires_DiskFiala::feed_inputs(const d_vec &u) {
	m_sb4whltr_inputs = u;
}

void NMSPC::System_4WheelTires_DiskFiala::preprocess(const double &t) {
	m_t = t;
	m_sptr_sb4whltr->preprocess(m_sb4whltr_inputs, t);
}

void NMSPC::System_4WheelTires_DiskFiala::collect_con_states() {
	m_sptr_sb4whltr->collect_con_states();

	std::copy(m_sptr_sb4whltr->m_con_states.begin(), \
	m_sptr_sb4whltr->m_con_states.end(), \
	m_system_con_states.begin());
}

void NMSPC::System_4WheelTires_DiskFiala::calculate_and_collect_derivatives() {
	m_sptr_sb4whltr->calculate_and_collect_derivatives();
	
	std::copy(m_sptr_sb4whltr->m_drvs.begin(), \
	m_sptr_sb4whltr->m_drvs.end(), \
	m_system_drvs.begin());
}

void NMSPC::System_4WheelTires_DiskFiala::distribute_con_states() {
	std::copy(m_system_con_states.begin(), \
	m_system_con_states.end(), \
	m_sptr_sb4whltr->m_con_states.begin());
	
	m_sptr_sb4whltr->distribute_con_states();
}

void NMSPC::System_4WheelTires_DiskFiala::postprocess() {
	m_sptr_sb4whltr->postprocess();
}

void NMSPC::System_4WheelTires_DiskFiala::output() {
	m_sptr_sb4whltr->output(m_system_outputs);
}

void NMSPC::System_4WheelTires_DiskFiala::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
void NMSPC::System_4WheelTires_DiskFiala::derivative (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
