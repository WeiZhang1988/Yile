#include "subsystem_4wheeltires_diskfiala.hpp"

void NMSPC::Subsystem_4WheelTires_DiskFiala::add_wheeltires ( \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whltr_fl, \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whltr_fr, \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whltr_rl, \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whltr_rr) {
	m_sptr_whltr_fl = sptr_whltr_fl;
	m_sptr_whltr_fr = sptr_whltr_fr;
	m_sptr_whltr_rl = sptr_whltr_rl;
	m_sptr_whltr_rr = sptr_whltr_rr;
}

void NMSPC::Subsystem_4WheelTires_DiskFiala::preprocess(const d_vec &u, \
const double &t){
	//get system time
	m_t = t;
	//
	d_vec u_fl = d_vec(u.begin(),u.begin() + m_inputs_num / m_whltr_num);
	d_vec u_fr = d_vec(u.begin() + m_inputs_num / m_whltr_num, \
	u.begin() + 2 * m_inputs_num / m_whltr_num);
	d_vec u_rl = d_vec(u.begin() + 2 * m_inputs_num / m_whltr_num, \
	u.begin() + 3 * m_inputs_num / m_whltr_num);
	d_vec u_rr = d_vec(u.begin() + 3 * m_inputs_num / m_whltr_num, \
	u.end());
	m_sptr_whltr_fl->preupdate(u_fl,t);
	m_sptr_whltr_fr->preupdate(u_fr,t);
	m_sptr_whltr_rl->preupdate(u_rl,t);
	m_sptr_whltr_rr->preupdate(u_rr,t);
}

void NMSPC::Subsystem_4WheelTires_DiskFiala::collect_con_states() {
	//front left
	std::copy(m_sptr_whltr_fl->m_con_states.begin(), \
	m_sptr_whltr_fl->m_con_states.end(), \
	m_con_states.begin());
	//front right
	std::copy(m_sptr_whltr_fr->m_con_states.begin(), \
	m_sptr_whltr_fr->m_con_states.end(), \
	m_con_states.begin() + m_con_states_num / m_whltr_num);
	//rear left
	std::copy(m_sptr_whltr_rl->m_con_states.begin(), \
	m_sptr_whltr_rl->m_con_states.end(), \
	m_con_states.begin() + 2 * m_con_states_num / m_whltr_num);
	//rear right
	std::copy(m_sptr_whltr_rr->m_con_states.begin(), \
	m_sptr_whltr_rr->m_con_states.end(), \
	m_con_states.begin() + 3 * m_con_states_num / m_whltr_num);
}

void NMSPC::Subsystem_4WheelTires_DiskFiala::calculate_and_collect_derivatives(){
	m_sptr_whltr_fl->calculate_derivatives(m_drvs[0], \
	m_drvs[1], m_drvs[2], m_drvs[3]);
	m_sptr_whltr_fr->calculate_derivatives(m_drvs[4], \
	m_drvs[5], m_drvs[6], m_drvs[7]);
	m_sptr_whltr_rl->calculate_derivatives(m_drvs[8], \
	m_drvs[9], m_drvs[10], m_drvs[11]);
	m_sptr_whltr_rr->calculate_derivatives(m_drvs[12], \
	m_drvs[13], m_drvs[14], m_drvs[15]);
}

void NMSPC::Subsystem_4WheelTires_DiskFiala::distribute_con_states() {
	//front left
	std::copy(m_con_states.begin(), \
	m_con_states.begin() + m_con_states_num / m_whltr_num, \
	m_sptr_whltr_fl->m_con_states.begin());
	//front right
	std::copy(m_con_states.begin() + m_con_states_num / m_whltr_num, \
	m_con_states.begin() + 2 * m_con_states_num / m_whltr_num, \
	m_sptr_whltr_fr->m_con_states.begin());
	//rear left
	std::copy(m_con_states.begin() + 2 * m_con_states_num / m_whltr_num, \
	m_con_states.begin() + 3 * m_con_states_num / m_whltr_num, \
	m_sptr_whltr_rl->m_con_states.begin());
	//rear right
	std::copy(m_con_states.begin() + 3 * m_con_states_num / m_whltr_num, \
	m_con_states.end(), \
	m_sptr_whltr_rr->m_con_states.begin());
}

void NMSPC::Subsystem_4WheelTires_DiskFiala::postprocess() {
	m_sptr_whltr_fl->postupdate();
	m_sptr_whltr_fr->postupdate();
	m_sptr_whltr_rl->postupdate();
	m_sptr_whltr_rr->postupdate();
}

void NMSPC::Subsystem_4WheelTires_DiskFiala::output(d_vec &outputs) {
	d_vec output_fl = d_vec(m_outputs_num / m_whltr_num, NaN);
	d_vec output_fr = d_vec(m_outputs_num / m_whltr_num, NaN);
	d_vec output_rl = d_vec(m_outputs_num / m_whltr_num, NaN);
	d_vec output_rr = d_vec(m_outputs_num / m_whltr_num, NaN);
	m_sptr_whltr_fl->output(output_fl);
	m_sptr_whltr_fr->output(output_fr);
	m_sptr_whltr_rl->output(output_rl);
	m_sptr_whltr_rr->output(output_rr);
	std::copy(output_fl.begin(), output_fl.end(), outputs.begin());
	std::copy(output_fr.begin(), output_fr.end(), outputs.begin() + \
	m_outputs_num / m_whltr_num);
	std::copy(output_rl.begin(), output_rl.end(), outputs.begin() + \
	2 * m_outputs_num / m_whltr_num);
	std::copy(output_rr.begin(), output_rr.end(), outputs.begin() + \
	3 * m_outputs_num / m_whltr_num);
}
