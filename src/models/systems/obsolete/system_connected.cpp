#include "system_connected.hpp"

void System_Connected::add_wheels(\
std::shared_ptr<Wheel> sptr_whl_fl, \
std::shared_ptr<Wheel> sptr_whl_fr, \
std::shared_ptr<Wheel> sptr_whl_rl, \
std::shared_ptr<Wheel> sptr_whl_rr){
	m_sptr_whl_fl = sptr_whl_fl;
	m_sptr_whl_fr = sptr_whl_fr;
	m_sptr_whl_rl = sptr_whl_rl;
	m_sptr_whl_rr = sptr_whl_rr;
}

void System_Connected::add_tires(\
std::shared_ptr<Tire_Fiala> sptr_tr_fl, \
std::shared_ptr<Tire_Fiala> sptr_tr_fr, \
std::shared_ptr<Tire_Fiala> sptr_tr_rl, \
std::shared_ptr<Tire_Fiala> sptr_tr_rr) {
	m_sptr_tr_fl = sptr_tr_fl;
	m_sptr_tr_fr = sptr_tr_fr;
	m_sptr_tr_rl = sptr_tr_rl;
	m_sptr_tr_rr = sptr_tr_rr;
}
	
void System_Connected::add_connected_inputs(\
std::shared_ptr<d_vec> sptr_inputs_connected_fl, \
std::shared_ptr<d_vec> sptr_inputs_connected_fr, \
std::shared_ptr<d_vec> sptr_inputs_connected_rl, \
std::shared_ptr<d_vec> sptr_inputs_connected_rr) {
	m_sptr_inputs_connected_fl = sptr_inputs_connected_fl;
	m_sptr_inputs_connected_fr = sptr_inputs_connected_fr;
	m_sptr_inputs_connected_rl = sptr_inputs_connected_rl;
	m_sptr_inputs_connected_rr = sptr_inputs_connected_rr;
}

void System_Connected::preprocess(const double &t) {
	//get time
	m_t = t;
	//get kinematics outputs
	m_sptr_whl_fl->output(m_kine_output_whls_fl);
	m_sptr_whl_fr->output(m_kine_output_whls_fr);
	m_sptr_whl_rl->output(m_kine_output_whls_rl);
	m_sptr_whl_rr->output(m_kine_output_whls_rr);
	//organize intputs for dynamics components
	m_inputs_tr_fl[0] = m_kine_output_whls_fl[0];
	m_inputs_tr_fl[1] = (*m_sptr_inputs_connected_fl)[2];
	m_inputs_tr_fl[2] = (*m_sptr_inputs_connected_fl)[3];
	m_inputs_tr_fl[3] = (*m_sptr_inputs_connected_fl)[4];
	m_inputs_tr_fl[4] = (*m_sptr_inputs_connected_fl)[5];
	m_inputs_tr_fl[5] = (*m_sptr_inputs_connected_fl)[6];
	m_inputs_tr_fl[6] = (*m_sptr_inputs_connected_fl)[7];
	m_inputs_tr_fl[7] = (*m_sptr_inputs_connected_fl)[8];
	m_inputs_tr_fl[8] = (*m_sptr_inputs_connected_fl)[9];
	//---
	m_inputs_tr_fr[0] = m_kine_output_whls_fr[0];
	m_inputs_tr_fr[1] = (*m_sptr_inputs_connected_fr)[2];
	m_inputs_tr_fr[2] = (*m_sptr_inputs_connected_fr)[3];
	m_inputs_tr_fr[3] = (*m_sptr_inputs_connected_fr)[4];
	m_inputs_tr_fr[4] = (*m_sptr_inputs_connected_fr)[5];
	m_inputs_tr_fr[5] = (*m_sptr_inputs_connected_fr)[6];
	m_inputs_tr_fr[6] = (*m_sptr_inputs_connected_fr)[7];
	m_inputs_tr_fr[7] = (*m_sptr_inputs_connected_fr)[8];
	m_inputs_tr_fr[8] = (*m_sptr_inputs_connected_fr)[9];
	//---
	m_inputs_tr_rl[0] = m_kine_output_whls_rl[0];
	m_inputs_tr_rl[1] = (*m_sptr_inputs_connected_rl)[2];
	m_inputs_tr_rl[2] = (*m_sptr_inputs_connected_rl)[3];
	m_inputs_tr_rl[3] = (*m_sptr_inputs_connected_rl)[4];
	m_inputs_tr_rl[4] = (*m_sptr_inputs_connected_rl)[5];
	m_inputs_tr_rl[5] = (*m_sptr_inputs_connected_rl)[6];
	m_inputs_tr_rl[6] = (*m_sptr_inputs_connected_rl)[7];
	m_inputs_tr_rl[7] = (*m_sptr_inputs_connected_rl)[8];
	m_inputs_tr_rl[8] = (*m_sptr_inputs_connected_rl)[9];
	//---
	m_inputs_tr_rr[0] = m_kine_output_whls_rr[0];
	m_inputs_tr_rr[1] = (*m_sptr_inputs_connected_rr)[2];
	m_inputs_tr_rr[2] = (*m_sptr_inputs_connected_rr)[3];
	m_inputs_tr_rr[3] = (*m_sptr_inputs_connected_rr)[4];
	m_inputs_tr_rr[4] = (*m_sptr_inputs_connected_rr)[5];
	m_inputs_tr_rr[5] = (*m_sptr_inputs_connected_rr)[6];
	m_inputs_tr_rr[6] = (*m_sptr_inputs_connected_rr)[7];
	m_inputs_tr_rr[7] = (*m_sptr_inputs_connected_rr)[8];
	m_inputs_tr_rr[8] = (*m_sptr_inputs_connected_rr)[9];
	//do preupdate for dynamics components
	m_sptr_tr_fl->preupdate(m_inputs_tr_fl, m_t);
	m_sptr_tr_fr->preupdate(m_inputs_tr_fr, m_t);
	m_sptr_tr_rl->preupdate(m_inputs_tr_rl, m_t);
	m_sptr_tr_rr->preupdate(m_inputs_tr_rr, m_t);
	//get dynamics outputs
	m_sptr_tr_fl->output(m_dyna_output_tr_fl);
	m_sptr_tr_fr->output(m_dyna_output_tr_fr);
	m_sptr_tr_rl->output(m_dyna_output_tr_rl);
	m_sptr_tr_rr->output(m_dyna_output_tr_rr);
	//organize intputs for kinematics components
	m_inputs_whl_fl[0] = (*m_sptr_inputs_connected_fl)[0];
	m_inputs_whl_fl[1] = (*m_sptr_inputs_connected_fl)[1];
	m_inputs_whl_fl[2] = m_dyna_output_tr_fl[4];	//MRoll
	m_inputs_whl_fl[3] = m_dyna_output_tr_fl[0];	//Fx
	m_inputs_whl_fl[4] = m_dyna_output_tr_fl[6];	//rho
	//---
	m_inputs_whl_fr[0] = (*m_sptr_inputs_connected_fr)[0];
	m_inputs_whl_fr[1] = (*m_sptr_inputs_connected_fr)[1];
	m_inputs_whl_fr[2] = m_dyna_output_tr_fr[4];	//MRoll
	m_inputs_whl_fr[3] = m_dyna_output_tr_fr[0];	//Fx
	m_inputs_whl_fr[4] = m_dyna_output_tr_fr[6];	//rho
	//---
	m_inputs_whl_rl[0] = (*m_sptr_inputs_connected_rl)[0];
	m_inputs_whl_rl[1] = (*m_sptr_inputs_connected_rl)[1];
	m_inputs_whl_rl[2] = m_dyna_output_tr_rl[4];	//MRoll
	m_inputs_whl_rl[3] = m_dyna_output_tr_rl[0];	//Fx
	m_inputs_whl_rl[4] = m_dyna_output_tr_rl[6];	//rho
	//---
	m_inputs_whl_rr[0] = (*m_sptr_inputs_connected_rr)[0];
	m_inputs_whl_rr[1] = (*m_sptr_inputs_connected_rr)[1];
	m_inputs_whl_rr[2] = m_dyna_output_tr_rr[4];	//MRoll
	m_inputs_whl_rr[3] = m_dyna_output_tr_rr[0];	//Fx
	m_inputs_whl_rr[4] = m_dyna_output_tr_rr[6];	//rho
	//do preupdate for dynamics components
	m_sptr_whl_fl->preupdate(m_inputs_whl_fl, m_t);
	m_sptr_whl_fr->preupdate(m_inputs_whl_fr, m_t);
	m_sptr_whl_rl->preupdate(m_inputs_whl_rl, m_t);
	m_sptr_whl_rr->preupdate(m_inputs_whl_rr, m_t);
}

void System_Connected::collect_con_states() {
	m_system_con_states[0] = m_sptr_whl_fl->m_con_states[0];
	m_system_con_states[1] = m_sptr_whl_fr->m_con_states[0];
	m_system_con_states[2] = m_sptr_whl_rl->m_con_states[0];
	m_system_con_states[3] = m_sptr_whl_rr->m_con_states[0];
	
	m_system_con_states[4] = m_sptr_tr_fl->m_con_states[0];
	m_system_con_states[5] = m_sptr_tr_fl->m_con_states[1];
	m_system_con_states[6] = m_sptr_tr_fl->m_con_states[2];
	m_system_con_states[7] = m_sptr_tr_fr->m_con_states[0];
	m_system_con_states[8] = m_sptr_tr_fr->m_con_states[1];
	m_system_con_states[9] = m_sptr_tr_fr->m_con_states[2];
	m_system_con_states[10] = m_sptr_tr_rl->m_con_states[0];
	m_system_con_states[11] = m_sptr_tr_rl->m_con_states[1];
	m_system_con_states[12] = m_sptr_tr_rl->m_con_states[2];
	m_system_con_states[13] = m_sptr_tr_rr->m_con_states[0];
	m_system_con_states[14] = m_sptr_tr_rr->m_con_states[1];
	m_system_con_states[15] = m_sptr_tr_rr->m_con_states[2];
}

void System_Connected::calculate_and_collect_derivatives() {
	m_sptr_whl_fl->calculate_derivatives(m_system_drvs[0]);
	m_sptr_whl_fr->calculate_derivatives(m_system_drvs[1]);
	m_sptr_whl_rl->calculate_derivatives(m_system_drvs[2]);
	m_sptr_whl_rr->calculate_derivatives(m_system_drvs[3]);
	
	m_sptr_tr_fl->calculate_derivatives(m_system_drvs[4],
										m_system_drvs[5],
										m_system_drvs[6]);
	m_sptr_tr_fr->calculate_derivatives(m_system_drvs[7],
										m_system_drvs[8],
										m_system_drvs[9]);
	m_sptr_tr_rl->calculate_derivatives(m_system_drvs[10],
										m_system_drvs[11],
										m_system_drvs[12]);
	m_sptr_tr_rr->calculate_derivatives(m_system_drvs[13],
										m_system_drvs[14],
										m_system_drvs[15]);
}

void System_Connected::distribute_con_states() {
	m_sptr_whl_fl->m_con_states[0] = m_system_con_states[0];
	m_sptr_whl_fr->m_con_states[0] = m_system_con_states[1];
	m_sptr_whl_rl->m_con_states[0] = m_system_con_states[2];
	m_sptr_whl_rr->m_con_states[0] = m_system_con_states[3];
	
	m_sptr_tr_fl->m_con_states[0] = m_system_con_states[4];
	m_sptr_tr_fl->m_con_states[1] = m_system_con_states[5];
	m_sptr_tr_fl->m_con_states[2] = m_system_con_states[6];
	m_sptr_tr_fr->m_con_states[0] = m_system_con_states[7];
	m_sptr_tr_fr->m_con_states[1] = m_system_con_states[8];
	m_sptr_tr_fr->m_con_states[2] = m_system_con_states[9];
	m_sptr_tr_rl->m_con_states[0] = m_system_con_states[10];
	m_sptr_tr_rl->m_con_states[1] = m_system_con_states[11];
	m_sptr_tr_rl->m_con_states[2] = m_system_con_states[12];
	m_sptr_tr_rr->m_con_states[0] = m_system_con_states[13];
	m_sptr_tr_rr->m_con_states[1] = m_system_con_states[14];
	m_sptr_tr_rr->m_con_states[2] = m_system_con_states[15];
}

void System_Connected::postprocess() {
	m_sptr_tr_fl->postupdate();
	m_sptr_tr_fr->postupdate();
	m_sptr_tr_rl->postupdate();
	m_sptr_tr_rr->postupdate();
	m_sptr_whl_fl->postupdate();
	m_sptr_whl_fr->postupdate();
	m_sptr_whl_rl->postupdate();
	m_sptr_whl_rr->postupdate();
}

void System_Connected::operator() (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
void System_Connected::derivative (const d_vec &x, d_vec &dxdt, const double &t) {
	std::copy(m_system_drvs.begin(),m_system_drvs.end(),dxdt.begin());
}
