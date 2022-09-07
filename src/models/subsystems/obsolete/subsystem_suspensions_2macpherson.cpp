#include "subsystem_suspensions_2macpherson.hpp"

void NMSPC::Subsystem_Suspensions_2Macpherson::add_suspensions( \
	std::shared_ptr<Sus_Macpherson_2Tracks> sptr_sus_f, \
	std::shared_ptr<Sus_Macpherson_2Tracks> sptr_sus_r) {
	m_sptr_sus_f = sptr_sus_f;
	m_sptr_sus_r = sptr_sus_r;
}

void NMSPC::Subsystem_Suspensions_2Macpherson::preprocess(const d_vec &u, const double &t){
	//get system time
	m_t = t;
	//
	d_vec u_f = d_vec(u.begin(),u.begin() + m_inputs_num / m_sus_num);
	d_vec u_r = d_vec(u.begin() + m_inputs_num / m_sus_num, u.end());
	m_sptr_sus_f->preupdate(u_f,t);
	m_sptr_sus_r->preupdate(u_r,t);
}

void NMSPC::Subsystem_Suspensions_2Macpherson::output(d_vec &outputs) {
	d_vec output_f = d_vec(m_outputs_num / m_sus_num, NaN);
	d_vec output_r = d_vec(m_outputs_num / m_sus_num, NaN);
	m_sptr_sus_f->output(output_f);
	m_sptr_sus_r->output(output_r);
	std::copy(output_f.begin(), output_f.end(), outputs.begin());
	std::copy(output_r.begin(), output_r.end(), outputs.begin() + \
	m_outputs_num / m_sus_num);
}
