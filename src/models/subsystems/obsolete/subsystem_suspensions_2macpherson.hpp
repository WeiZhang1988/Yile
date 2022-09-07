#ifndef SUBSYSTEM_SUSPENSIONS_2MACPHERSON_HPP
#define SUBSYSTEM_SUSPENSIONS_2MACPHERSON_HPP
#include "components/common.hpp"
#include "components/sus_macpherson_2tracks.hpp"

namespace NMSPC{
class Subsystem_Suspensions_2Macpherson {
public:
	static const int m_sus_num = 2;
	static const int m_params_num = m_sus_num * \
	Sus_Macpherson_2Tracks::m_params_num;
	static const int m_inputs_num = m_sus_num * \
	Sus_Macpherson_2Tracks::m_inputs_num;
	static const int m_con_states_num = m_sus_num * \
	Sus_Macpherson_2Tracks::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = m_sus_num * \
	Sus_Macpherson_2Tracks::m_dis_states_num;
	static const int m_outputs_num = m_sus_num * \
	Sus_Macpherson_2Tracks::m_outputs_num;
	
	Subsystem_Suspensions_2Macpherson(const double &t) : m_t(t) {};
	void add_suspensions( \
	std::shared_ptr<Sus_Macpherson_2Tracks> sptr_sus_f, \
	std::shared_ptr<Sus_Macpherson_2Tracks> sptr_sus_r);
	
	void preprocess (const d_vec &u, const double &t);
	void collect_con_states() {};
	void calculate_and_collect_derivatives() {};
	void distribute_con_states() {};
	void postprocess() {};
	void output (d_vec &outputs);

private:
	std::shared_ptr<Sus_Macpherson_2Tracks> m_sptr_sus_f, m_sptr_sus_r;

public:
	double m_t;	//subsystem time
	//for states communication with class System
	d_vec m_drvs = d_vec(m_derivatives_num, NaN);
	d_vec m_con_states = d_vec(m_con_states_num, NaN);

};

}	//end of name space
#endif //SUBSYSTEM_SUSPENSIONS_2MACPHERSON_HPP
