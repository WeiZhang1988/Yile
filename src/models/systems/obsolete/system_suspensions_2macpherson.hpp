#ifndef SYSTEM_SUSPENSIONS_2MACPHERSON_HPP
#define SYSTEM_SUSPENSIONS_2MACPHERSON_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_suspensions_2macpherson.hpp"

namespace NMSPC{
class System_Suspensions_2Macpherson {
public:
	System_Suspensions_2Macpherson(const double &t) : m_t(t) {};
	void add_subsystems(\
	std::shared_ptr<Subsystem_Suspensions_2Macpherson> sptr_sb2mpsn) {
		m_sptr_sb2mpsn = sptr_sb2mpsn;
	};
	
	void feed_inputs(const d_vec &u);
	void preprocess(const double &t);
	void collect_con_states();
	void calculate_and_collect_derivatives();
    void distribute_con_states();
    void postprocess();
    void output();
    
    void operator() (const d_vec &x, d_vec &dxdt, const double &t);
    void derivative (const d_vec &x, d_vec &dxdt, const double &t);

private:
	double m_t;	//system current time
	
	const int m_sub_sus_nums = 1;
	const int m_total_inputs_num = m_sub_sus_nums * \
	Subsystem_Suspensions_2Macpherson::m_inputs_num;
	const int m_total_con_states_num = m_sub_sus_nums * \
	Subsystem_Suspensions_2Macpherson::m_con_states_num;
	const int m_total_drvs_num = m_total_con_states_num;
	const int m_total_outputs_num = m_sub_sus_nums * \
	Subsystem_Suspensions_2Macpherson::m_outputs_num;

	std::shared_ptr<Subsystem_Suspensions_2Macpherson> m_sptr_sb2mpsn;
	d_vec m_sb2mpsn_inputs = d_vec(Subsystem_Suspensions_2Macpherson::m_inputs_num, 0.0);
public:
	d_vec m_system_outputs = d_vec(m_total_outputs_num, 0.0);
	d_vec m_system_drvs = d_vec(m_total_drvs_num, NaN);
	d_vec m_system_con_states = d_vec(m_total_con_states_num, NaN);
	
};

}	//end of name space
#endif //SYSTEM_4WHEELTIRES_DISKFIALA_HPP
