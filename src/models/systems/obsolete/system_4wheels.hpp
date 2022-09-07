#ifndef SYSTEM_4WHEELS_HPP
#define SYSTEM_4WHEELS_HPP
#include "components/common.hpp"
#include "components/wheel.hpp"

class System_4Wheels {
public:
	System_4Wheels(const double &t) : m_t(t) {};
	void add_wheels(std::shared_ptr<Wheel> sptr_whl_fl, \
					std::shared_ptr<Wheel> sptr_whl_fr, \
					std::shared_ptr<Wheel> sptr_whl_rl, \
					std::shared_ptr<Wheel> sptr_whl_rr);
	void add_wheels_inputs(std::shared_ptr<d_vec> sptr_inputs_whl_fl, \
						   std::shared_ptr<d_vec> sptr_inputs_whl_fr, \
						   std::shared_ptr<d_vec> sptr_inputs_whl_rl, \
						   std::shared_ptr<d_vec> sptr_inputs_whl_rr);
	
	void preprocess(const double &t);
	void collect_con_states();
	void calculate_and_collect_derivatives();
    void distribute_con_states();
    void postprocess();
    
    void operator() (const d_vec &x, d_vec &dxdt, const double &t);
    void derivative (const d_vec &x, d_vec &dxdt, const double &t);

private:
	double m_t;	//system current time
	
	const int m_whl_nums = 4;
	const int m_total_con_states_num = m_whl_nums * \
	Wheel::m_con_states_num;
	const int m_total_drvs_num = m_total_con_states_num;

	std::shared_ptr<d_vec> m_sptr_inputs_whl_fl, m_sptr_inputs_whl_fr, m_sptr_inputs_whl_rl, m_sptr_inputs_whl_rr;
	std::shared_ptr<Wheel> m_sptr_whl_fl, m_sptr_whl_fr, m_sptr_whl_rl, m_sptr_whl_rr;
	
public:
	d_vec m_system_drvs = d_vec(m_total_drvs_num, NaN);
	d_vec m_system_con_states = d_vec(m_total_con_states_num, NaN);
	
};

#endif //SYSTEM_4WHEELS_HPP
