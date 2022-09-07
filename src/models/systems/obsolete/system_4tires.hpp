#ifndef SYSTEM_4TIRES_HPP
#define SYSTEM_4TIRES_HPP
#include "components/common.hpp"
#include "components/tire_fiala.hpp"

class System_4Tires {
public:
	System_4Tires(const double &t) : m_t(t) {};
	void add_tires(std::shared_ptr<Tire_Fiala> sptr_tr_fl, \
					std::shared_ptr<Tire_Fiala> sptr_tr_fr, \
					std::shared_ptr<Tire_Fiala> sptr_tr_rl, \
					std::shared_ptr<Tire_Fiala> sptr_tr_rr);
	void add_tires_inputs(std::shared_ptr<d_vec> sptr_inputs_tr_fl, \
						   std::shared_ptr<d_vec> sptr_inputs_tr_fr, \
						   std::shared_ptr<d_vec> sptr_inputs_tr_rl, \
						   std::shared_ptr<d_vec> sptr_inputs_tr_rr);
	
	void preprocess(const double &t);
	void collect_con_states();
	void calculate_and_collect_derivatives();
    void distribute_con_states();
    void postprocess();
    
    void operator() (const d_vec &x, d_vec &dxdt, const double &t);
    void derivative (const d_vec &x, d_vec &dxdt, const double &t);

private:
	double m_t;	//system current time
	
	const int m_tire_num = 4;
	const int m_total_con_states_num = m_tire_num * \
	 Tire_Fiala::m_con_states_num;
	const int m_total_drvs_num = m_total_con_states_num;

	std::shared_ptr<d_vec> m_sptr_inputs_tr_fl, m_sptr_inputs_tr_fr, \
	m_sptr_inputs_tr_rl, m_sptr_inputs_tr_rr;
	std::shared_ptr<Tire_Fiala> m_sptr_tr_fl, m_sptr_tr_fr, m_sptr_tr_rl, \
	m_sptr_tr_rr;
	
public:
	d_vec m_system_drvs = d_vec(m_total_drvs_num, NaN);
	d_vec m_system_con_states = d_vec(m_total_con_states_num, NaN);
	
};

#endif //SYSTEM_4TIRES_HPP
