#ifndef SYSTEM_WHEELS_TIRES_HPP
#define SYSTEM_WHEELS_TIRES_HPP
#include "components/common.hpp"
#include "components/wheel_disk_tire_fiala.hpp"

class System_Wheels_Tires {
public:
	System_Wheels_Tires(const double &t) : m_t(t) {};
	void add_wheel_tire(\
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whl_tr_fl, \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whl_tr_fr, \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whl_tr_rl, \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whl_tr_rr);
	void add_wheel_tire_inputs(\
	std::shared_ptr<d_vec> sptr_inputs_whl_tr_fl, \
	std::shared_ptr<d_vec> sptr_inputs_whl_tr_fr, \
	std::shared_ptr<d_vec> sptr_inputs_whl_tr_rl, \
	std::shared_ptr<d_vec> sptr_inputs_whl_tr_rr);
	
	void preprocess(const double &t);
	void collect_con_states();
	void calculate_and_collect_derivatives();
    void distribute_con_states();
    void postprocess();
    
    void operator() (const d_vec &x, d_vec &dxdt, const double &t);
    void derivative (const d_vec &x, d_vec &dxdt, const double &t);

private:
	double m_t;	//system current time
	
	const int m_whl_tr_nums = 4;
	const int m_total_con_states_num = m_whl_tr_nums * \
	Wheel_Disk_Tire_Fiala::m_con_states_num;
	const int m_total_drvs_num = m_total_con_states_num;

	std::shared_ptr<d_vec> m_sptr_inputs_whl_tr_fl, \
	m_sptr_inputs_whl_tr_fr, \
	m_sptr_inputs_whl_tr_rl, \
	m_sptr_inputs_whl_tr_rr;
	std::shared_ptr<Wheel_Disk_Tire_Fiala> m_sptr_whl_tr_fl, \
	m_sptr_whl_tr_fr, \
	m_sptr_whl_tr_rl, \
	m_sptr_whl_tr_rr;
	
public:
	d_vec m_system_drvs = d_vec(m_total_drvs_num, NaN);
	d_vec m_system_con_states = d_vec(m_total_con_states_num, NaN);
	
};

#endif //SYSTEM_4WHEELS_DISK_TIRE_FIALA_HPP
