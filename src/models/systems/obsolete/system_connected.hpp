#ifndef SYSTEM_CONNECTED_HPP
#define SYSTEM_CONNECTED_HPP
#include "components/common.hpp"
#include "components/wheel.hpp"
#include "components/tire_fiala.hpp"

class System_Connected {
public:
	System_Connected(const double &t) : m_t(t) {};
	void add_wheels(std::shared_ptr<Wheel> sptr_whl_fl, \
	std::shared_ptr<Wheel> sptr_whl_fr, \
	std::shared_ptr<Wheel> sptr_whl_rl, \
	std::shared_ptr<Wheel> sptr_whl_rr);
	void add_tires(std::shared_ptr<Tire_Fiala> sptr_tr_fl, \
	std::shared_ptr<Tire_Fiala> sptr_tr_fr, \
	std::shared_ptr<Tire_Fiala> sptr_tr_rl, \
	std::shared_ptr<Tire_Fiala> sptr_tr_rr);
					
	void add_connected_inputs(\
	std::shared_ptr<d_vec> sptr_inputs_connected_fl, \
	std::shared_ptr<d_vec> sptr_inputs_connected_fr, \
	std::shared_ptr<d_vec> sptr_inputs_connected_rl, \
	std::shared_ptr<d_vec> sptr_inputs_connected_rr);
	
	void preprocess(const double &t);
	void collect_con_states();
	void calculate_and_collect_derivatives();
    void distribute_con_states();
    void postprocess();
    
    void operator() (const d_vec &x, d_vec &dxdt, const double &t);
    void derivative (const d_vec &x, d_vec &dxdt, const double &t);

private:
	double m_t;	//system current time
	
	const int m_whl_num = 4;
	const int m_tire_num = 4;
	const int m_total_con_states_num = \
	m_whl_num * Wheel::m_con_states_num + \
	m_tire_num * Tire_Fiala::m_con_states_num;
	const int m_total_drvs_num = m_total_con_states_num;
	
	//kinematics outputs
	d_vec m_kine_output_whls_fl = d_vec(Wheel::m_outputs_num, NaN);
	d_vec m_kine_output_whls_fr = d_vec(Wheel::m_outputs_num, NaN);
	d_vec m_kine_output_whls_rl = d_vec(Wheel::m_outputs_num, NaN);
	d_vec m_kine_output_whls_rr = d_vec(Wheel::m_outputs_num, NaN);
	//dynamics outputs
	d_vec m_dyna_output_tr_fl = d_vec(Tire_Fiala::m_outputs_num, NaN);
	d_vec m_dyna_output_tr_fr = d_vec(Tire_Fiala::m_outputs_num, NaN);
	d_vec m_dyna_output_tr_rl = d_vec(Tire_Fiala::m_outputs_num, NaN);
	d_vec m_dyna_output_tr_rr = d_vec(Tire_Fiala::m_outputs_num, NaN);

	//external inputs of the conneced system:
	//Brake_Pressure, Axl_trq, vx, vy, susF, gamma, psidot, tirePrs, Gnd, scale
	std::shared_ptr<d_vec> m_sptr_inputs_connected_fl, \
	m_sptr_inputs_connected_fr, \
	m_sptr_inputs_connected_rl, \
	m_sptr_inputs_connected_rr;
	//wheel inputs
	d_vec m_inputs_whl_fl = d_vec(Wheel::m_inputs_num, NaN);
	d_vec m_inputs_whl_fr = d_vec(Wheel::m_inputs_num, NaN);
	d_vec m_inputs_whl_rl = d_vec(Wheel::m_inputs_num, NaN);
	d_vec m_inputs_whl_rr = d_vec(Wheel::m_inputs_num, NaN);
	//tire inputs
	d_vec m_inputs_tr_fl = d_vec(Tire_Fiala::m_inputs_num, NaN);
	d_vec m_inputs_tr_fr = d_vec(Tire_Fiala::m_inputs_num, NaN);
	d_vec m_inputs_tr_rl = d_vec(Tire_Fiala::m_inputs_num, NaN);
	d_vec m_inputs_tr_rr = d_vec(Tire_Fiala::m_inputs_num, NaN);
	
	std::shared_ptr<Wheel> m_sptr_whl_fl, m_sptr_whl_fr, m_sptr_whl_rl, \
	m_sptr_whl_rr;
	std::shared_ptr<Tire_Fiala> m_sptr_tr_fl, m_sptr_tr_fr, m_sptr_tr_rl, \
	m_sptr_tr_rr;
	
public:
	d_vec m_system_drvs = d_vec(m_total_drvs_num, NaN);
	d_vec m_system_con_states = d_vec(m_total_con_states_num, NaN);
	
};

#endif //SYSTEM_CONNECTED_HPP
