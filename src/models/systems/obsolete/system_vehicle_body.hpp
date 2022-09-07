#ifndef SYSTEM_VEHICLE_BODY_HPP
#define SYSTEM_VEHICLE_BODY_HPP
#include "components/common.hpp"
#include "components/vehicle_body.hpp"

namespace NMSPC{
class System_Vehicle_Body {
public:
	System_Vehicle_Body(const double &t) : m_t(t) {};
	void add_vehicle_body(\
	std::shared_ptr<Vehicle_Body> sptr_vhlbdy) {
		m_sptr_vhlbdy = sptr_vhlbdy;
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
	
	const int m_vhlbdy_nums = 1;
	const int m_total_inputs_num = m_vhlbdy_nums * \
	Vehicle_Body::m_inputs_num;
	const int m_total_con_states_num = m_vhlbdy_nums * \
	Vehicle_Body::m_con_states_num;
	const int m_total_drvs_num = m_total_con_states_num;
	const int m_total_outputs_num = m_vhlbdy_nums * \
	Vehicle_Body::m_outputs_num;

	std::shared_ptr<Vehicle_Body> m_sptr_vhlbdy;
	d_vec m_vhlbdy_inputs = d_vec(Vehicle_Body::m_inputs_num, 0.0);
public:
	d_vec m_system_outputs = d_vec(m_total_outputs_num, NaN);
	Eigen::Matrix3d m_output_DCM{{NaN, NaN, NaN},\
	{NaN, NaN, NaN},{NaN, NaN, NaN}};
	d_vec m_system_drvs = d_vec(m_total_drvs_num, NaN);
	d_vec m_system_con_states = d_vec(m_total_con_states_num, NaN);
	
};

}	//end of name space
#endif //SYSTEM_VEHICLE_BODY_HPP
