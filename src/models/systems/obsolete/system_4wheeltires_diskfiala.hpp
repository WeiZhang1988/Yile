#ifndef SYSTEM_4WHEELTIRES_DISKFIALA_HPP
#define SYSTEM_4WHEELTIRES_DISKFIALA_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_4wheeltires_diskfiala.hpp"

namespace NMSPC{
class System_4WheelTires_DiskFiala {
public:
	System_4WheelTires_DiskFiala(const double &t) : m_t(t) {
		m_sb4whltr_inputs = {0.0, 10.0, 0.0, 0.0, 0, 0.0, 0.0, 2e4, 0.0, 1.0, \
		0.0, 50.0, 0.0, 0.0, 0, 0.0, 0.0, 2e4, 0.0, 1.0, \
		0.0, 100.0, 0.0, 0.0, 0, 0.0, 0.0, 2e4, 0.0, 1.0, \
		0.0, 200.0, 0.0, 0.0, 0, 0.0, 0.0, 2e4, 0.0, 1.0};
	};
	void add_subsystems(\
	std::shared_ptr<Subsystem_4WheelTires_DiskFiala> sptr_sb4whltr) {
		m_sptr_sb4whltr = sptr_sb4whltr;
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
	
	const int m_sub_whltr_nums = 1;
	const int m_total_inputs_num = m_sub_whltr_nums * \
	Subsystem_4WheelTires_DiskFiala::m_inputs_num;
	const int m_total_con_states_num = m_sub_whltr_nums * \
	Subsystem_4WheelTires_DiskFiala::m_con_states_num;
	const int m_total_drvs_num = m_total_con_states_num;
	const int m_total_outputs_num = m_sub_whltr_nums * \
	Subsystem_4WheelTires_DiskFiala::m_outputs_num;

	std::shared_ptr<Subsystem_4WheelTires_DiskFiala> m_sptr_sb4whltr;
	d_vec m_sb4whltr_inputs = d_vec(Subsystem_4WheelTires_DiskFiala::m_inputs_num, 0.0);
public:
	d_vec m_system_outputs = d_vec(m_total_outputs_num, NaN);
	d_vec m_system_drvs = d_vec(m_total_drvs_num, NaN);
	d_vec m_system_con_states = d_vec(m_total_con_states_num, NaN);
	
};

}	//end of name space
#endif //SYSTEM_4WHEELTIRES_DISKFIALA_HPP
