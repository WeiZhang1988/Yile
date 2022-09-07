#ifndef SUBSYSTEM_4WHEELTIRES_DISKFIALA_HPP
#define SUBSYSTEM_4WHEELTIRES_DISKFIALA_HPP
#include "components/common.hpp"
#include "components/wheel_disk_tire_fiala.hpp"

namespace NMSPC{
class Subsystem_4WheelTires_DiskFiala {
public:
	static const int m_whltr_num = 4;
	static const int m_params_num = m_whltr_num * \
	Wheel_Disk_Tire_Fiala::m_params_num;
	static const int m_k_inputs_num = m_whltr_num * \
	Wheel_Disk_Tire_Fiala::m_k_inputs_num;
	static const int m_d_inputs_num = m_whltr_num * \
	Wheel_Disk_Tire_Fiala::m_d_inputs_num;
	static const int m_inputs_num = m_whltr_num * \
	Wheel_Disk_Tire_Fiala::m_inputs_num;
	static const int m_con_states_num = m_whltr_num * \
	Wheel_Disk_Tire_Fiala::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = m_whltr_num * \
	Wheel_Disk_Tire_Fiala::m_dis_states_num;
	static const int m_k_outputs_num = m_whltr_num * \
	Wheel_Disk_Tire_Fiala::m_k_outputs_num;
	static const int m_d_outputs_num = m_whltr_num * \
	Wheel_Disk_Tire_Fiala::m_d_outputs_num;
	static const int m_outputs_num = m_whltr_num * \
	Wheel_Disk_Tire_Fiala::m_outputs_num;
	
	Subsystem_4WheelTires_DiskFiala(const double &t) : m_t(t) {};
	void add_wheeltires( \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whltr_fl, \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whltr_fr, \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whltr_rl, \
	std::shared_ptr<Wheel_Disk_Tire_Fiala> sptr_whltr_rr);
	
	void preprocess (const d_vec &u, const double &t);
	void collect_con_states();
	void calculate_and_collect_derivatives();
	void distribute_con_states();
    void postprocess();
	void output (d_vec &outputs);

private:
	std::shared_ptr<Wheel_Disk_Tire_Fiala> m_sptr_whltr_fl, \
	m_sptr_whltr_fr, \
	m_sptr_whltr_rl, \
	m_sptr_whltr_rr;

public:
	double m_t;	//subsystem time
	//for states communication with class System
	d_vec m_drvs = d_vec(m_derivatives_num, NaN);
	d_vec m_con_states = d_vec(m_con_states_num, NaN);

};

}	//end of name space
#endif //SUBSYSTEM_4WHEELTIRES_DISKFIALA_HPP
