#ifndef SUS_Macpherson_HPP
#define SUS_Macpherson_HPP
#include "common.hpp"

class Sus_Macpherson {
public:
	static const int m_params_num = 16;	//amount of parameters
	static const int m_inputs_num = 13;	//amount of inputs
	static const int m_outputs_num = 15;	//amount of outputs



	Sus_Macpherson (double left_or_right, \
	double F0z, double Kz, double Cz, double Hmax, double \
	roll_strg_H_slp, \
	double toe, double toe_strg_slp, \
	double caster, double caster_H_slp, double caster_strg_slp, \
	double camber, double camber_H_slp, double camber_strg_slp, \
	double strg_hgt_slp, \
	bool is_strg, \
	double t) :
	m_left_or_right(left_or_right), \
	m_F0z(F0z), m_Kz(Kz), m_Cz(Cz), m_Hmax(Hmax), \
	m_roll_strg_H_slp(roll_strg_H_slp), \
	m_toe(toe), m_toe_strg_slp(toe_strg_slp), \
	m_caster(caster), m_caster_H_slp(caster_H_slp), \
	m_caster_strg_slp(caster_strg_slp), \
	m_camber(camber), m_camber_H_slp(camber_H_slp), \
	m_camber_strg_slp(camber_strg_slp), \
	m_strg_hgt_slp(strg_hgt_slp), \
	m_is_strg(is_strg), \
	m_t(t) \
	{	
		if (!is_strg)
			m_strg_hgt_slp = 0.0;
	}
	
	Sus_Macpherson (const d_vec &params, const bool is_strg, \
	const double &t);

	//functionalities of preupdate:
	//1, to update continous state vector to be current state
	//2, to grasp inputs and system time
	//3, to calculate middle ravriables and outputs
	void preupdate (const d_vec &u, const double &t);
	//functionalities of calculate_accs:
	//just to calculate rhs of a suspension system 
	//(maybe acc is not a good name)
	void calculate_derivatives(double &drv_omega) {};
	//functionalities of postupdate:
	//to update discrete state which is based on continuous update
	void postupdate () {};
	//functionalities of output:
	//to update output based on updated system states
	void output (d_vec &outputs);

private:
	//parameters
	// suspension location: left if <= 0.0, right otherwise
	double m_left_or_right; 	
	// preload , spring constant , damping , maximum height
	double m_F0z, m_Kz, m_Cz, m_Hmax; 		
	// rolling steering angle vs suspension height
	double m_roll_strg_H_slp; 
	double m_toe, m_toe_strg_slp; 		//
	double m_caster, m_caster_H_slp, m_caster_strg_slp; 
	double m_camber, m_camber_H_slp, m_camber_strg_slp; 
	//suspension height vs steering angle
	double m_strg_hgt_slp;
	// is steering 			
	bool m_is_strg; 				
	
	//inputs
	double m_whl_Pz   = NaN;
	double m_whl_Re   = NaN;
	double m_whl_Vz   = NaN;
	double m_whl_Fx   = NaN;
	double m_whl_Fy   = NaN;
	double m_whl_Mx   = NaN;
	double m_whl_My   = NaN;
	double m_whl_Mz   = NaN;
	double m_veh_Pz   = NaN;
	double m_veh_Vx   = NaN;
	double m_veh_Vy   = NaN;
	double m_veh_Vz   = NaN;
	double m_strg_ang = NaN;
	
	//outputs
	double m_veh_Fx = NaN;
	double m_veh_Fy = NaN;	
	double m_veh_Fz = NaN;
	double m_veh_Mx = NaN;
	double m_veh_My = NaN;	
	double m_veh_Mz = NaN;	
	//double m_whl_Fx = NaN;
	//double m_whl_Fy = NaN;	
	double m_whl_Fz = NaN;
	double m_whl_Vx = NaN;
	double m_whl_Vy = NaN;	
	//double m_whl_Vz = NaN;	
	double m_whl_camber = NaN;
	double m_whl_caster = NaN;
	double m_whl_strg   = NaN;
	
	//middle variables
	double m_veh_h  = NaN;	//ambiguous name. actually it means the suppression amount without Fz0
	double m_sus_h = NaN;	//ambiguous name. actually it means the total suppresion amount of suspension.
	double m_x_dot = NaN;
	double m_x_minus_hmax = NaN;
	double m_x_plus_hmax = NaN;
	double m_hard_stop_force = NaN;
	double m_total_effort = NaN;
	double m_adjusted_toe = NaN;
	double m_arm = NaN;
	
	
	//
	double calculate_hard_stop_force(const double &x_morp_hmax, \
	const double &x_dot);

public:
	double m_t;	//wheel time
	
};

#endif //SUS_Macpherson_HPP
