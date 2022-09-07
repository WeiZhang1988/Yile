#ifndef SUS_Macpherson_TWO_TRACKS_HPP
#define SUS_Macpherson_TWO_TRACKS_HPP
#include "common.hpp"

namespace NMSPC{
class Sus_Macpherson_2Tracks {
public:
	static const int m_params_num = (16 - 2) * 2 + 1;		//amount of parameters
	static const int m_inputs_num = 13 * 2;					//amount of inputs
	static const int m_con_states_num = 0;					//amount of continuous states;
	static const int m_derivatives_num = m_con_states_num;	//amount of derivatives;
	static const int m_dis_states_num = 0;					//amount of discrete states;
	static const int m_outputs_num = 15 * 2;				//amount of outputs



	Sus_Macpherson_2Tracks (double F0z=0.0, double Kz=52451.0, double Cz=1.0, \
	double Hmax=0.1, double roll_strg_H_slp=0.0, \
	double toe=0.0, double toe_strg_slp=0.0, \
	double caster=0.0, double caster_H_slp=0.0, double caster_strg_slp=0.0, \
	double camber=0.0, double camber_H_slp=0.0, double camber_strg_slp=0.0, \
	double strg_hgt_slp=0.0, \
	bool is_strg=false, \
	double t=0.0) :
	m_F0z_l(F0z), m_Kz_l(Kz), m_Cz_l(Cz), m_Hmax_l(Hmax), \
	m_roll_strg_H_slp_l(roll_strg_H_slp), \
	m_toe_l(toe), m_toe_strg_slp_l(toe_strg_slp), \
	m_caster_l(caster), m_caster_H_slp_l(caster_H_slp), \
	m_caster_strg_slp_l(caster_strg_slp), \
	m_camber_l(camber), m_camber_H_slp_l(camber_H_slp), \
	m_camber_strg_slp_l(camber_strg_slp), \
	m_strg_hgt_slp_l(strg_hgt_slp), \
	m_F0z_r(F0z), m_Kz_r(Kz), m_Cz_r(Cz), m_Hmax_r(Hmax), \
	m_roll_strg_H_slp_r(roll_strg_H_slp), \
	m_toe_r(toe), m_toe_strg_slp_r(toe_strg_slp), \
	m_caster_r(caster), m_caster_H_slp_r(caster_H_slp), \
	m_caster_strg_slp_r(caster_strg_slp), \
	m_camber_r(camber), m_camber_H_slp_r(camber_H_slp), \
	m_camber_strg_slp_r(camber_strg_slp), \
	m_strg_hgt_slp_r(strg_hgt_slp), \
	m_is_strg(is_strg), \
	m_t(t) \
	{	
		if (!is_strg) {
			m_strg_hgt_slp_l = 0.0;
			m_strg_hgt_slp_r = 0.0;
			}
	}
	
	Sus_Macpherson_2Tracks (const d_vec &params, const bool is_strg, \
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
	// preload , spring constant , damping , maximum height
	double m_F0z_l, m_Kz_l, m_Cz_l, m_Hmax_l, \
	m_F0z_r, m_Kz_r, m_Cz_r, m_Hmax_r; 		
	// rolling steering angle vs suspension height
	double m_roll_strg_H_slp_l, \
	m_roll_strg_H_slp_r; 
	double m_toe_l, m_toe_strg_slp_l, \
	m_toe_r, m_toe_strg_slp_r; 		//
	double m_caster_l, m_caster_H_slp_l, m_caster_strg_slp_l, \
	m_caster_r, m_caster_H_slp_r, m_caster_strg_slp_r; 
	double m_camber_l, m_camber_H_slp_l, m_camber_strg_slp_l, \
	m_camber_r, m_camber_H_slp_r, m_camber_strg_slp_r; 
	//suspension height vs steering angle
	double m_strg_hgt_slp_l, m_strg_hgt_slp_r;
	// is steering 			
	bool m_is_strg; 				
	
	//inputs
	//--left
	double m_whl_Pz_l   = NaN;
	double m_whl_Re_l   = NaN;
	double m_whl_Vz_l   = NaN;
	double m_whl_Fx_l   = NaN;
	double m_whl_Fy_l   = NaN;
	double m_whl_Mx_l   = NaN;
	double m_whl_My_l   = NaN;
	double m_whl_Mz_l   = NaN;
	double m_veh_Pz_l   = NaN;
	double m_veh_Vx_l   = NaN;
	double m_veh_Vy_l   = NaN;
	double m_veh_Vz_l   = NaN;
	double m_strg_ang_l = NaN;
	//--right
	double m_whl_Pz_r   = NaN;
	double m_whl_Re_r   = NaN;
	double m_whl_Vz_r   = NaN;
	double m_whl_Fx_r   = NaN;
	double m_whl_Fy_r   = NaN;
	double m_whl_Mx_r   = NaN;
	double m_whl_My_r   = NaN;
	double m_whl_Mz_r   = NaN;
	double m_veh_Pz_r   = NaN;
	double m_veh_Vx_r   = NaN;
	double m_veh_Vy_r   = NaN;
	double m_veh_Vz_r   = NaN;
	double m_strg_ang_r = NaN;
	
	//outputs
	//--left
	double m_veh_Fx_l = NaN;
	double m_veh_Fy_l = NaN;	
	double m_veh_Fz_l = NaN;
	double m_veh_Mx_l = NaN;
	double m_veh_My_l = NaN;	
	double m_veh_Mz_l = NaN;	
	//double m_whl_Fx_l = NaN;
	//double m_whl_Fy_l = NaN;	
	double m_whl_Fz_l = NaN;
	double m_whl_Vx_l = NaN;
	double m_whl_Vy_l = NaN;	
	//double m_whl_Vz_l = NaN;	
	double m_whl_camber_l = NaN;
	double m_whl_caster_l = NaN;
	double m_whl_strg_l   = NaN;
	//--right
	double m_veh_Fx_r = NaN;
	double m_veh_Fy_r = NaN;	
	double m_veh_Fz_r = NaN;
	double m_veh_Mx_r = NaN;
	double m_veh_My_r = NaN;	
	double m_veh_Mz_r = NaN;	
	//double m_whl_Fx_r = NaN;
	//double m_whl_Fy_r = NaN;	
	double m_whl_Fz_r = NaN;
	double m_whl_Vx_r = NaN;
	double m_whl_Vy_r = NaN;	
	//double m_whl_Vz_r = NaN;	
	double m_whl_camber_r = NaN;
	double m_whl_caster_r = NaN;
	double m_whl_strg_r   = NaN;
	
	//middle variables
	//--left
	double m_veh_h_l  = NaN;	//ambiguous name. actually it means the suppression amount without Fz0
	double m_sus_h_l = NaN;	//ambiguous name. actually it means the total suppresion amount of suspension.
	double m_x_dot_l = NaN;
	double m_x_minus_hmax_l = NaN;
	double m_x_plus_hmax_l = NaN;
	double m_hard_stop_force_l = NaN;
	double m_total_effort_l = NaN;
	double m_adjusted_toe_l = NaN;
	double m_arm_l = NaN;
	//--right
	double m_veh_h_r  = NaN;	//ambiguous name. actually it means the suppression amount without Fz0
	double m_sus_h_r = NaN;	//ambiguous name. actually it means the total suppresion amount of suspension.
	double m_x_dot_r = NaN;
	double m_x_minus_hmax_r = NaN;
	double m_x_plus_hmax_r = NaN;
	double m_hard_stop_force_r = NaN;
	double m_total_effort_r = NaN;
	double m_adjusted_toe_r = NaN;
	double m_arm_r = NaN;

	//
	double calculate_hard_stop_force(const double &x_morp_hmax, \
	const double &x_dot, const double &Hmax, const double &Kz, const double &Cz);

public:
	double m_t;	//suspension time
	
};

}	//end of name space
#endif //SUS_Macpherson_TWO_TRACKS_HPP
