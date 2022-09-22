// =============================================================================
// PROJECT YILE - 
//
// Copyright (c) 2023
// All rights reserved.
//
// Use of this source code is governed by a GPL-3.0 license that can be found
// in the LICENSE file
//
// Author of this file	Wei ZHANG wei_zhang_1988@outlook.com
//
// =============================================================================
#ifndef INTERFACE_ENTIRE_DISK_FIALA_HPP
#define INTERFACE_ENTIRE_DISK_FIALA_HPP
#include "components/common.hpp"
#include "components/vehicle_body.hpp"
#include "subsystems/subsystem_sus_2ind.hpp"
#include "subsystems/subsystem_whl_4disk.hpp"
#include "subsystems/subsystem_tir_4fiala.hpp"

namespace NMSPC{
class Int_Entire_Disk_Fiala {
public:
    //////////G E N E R A T E D  B Y   E N V & E X T E R N A L//////////////////////////
    //whl pv pull
	double m_Gnd_Pz_fl = NaN;               //Env.Gnd to wheel
    double m_Gnd_Pz_fr = NaN;               //Env.Gnd to wheel
    double m_Gnd_Pz_rl = NaN;               //Env.Gnd to wheel
    double m_Gnd_Pz_rr = NaN;               //Env.Gnd to wheel
    //tir fm pull
    double m_Gnd_scale_fl = NaN;            //Env.Gnd to tire
    double m_Tir_Prs_fl = NaN;              //Env.TirPrs to tire
    double m_Air_Tamb_fl = NaN;             //Env.Air to tire
    double m_Gnd_scale_fr = NaN;            //Env.Gnd to tire
    double m_Tir_Prs_fr = NaN;              //Env.TirPrs to tire
    double m_Air_Tamb_fr = NaN;             //Env.Air to tire
    double m_Gnd_scale_rl = NaN;            //Env.Gnd to tire
    double m_Tir_Prs_rl = NaN;              //Env.TirPrs to tire
    double m_Air_Tamb_rl = NaN;             //Env.Air to tire
    double m_Gnd_scale_rr = NaN;            //Env.Gnd to tire
    double m_Tir_Prs_rr = NaN;              //Env.TirPrs to tire
    double m_Air_Tamb_rr = NaN;             //Env.Air to tire
    //vb pv pull
    double m_Air_Wx = NaN;                  //Env.Air to vehicle body
    double m_Air_Wy = NaN;                  //Env.Air to vehicle body
    double m_Air_Wz = NaN;                  //Env.Air to vehicle body
    //vb fm pull
    double m_Air_Tair  = NaN;               //Env.Air to vehicle body
    double m_Ext_Fx_ext = NaN;              //Env to vehicle body
    double m_Ext_Fy_ext = NaN;              //Env to vehicle body
    double m_Ext_Fz_ext = NaN;              //Env to vehicle body
    double m_Ext_Mx_ext = NaN;              //Env to vehicle body
    double m_Ext_My_ext = NaN;              //Env to vehicle body
    double m_Ext_Mz_ext = NaN;              //Env to vehicle body
    //////////G E N E R A T E D  B Y  D R I V E L I N E & S T E E R & B R A K E//////////
    //whl fm pull
    double m_Axl_Trq_fl = NaN;              //driveline to wheel
    double m_Brk_Prs_fl = NaN;              //brake to wheel
    double m_Axl_Trq_fr = NaN;              //driveline to wheel
    double m_Brk_Prs_fr = NaN;              //brake to wheel
    double m_Axl_Trq_rl = NaN;              //driveline to wheel
    double m_Brk_Prs_rl = NaN;              //brake to wheel
    double m_Axl_Trq_rr = NaN;              //driveline to wheel
    double m_Brk_Prs_rr = NaN;              //brake to wheel
    //sus pv pull
    double m_Strg_str_fl  = NaN;            //steer to suspension
    double m_Strg_str_fr  = NaN;            //steer to suspension
    double m_Strg_str_rl  = NaN;            //steer to suspension
    double m_Strg_str_rr  = NaN;            //steer to suspension
    //////////G E N E R A T E D  B Y  W H E E L//////////////////////////////////////////
    //whl pv push
    double m_Tir_omega_fl = NaN;            //wheel to tire
    double m_Sus_TirPz_fl = NaN;            //wheel to suspension
    double m_Sus_Tirvz_fl = NaN;            //wheel to suspension
    double m_Tir_rhoz_fl  = NaN;            //wheel to tire
    double m_Tir_Re_fl    = NaN;            //wheel to tire and suspension
    double m_Tir_omega_fr = NaN;            //wheel to tire
    double m_Sus_TirPz_fr = NaN;            //wheel to suspension
    double m_Sus_Tirvz_fr = NaN;            //wheel to suspension
    double m_Tir_rhoz_fr  = NaN;            //wheel to tire
    double m_Tir_Re_fr    = NaN;            //wheel to tire and suspension
    double m_Tir_omega_rl = NaN;            //wheel to tire
    double m_Sus_TirPz_rl = NaN;            //wheel to suspension
    double m_Sus_Tirvz_rl = NaN;            //wheel to suspension
    double m_Tir_rhoz_rl  = NaN;            //wheel to tire
    double m_Tir_Re_rl    = NaN;            //wheel to tire and suspension
    double m_Tir_omega_rr = NaN;            //wheel to tire
    double m_Sus_TirPz_rr = NaN;            //wheel to suspension
    double m_Sus_Tirvz_rr = NaN;            //wheel to suspension
    double m_Tir_rhoz_rr  = NaN;            //wheel to tire
    double m_Tir_Re_rr    = NaN;            //wheel to tire and suspension
    //////////G E N E R A T E D  B Y  T I R E//////////////////////////////////////////
    //tir fm push
    double m_Sus_TirFx_fl = NaN;            //tire to suspension
    double m_Sus_TirFy_fl = NaN;            //tire to suspension
    double m_Tir_Fx_fl    = NaN;            //tire to wheel
    double m_Tir_Fz_fl    = NaN;            //tire to wheel
    double m_Tir_Mx_fl    = NaN;            //tire to suspension
    double m_Tir_My_fl    = NaN;            //tire to wheel and suspension
    double m_Tir_Mz_fl    = NaN;            //tire to suspension
    double m_Sus_TirFx_fr = NaN;            //tire to suspension
    double m_Sus_TirFy_fr = NaN;            //tire to suspension
    double m_Tir_Fx_fr    = NaN;            //tire to wheel
    double m_Tir_Fz_fr    = NaN;            //tire to wheel
    double m_Tir_Mx_fr    = NaN;            //tire to suspension
    double m_Tir_My_fr    = NaN;            //tire to wheel and suspension
    double m_Tir_Mz_fr    = NaN;            //tire to suspension
    double m_Sus_TirFx_rl = NaN;            //tire to suspension
    double m_Sus_TirFy_rl = NaN;            //tire to suspnesion
    double m_Tir_Fx_rl    = NaN;            //tire to wheel
    double m_Tir_Fz_rl    = NaN;            //tire to wheel
    double m_Tir_Mx_rl    = NaN;            //tire to suspension
    double m_Tir_My_rl    = NaN;            //tire to wheel and suspension
    double m_Tir_Mz_rl    = NaN;            //tire to suspension
    double m_Sus_TirFx_rr = NaN;            //tire to suspension
    double m_Sus_TirFy_rr = NaN;            //tire to suspension
    double m_Tir_Fx_rr    = NaN;            //tire to wheel
    double m_Tir_Fz_rr    = NaN;            //tire to wheel
    double m_Tir_Mx_rr    = NaN;            //tire to suspension
    double m_Tir_My_rr    = NaN;            //tire to wheel and suspension
    double m_Tir_Mz_rr    = NaN;            //tire to suspension
    //////////G E N E R A T E D  B Y  S U S P E N S I O N//////////////////////////////
    //sus pv push
    double m_Sus_str_fl   = NaN;            //suspension to tire
    double m_Sus_gamma_fl = NaN;            //suspension to tire
    double m_Sus_r_fl     = NaN;            //suspension to tire
    double m_Sus_vx_fl    = NaN;            //suspension to tire
    double m_Sus_vy_fl    = NaN;            //suspension to tire
    double m_Sus_vz_fl    = NaN;            //suspension to tire
	double m_Sus_str_fr   = NaN;            //suspension to tire
    double m_Sus_gamma_fr = NaN;            //suspension to tire
    double m_Sus_r_fr     = NaN;            //suspension to tire
    double m_Sus_vx_fr    = NaN;            //suspension to tire
    double m_Sus_vy_fr    = NaN;            //suspension to tire
    double m_Sus_vz_fr    = NaN;            //suspension to tire
    double m_Sus_str_rl   = NaN;            //suspension to tire
    double m_Sus_gamma_rl = NaN;            //suspension to tire
    double m_Sus_r_rl     = NaN;            //suspension to tire
    double m_Sus_vx_rl    = NaN;            //suspension to tire
    double m_Sus_vy_rl    = NaN;            //suspension to tire
    double m_Sus_vz_rl    = NaN;            //suspension to tire
    double m_Sus_str_rr   = NaN;            //suspension to tire
    double m_Sus_gamma_rr = NaN;            //suspension to tire
    double m_Sus_r_rr     = NaN;            //suspension to tire
    double m_Sus_vx_rr    = NaN;            //suspension to tire
    double m_Sus_vy_rr    = NaN;            //suspension to tire
    double m_Sus_vz_rr    = NaN;            //suspension to tire
    //sus fm push
    double m_Sus_VehFx_fl  =NaN;            //suspension to vehicle body; origin: m_Sus_Fx_fl in vehicle body
    double m_Sus_VehFy_fl  =NaN;            //suspension to vehicle body; origin: m_Sus_Fy_fl in vehicle body
    double m_Sus_VehFz_fl  =NaN;            //suspension to vehicle body; origin: m_Sus_Fz_fl in vehicle body
    double m_Sus_VehMx_fl  =NaN;            //suspension to vehicle body; origin: m_Sus_Mx_fl in vehicle body
    double m_Sus_VehMy_fl  =NaN;            //suspension to vehicle body; origin: m_Sus_My_fl in vehicle body
    double m_Sus_VehMz_fl  =NaN;            //suspension to vehicle body; origin: m_Sus_Mz_fl in vehicle body
    double m_Sus_Fz_fl     =NaN;            //suspension to wheel and tire
    double m_Sus_VehFx_fr  =NaN;            //suspension to vehicle body; origin: m_Sus_Fx_fr in vehicle body
    double m_Sus_VehFy_fr  =NaN;            //suspension to vehicle body; origin: m_Sus_Fy_fr in vehicle body
    double m_Sus_VehFz_fr  =NaN;            //suspension to vehicle body; origin: m_Sus_Fz_fr in vehicle body
    double m_Sus_VehMx_fr  =NaN;            //suspension to vehicle body; origin: m_Sus_Mx_fr in vehicle body
    double m_Sus_VehMy_fr  =NaN;            //suspension to vehicle body; origin: m_Sus_My_fr in vehicle body
    double m_Sus_VehMz_fr  =NaN;            //suspension to vehicle body; origin: m_Sus_Mz_fr in vehicle body
    double m_Sus_Fz_fr     =NaN;            //suspension to wheel and tire
    double m_Sus_VehFx_rl  =NaN;            //suspension to vehicle body; origin: m_Sus_Fx_rl in vehicle body
    double m_Sus_VehFy_rl  =NaN;            //suspension to vehicle body; origin: m_Sus_Fy_rl in vehicle body
    double m_Sus_VehFz_rl  =NaN;            //suspension to vehicle body; origin: m_Sus_Fz_rl in vehicle body
    double m_Sus_VehMx_rl  =NaN;            //suspension to vehicle body; origin: m_Sus_Mx_rl in vehicle body
    double m_Sus_VehMy_rl  =NaN;            //suspension to vehicle body; origin: m_Sus_My_rl in vehicle body
    double m_Sus_VehMz_rl  =NaN;            //suspension to vehicle body; origin: m_Sus_Mz_rl in vehicle body
    double m_Sus_Fz_rl     =NaN;            //suspension to wheel and tire
	double m_Sus_VehFx_rr  =NaN;            //suspension to vehicle body; origin: m_Sus_Fx_rr in vehicle body
    double m_Sus_VehFy_rr  =NaN;            //suspension to vehicle body; origin: m_Sus_Fy_rr in vehicle body
    double m_Sus_VehFz_rr  =NaN;            //suspension to vehicle body; origin: m_Sus_Fz_rr in vehicle body
    double m_Sus_VehMx_rr  =NaN;            //suspension to vehicle body; origin: m_Sus_Mx_rr in vehicle body
    double m_Sus_VehMy_rr  =NaN;            //suspension to vehicle body; origin: m_Sus_My_rr in vehicle body
    double m_Sus_VehMz_rr  =NaN;            //suspension to vehicle body; origin: m_Sus_Mz_rr in vehicle body
    double m_Sus_Fz_rr     =NaN;            //suspension to wheel and tire
    //////////G E N E R A T E D  B Y  V E H I C L E  B O D Y//////////////////////////////
    //vb pv push
    double m_Veh_hgt_cg    = NaN;           //vehicle body to suspension; origin: m_h_c
    double m_Veh_r         = NaN;           //vehicle body to suspension; origin: m_r_c
    double m_Veh_Pz_fl     = NaN;           //vehicle body to suspension; origin: m_xb_z_fl
    double m_Veh_vx_fl     = NaN;           //vehicle body to suspension; origin: m_vb_x_fl
    double m_Veh_vy_fl     = NaN;           //vehicle body to suspension; origin: m_vb_y_fl
    double m_Veh_vz_fl     = NaN;           //vehicle body to suspension; origin: m_vb_z_fl
    double m_Veh_Pz_fr     = NaN;           //vehicle body to suspension; origin: m_xb_z_fr
    double m_Veh_vx_fr     = NaN;           //vehicle body to suspension; origin: m_vb_x_fr
    double m_Veh_vy_fr     = NaN;           //vehicle body to suspension; origin: m_vb_y_fr
    double m_Veh_vz_fr     = NaN;           //vehicle body to suspension; origin: m_vb_z_fr
    double m_Veh_Pz_rl     = NaN;           //vehicle body to suspension; origin: m_xb_z_rl
    double m_Veh_vx_rl     = NaN;           //vehicle body to suspension; origin: m_vb_x_rl
    double m_Veh_vy_rl     = NaN;           //vehicle body to suspension; origin: m_vb_y_rl
    double m_Veh_vz_rl     = NaN;           //vehicle body to suspension; origin: m_vb_z_rl
    double m_Veh_Pz_rr     = NaN;           //vehicle body to suspension; origin: m_xb_z_rr
    double m_Veh_vx_rr     = NaN;           //vehicle body to suspension; origin: m_vb_x_rr
    double m_Veh_vy_rr     = NaN;           //vehicle body to suspension; origin: m_vb_y_rr
    double m_Veh_vz_rr     = NaN;           //vehicle body to suspension; origin: m_vb_z_rr

    //////////N O T  C O N S U M E D//////////////////////////////////////////
    /////////////////////////W H E E L/////////////////////////////////
    //whl pv push
    double m_Tir_Pz_fl = NaN;               //not consumed
    double m_Tir_vz_fl = NaN;               //not consumed
    double m_Tir_Pz_fr = NaN;               //not consumed
    double m_Tir_vz_fr = NaN;               //not consumed
    double m_Tir_Pz_rl = NaN;               //not consumed
    double m_Tir_vz_rl = NaN;               //not consumed
    double m_Tir_Pz_rr = NaN;               //not consuemd
    double m_Tir_vz_rr = NaN;               //not consuemd
    //whl fm push
    double m_Brk_Trq_fl = NaN;              //not consumed
    double m_Brk_Trq_fr = NaN;              //not consumed
    double m_Brk_Trq_rl = NaN;              //not consumed
    double m_Brk_Trq_rr = NaN;              //not consumed
    /////////////////////////T I R E //////////////////////////////////
    //tir pv push nothing
    //tir fm push
    double m_Sus_TirFz_fl = NaN;            //not consumed
    double m_Tir_Fy_fl = NaN;               //not consumed
    double m_Sus_TirFz_fr = NaN;            //not consumed
    double m_Tir_Fy_fr = NaN;               //not consuemd
    double m_Sus_TirFz_rl = NaN;            //not consuemd
    double m_Tir_Fy_rl = NaN;               //not consuemd
    double m_Sus_TirFz_rr = NaN;            //not consuemd
    double m_Tir_Fy_rr = NaN;               //not consuemd
    /////////////////////////S U S P E N S I O N//////////////////////////////////
    //sus pv push
    double m_Sus_caster_fl= NaN;            //not consumed
    double m_Sus_caster_fr= NaN;            //not consumed
    double m_Sus_caster_rl= NaN;            //not consumed
    double m_Sus_caster_rr= NaN;            //not consumed
    //sus fm push
    /////////////////////////V E H I C L E  B O D Y//////////////////////////////////
    //vb pv push
    double m_xe_x_c = NaN;            //not consumed
    double m_xe_y_c = NaN;            //not consumed
    double m_xe_z_c = NaN;            //not consumed
    double m_ve_x_c = NaN;            //not consumed
    double m_ve_y_c = NaN;            //not consumed
    double m_ve_z_c = NaN;            //not consumed
    double m_vb_x_c = NaN;            //not consumed
    double m_vb_y_c = NaN;            //not consumed
    double m_vb_z_c = NaN;            //not consumed
    double m_phai_c = NaN;            //not consumed
    double m_theta_c = NaN;           //not consumed
    double m_psi_c = NaN;             //not consumed
    double m_p_c = NaN;               //not consumed
    double m_q_c = NaN;               //not consumed
    double m_beta_c = NaN;            //not consumed
    double m_xe_x_o = NaN;            //not consumed
    double m_xe_y_o = NaN;            //not consumed
    double m_xe_z_o = NaN;            //not consumed
    double m_ve_x_o = NaN;            //not consumed
    double m_ve_y_o = NaN;            //not consumed
    double m_ve_z_o = NaN;            //not consumed
    double m_xb_x_o = NaN;            //not consumed
    double m_xb_y_o = NaN;            //not consumed
    double m_xb_z_o = NaN;            //not consumed
    double m_vb_x_o = NaN;            //not consumed
    double m_vb_y_o = NaN;            //not consumed
    double m_vb_z_o = NaN;            //not consumed
    double m_beta_o = NaN;            //not consumed
    double m_xe_x_fl = NaN;           //not consumed
    double m_xe_y_fl = NaN;           //not consumed
    double m_xe_z_fl = NaN;           //not consumed
    double m_ve_x_fl = NaN;           //not consumed
    double m_ve_y_fl = NaN;           //not consumed
    double m_ve_z_fl = NaN;           //not consumed
    double m_xb_x_fl = NaN;           //not consumed
    double m_xb_y_fl = NaN;           //not consumed
    double m_xe_x_fr = NaN;           //not consumed
    double m_xe_y_fr = NaN;           //not consumed
    double m_xe_z_fr = NaN;           //not consumed
    double m_ve_x_fr = NaN;           //not consumed
    double m_ve_y_fr = NaN;           //not consumed
    double m_ve_z_fr = NaN;           //not consumed
    double m_xb_x_fr = NaN;           //not consumed
    double m_xb_y_fr = NaN;           //not consumed
    double m_xe_x_rl = NaN;           //not consumed
    double m_xe_y_rl = NaN;           //not consumed
    double m_xe_z_rl = NaN;           //not consumed
    double m_ve_x_rl = NaN;           //not consumed
    double m_ve_y_rl = NaN;           //not consumed
    double m_ve_z_rl = NaN;           //not consumed
    double m_xb_x_rl = NaN;           //not consumed
    double m_xb_y_rl = NaN;           //not consumed
    double m_xe_x_rr = NaN;           //not consumed
    double m_xe_y_rr = NaN;           //not consumed
    double m_xe_z_rr = NaN;           //not consumed
    double m_ve_x_rr = NaN;           //not consumed
    double m_ve_y_rr = NaN;           //not consumed
    double m_ve_z_rr = NaN;           //not consumed
    double m_xb_x_rr = NaN;           //not consumed
    double m_xb_y_rr = NaN;           //not consumed
    //vb fm push nothing
	
};

}//end of name space 
#endif //INTERFACE_ENTIRE_DISK_FIALA_HPP
