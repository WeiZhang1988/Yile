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
#ifndef INTERFACE_VEHICLE_BODY_HPP
#define INTERFACE_VEHICLE_BODY_HPP
#include "components/common.hpp"
#include "components/vehicle_body.hpp"

namespace NMSPC{
class Int_Vehicle_Body {
public:
	//vb pv pull
    real_Y m_Air_Wx = NaN;            //from Env.Air
    real_Y m_Air_Wy = NaN;            //from Env.Air
    real_Y m_Air_Wz = NaN;            //from Env.Air
    real_Y m_Gnd_Grade_phai = NaN;    //Env.Gnd
    real_Y m_Gnd_Grade_theta = NaN;   //Env.Gnd
    real_Y m_Gnd_Grade_psi = NaN;     //Env.Gnd

    //vb pv push
    real_Y m_Veh_hgt_cg = NaN;        //to suspension; origin: m_h_c
    real_Y m_xe_x_c = NaN;            //not consumed
    real_Y m_xe_y_c = NaN;            //not consumed
    real_Y m_xe_z_c = NaN;            //not consumed
    real_Y m_ve_x_c = NaN;            //not consumed
    real_Y m_ve_y_c = NaN;            //not consumed
    real_Y m_ve_z_c = NaN;            //not consumed
    real_Y m_vb_x_c = NaN;            //not consumed
    real_Y m_vb_y_c = NaN;            //not consumed
    real_Y m_vb_z_c = NaN;            //not consumed
    real_Y m_phai_c = NaN;            //not consumed
    real_Y m_theta_c = NaN;           //not consumed
    real_Y m_psi_c = NaN;             //not consumed
    real_Y m_p_c = NaN;               //not consumed
    real_Y m_q_c = NaN;               //not consumed
    real_Y m_Veh_r = NaN;             //to suspension; origin: m_r_c
    real_Y m_beta_c = NaN;            //not consumed
    real_Y m_xe_x_o = NaN;            //not consumed
    real_Y m_xe_y_o = NaN;            //not consumed
    real_Y m_xe_z_o = NaN;            //not consumed
    real_Y m_ve_x_o = NaN;            //not consumed
    real_Y m_ve_y_o = NaN;            //not consumed
    real_Y m_ve_z_o = NaN;            //not consumed
    real_Y m_xb_x_o = NaN;            //not consumed
    real_Y m_xb_y_o = NaN;            //not consumed
    real_Y m_xb_z_o = NaN;            //not consumed
    real_Y m_vb_x_o = NaN;            //not consumed
    real_Y m_vb_y_o = NaN;            //not consumed
    real_Y m_vb_z_o = NaN;            //not consumed
    real_Y m_beta_o = NaN;            //not consumed
    real_Y m_xe_x_fl = NaN;           //not consumed
    real_Y m_xe_y_fl = NaN;           //not consumed
    real_Y m_Int_Pz_fl = NaN;         //to suspension; origin: m_xe_z_fl
    real_Y m_ve_x_fl = NaN;           //not consumed
    real_Y m_ve_y_fl = NaN;           //not consumed
    real_Y m_Int_Vz_fl = NaN;         //to suspension; origin: m_ve_z_fl
    real_Y m_xb_x_fl = NaN;           //not consumed
    real_Y m_xb_y_fl = NaN;           //not consumed
    real_Y m_xb_z_fl = NaN;           //not consumed
    real_Y m_Veh_vx_fl = NaN;         //to suspension; origin: m_vb_x_fl
    real_Y m_Veh_vy_fl = NaN;         //to suspension; origin: m_vb_y_fl
    real_Y m_Veh_vz_fl = NaN;         //to suspension; origin: m_vb_z_fl
    real_Y m_xe_x_fr = NaN;           //not consumed
    real_Y m_xe_y_fr = NaN;           //not consumed
    real_Y m_Int_Pz_fr = NaN;         //to suspension; origin: m_xe_z_fr
    real_Y m_ve_x_fr = NaN;           //not consumed
    real_Y m_ve_y_fr = NaN;           //not consumed
    real_Y m_Int_Vz_fr = NaN;         //to suspension; origin: m_ve_z_fr
    real_Y m_xb_x_fr = NaN;           //not consumed
    real_Y m_xb_y_fr = NaN;           //not consumed
    real_Y m_xb_z_fr = NaN;           //not consumed
    real_Y m_Veh_vx_fr = NaN;         //to suspension; origin: m_vb_x_fr
    real_Y m_Veh_vy_fr = NaN;         //to suspension; origin: m_vb_y_fr
    real_Y m_Veh_vz_fr = NaN;         //to suspension; origin: m_vb_z_fr
    real_Y m_xe_x_rl = NaN;           //not consumed
    real_Y m_xe_y_rl = NaN;           //not consumed
    real_Y m_Int_Pz_rl = NaN;         //to suspension; origin: m_xe_z_rl
    real_Y m_ve_x_rl = NaN;           //not consumed
    real_Y m_ve_y_rl = NaN;           //not consumed
    real_Y m_Int_Vz_rl = NaN;         //to suspension; origin: m_ve_z_rl
    real_Y m_xb_x_rl = NaN;           //not consumed
    real_Y m_xb_y_rl = NaN;           //not consumed
    real_Y m_xb_z_rl = NaN;           //not consumed
    real_Y m_Veh_vx_rl = NaN;         //to suspension; origin: m_vb_x_rl
    real_Y m_Veh_vy_rl = NaN;         //to suspension; origin: m_vb_y_rl
    real_Y m_Veh_vz_rl = NaN;         //to suspension; origin: m_vb_z_rl
    real_Y m_xe_x_rr = NaN;           //not consumed
    real_Y m_xe_y_rr = NaN;           //not consumed
    real_Y m_Int_Pz_rr = NaN;         //to suspension; origin: m_xe_z_rr
    real_Y m_ve_x_rr = NaN;           //not consumed
    real_Y m_ve_y_rr = NaN;           //not consumed
    real_Y m_Int_Vz_rr = NaN;         //to suspension; origin: m_ve_z_rr
    real_Y m_xb_x_rr = NaN;           //not consumed
    real_Y m_xb_y_rr = NaN;           //not consumed
    real_Y m_xb_z_rr = NaN;           //not consumed
    real_Y m_Veh_vx_rr = NaN;         //to suspension; origin: m_vb_x_rr
    real_Y m_Veh_vy_rr = NaN;         //to suspension; origin: m_vb_y_rr
    real_Y m_Veh_vz_rr = NaN;         //to suspension; origin: m_vb_z_rr

    //vb fm pull
    real_Y m_Air_Tair  = NaN;         //from Env.Air
    real_Y m_Sus_VehFx_fl = NaN;      //from suspension; origin: m_Sus_Fx_fl
    real_Y m_Sus_VehFx_fr = NaN;      //from suspension; origin: m_Sus_Fx_fr
    real_Y m_Sus_VehFx_rl = NaN;      //from suspension; origin: m_Sus_Fx_rl
    real_Y m_Sus_VehFx_rr = NaN;      //from suspension; origin: m_Sus_Fx_rr
    real_Y m_Sus_VehFy_fl = NaN;      //from suspension; origin: m_Sus_Fy_fl
    real_Y m_Sus_VehFy_fr = NaN;      //from suspension; origin: m_Sus_Fy_fr
    real_Y m_Sus_VehFy_rl = NaN;      //from suspension; origin: m_Sus_Fy_rl
    real_Y m_Sus_VehFy_rr = NaN;      //from suspension; origin: m_Sus_Fy_rr
    real_Y m_Sus_VehFz_fl = NaN;      //from suspension; origin: m_Sus_Fz_fl
    real_Y m_Sus_VehFz_fr = NaN;      //from suspension; origin: m_Sus_Fz_fr
    real_Y m_Sus_VehFz_rl = NaN;      //from suspension; origin: m_Sus_Fz_rl
    real_Y m_Sus_VehFz_rr = NaN;      //from suspension; origin: m_Sus_Fz_rr
    real_Y m_Sus_VehMx_fl = NaN;      //from suspension; origin: m_Sus_Mx_fl
    real_Y m_Sus_VehMx_fr = NaN;      //from suspension; origin: m_Sus_Mx_fr
    real_Y m_Sus_VehMx_rl = NaN;      //from suspension; origin: m_Sus_Mx_rl
    real_Y m_Sus_VehMx_rr = NaN;      //from suspension; origin: m_Sus_Mx_rr
    real_Y m_Sus_VehMy_fl = NaN;      //from suspension; origin: m_Sus_My_fl
    real_Y m_Sus_VehMy_fr = NaN;      //from suspension; origin: m_Sus_My_fr
    real_Y m_Sus_VehMy_rl = NaN;      //from suspension; origin: m_Sus_My_rl
    real_Y m_Sus_VehMy_rr = NaN;      //from suspension; origin: m_Sus_My_rr
    real_Y m_Sus_VehMz_fl = NaN;      //from suspension; origin: m_Sus_Mz_fl
    real_Y m_Sus_VehMz_fr = NaN;      //from suspension; origin: m_Sus_Mz_fr
    real_Y m_Sus_VehMz_rl = NaN;      //from suspension; origin: m_Sus_Mz_rl
    real_Y m_Sus_VehMz_rr = NaN;      //from suspension; origin: m_Sus_Mz_rr
    real_Y m_Ext_Fx_ext = NaN;        //from Env
    real_Y m_Ext_Fy_ext = NaN;        //from Env
    real_Y m_Ext_Fz_ext = NaN;        //from Env
    real_Y m_Ext_Mx_ext = NaN;        //from Env
    real_Y m_Ext_My_ext = NaN;        //from Env
    real_Y m_Ext_Mz_ext = NaN;        //from Env

    //vb fm push
    //nothing to push


    //for test only
    real_Y m_drv_phai = NaN;
    real_Y m_drv_theta = NaN;
    real_Y m_drv_psi = NaN;
    real_Y m_phai = NaN;
    real_Y m_theta = NaN;
    real_Y m_psi = NaN;
};

}//end of name space 
#endif //INTERFACE_VEHICLE_BODY_HPP
