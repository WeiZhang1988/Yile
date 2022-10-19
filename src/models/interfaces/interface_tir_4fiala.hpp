// =============================================================================
// PROJECT YILE - 
//
// Copyright (c) 2023
// All rights reserved.
//
// Use of this source code is governed by a GPL-3.0 license that can be found
// in the LICENSE file
//
// Author of this file	Wei ZHANG wei_zhang_1988@outlook.com,ChangMeng Hou 945881625@qq.com
//
// =============================================================================
#ifndef INTERFACE_TIRE_4FIALA_HPP
#define INTERFACE_TIRE_4FIALA_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_tir_4fiala.hpp"

namespace NMSPC{
class Int_Tir_4Fiala {
public:
    //tir pv pull
	real_Y m_Tir_omega_fl = NaN;            //from wheel
    real_Y m_Tir_rhoz_fl = NaN;             //from wheel
    real_Y m_Tir_Re_fl = NaN;               //from wheel
    real_Y m_Sus_vx_fl = NaN;               //from suspension
    real_Y m_Sus_vy_fl = NaN;               //from suspension
    real_Y m_Sus_vz_fl = NaN;               //from suspension
	real_Y m_Sus_gamma_fl = NaN;            //from suspension          
    real_Y m_Sus_str_fl = NaN;              //from suspension
    real_Y m_Sus_r_fl = NaN;                //from suspension
    real_Y m_Tir_omega_fr = NaN;            //from wheel
    real_Y m_Tir_rhoz_fr = NaN;             //from wheel
    real_Y m_Tir_Re_fr = NaN;               //from wheel
    real_Y m_Sus_vx_fr = NaN;               //from suspension
    real_Y m_Sus_vy_fr = NaN;               //from suspension
    real_Y m_Sus_vz_fr = NaN;               //from suspension
	real_Y m_Sus_gamma_fr = NaN;            //from suspension
    real_Y m_Sus_str_fr = NaN;              //from suspension
    real_Y m_Sus_r_fr = NaN;                //from suspension
    real_Y m_Tir_omega_rl = NaN;            //from wheel
    real_Y m_Tir_rhoz_rl = NaN;             //from wheel
    real_Y m_Tir_Re_rl = NaN;               //from wheel
    real_Y m_Sus_vx_rl = NaN;               //from suspension
    real_Y m_Sus_vy_rl = NaN;               //from suspension
    real_Y m_Sus_vz_rl = NaN;               //from suspension
	real_Y m_Sus_gamma_rl = NaN;            //from suspension
    real_Y m_Sus_str_rl = NaN;              //from suspension
    real_Y m_Sus_r_rl = NaN;                //from suspension
    real_Y m_Tir_omega_rr = NaN;            //from wheel
    real_Y m_Tir_rhoz_rr = NaN;             //from wheel
    real_Y m_Tir_Re_rr = NaN;               //from wheel
    real_Y m_Sus_vx_rr = NaN;               //from suspension
    real_Y m_Sus_vy_rr = NaN;               //from suspension
    real_Y m_Sus_vz_rr = NaN;               //from suspension
	real_Y m_Sus_gamma_rr = NaN;            //from suspension
    real_Y m_Sus_str_rr = NaN;              //from suspension
    real_Y m_Sus_r_rr = NaN;                //from suspension
    //tir pv push nothing
    //tir fm pull
    real_Y m_Sus_Fz_fl = NaN;               //from suspension
    real_Y m_Gnd_scale_fl = NaN;            //from Env.Gnd
    real_Y m_Tir_Prs_fl = NaN;              //from Env.TirPrs
    real_Y m_Air_Tamb_fl = NaN;             //from Env.Air
    real_Y m_Sus_Fz_fr = NaN;               //from suspension
    real_Y m_Gnd_scale_fr = NaN;            //from Env.Gnd
    real_Y m_Tir_Prs_fr = NaN;              //from Env.TirPrs
    real_Y m_Air_Tamb_fr = NaN;             //from Env.Air
    real_Y m_Sus_Fz_rl = NaN;               //from suspension
    real_Y m_Gnd_scale_rl = NaN;            //from Env.Gnd
    real_Y m_Tir_Prs_rl = NaN;              //from Env.TirPrs
    real_Y m_Air_Tamb_rl = NaN;             //from Env.Air
    real_Y m_Sus_Fz_rr = NaN;               //from suspension
    real_Y m_Gnd_scale_rr = NaN;            //from Env.Gnd
    real_Y m_Tir_Prs_rr = NaN;              //from Env.TirPrs
    real_Y m_Air_Tamb_rr = NaN;             //from Env.Air
    //tir fm push
    real_Y m_Sus_TirFx_fl = NaN;            //to suspension
    real_Y m_Sus_TirFy_fl = NaN;            //to suspension
    real_Y m_Sus_TirFz_fl = NaN;            //not consumed
    real_Y m_Tir_Fx_fl = NaN;               //to wheel
    real_Y m_Tir_Fy_fl = NaN;               //not consumed
    real_Y m_Tir_Fz_fl = NaN;               //to wheel
    real_Y m_Tir_Mx_fl = NaN;               //to suspension
    real_Y m_Tir_My_fl = NaN;               //to wheel and suspension
    real_Y m_Tir_Mz_fl = NaN;               //to suspension
    real_Y m_Sus_TirFx_fr = NaN;            //to suspension
    real_Y m_Sus_TirFy_fr = NaN;            //to suspension
    real_Y m_Sus_TirFz_fr = NaN;            //not consumed
    real_Y m_Tir_Fx_fr = NaN;               //to wheel
    real_Y m_Tir_Fy_fr = NaN;               //not consuemd
    real_Y m_Tir_Fz_fr = NaN;               //to wheel
    real_Y m_Tir_Mx_fr = NaN;               //to suspension
    real_Y m_Tir_My_fr = NaN;               //to wheel and suspension
    real_Y m_Tir_Mz_fr = NaN;               //to suspension
    real_Y m_Sus_TirFx_rl = NaN;            //to suspension
    real_Y m_Sus_TirFy_rl = NaN;            //to suspnesion
    real_Y m_Sus_TirFz_rl = NaN;            //not consuemd
    real_Y m_Tir_Fx_rl = NaN;               //to wheel
    real_Y m_Tir_Fy_rl = NaN;               //not consuemd
    real_Y m_Tir_Fz_rl = NaN;               //to wheel
    real_Y m_Tir_Mx_rl = NaN;               //to suspension
    real_Y m_Tir_My_rl = NaN;               //to wheel and suspension
    real_Y m_Tir_Mz_rl = NaN;               //to suspension
    real_Y m_Sus_TirFx_rr = NaN;            //to suspension
    real_Y m_Sus_TirFy_rr = NaN;            //to suspension
    real_Y m_Sus_TirFz_rr = NaN;            //not consuemd
    real_Y m_Tir_Fx_rr = NaN;               //to wheel
    real_Y m_Tir_Fy_rr = NaN;               //not consuemd
    real_Y m_Tir_Fz_rr = NaN;               //to wheel
    real_Y m_Tir_Mx_rr = NaN;               //to suspension
    real_Y m_Tir_My_rr = NaN;               //to wheel and suspension
    real_Y m_Tir_Mz_rr = NaN;               //to suspension
};

}//end of name space 
#endif //INTERFACE_TIRE_4FIALA_HPP
