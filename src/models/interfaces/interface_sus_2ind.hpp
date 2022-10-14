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
#ifndef INTERFACE_SUS_2IND_HPP
#define INTERFACE_SUS_2IND_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_sus_2ind.hpp"

namespace NMSPC{
class Int_Sus_2Ind {
public:
    //sus pv pull
	double m_Veh_hgt_cg   = NaN;            //from vehicle body        
    double m_Veh_r        = NaN;            //from vehicle body  
    double m_Strg_str_fl  = NaN;            //from steer
    double m_Sus_TirPz_fl = NaN;            //from wheel
    double m_Sus_Tirvz_fl = NaN;            //from wheel
    double m_Tir_Re_fl    = NaN;            //from wheel
    double m_Int_Pz_fl    = NaN;            //from vehicle body
    double m_Int_Vz_fl    = NaN;            //from vehicle body
    double m_Veh_vx_fl    = NaN;            //from vehicle body
    double m_Veh_vy_fl    = NaN;            //from vehicle body
    double m_Veh_vz_fl    = NaN;            //from vehicle body
    double m_Strg_str_fr  = NaN;            //from steer
    double m_Sus_TirPz_fr = NaN;            //from wheel
    double m_Sus_Tirvz_fr = NaN;            //from wheel
    double m_Tir_Re_fr    = NaN;            //from wheel
    double m_Int_Pz_fr    = NaN;            //from vehicle body
    double m_Int_Vz_fr    = NaN;            //from vehicle body
    double m_Veh_vx_fr    = NaN;            //from vehicle body
    double m_Veh_vy_fr    = NaN;            //from vehicle body
    double m_Veh_vz_fr    = NaN;            //from vehicle body
    double m_Strg_str_rl  = NaN;            //from steer
    double m_Sus_TirPz_rl = NaN;            //from wheel
    double m_Sus_Tirvz_rl = NaN;            //from wheel
    double m_Tir_Re_rl    = NaN;            //from wheel
    double m_Int_Pz_rl    = NaN;            //from vehicle body
    double m_Int_Vz_rl    = NaN;            //from vehicle body
    double m_Veh_vx_rl    = NaN;            //from vehicle body
    double m_Veh_vy_rl    = NaN;            //from vehicle body
    double m_Veh_vz_rl    = NaN;            //from vehicle body
    double m_Strg_str_rr  = NaN;            //from steer
    double m_Sus_TirPz_rr = NaN;            //from wheel
    double m_Sus_Tirvz_rr = NaN;            //from wheel
    double m_Tir_Re_rr    = NaN;            //from wheel
    double m_Int_Pz_rr    = NaN;            //from vehicle body
    double m_Int_Vz_rr    = NaN;            //from vehicle body
    double m_Veh_vx_rr    = NaN;            //from vehicle body
    double m_Veh_vy_rr    = NaN;            //from vehicle body
    double m_Veh_vz_rr    = NaN;            //from vehicle body

    //sus pv push
    double m_Sus_str_fl   = NaN;            //to tire
    double m_Sus_gamma_fl = NaN;            //to tire
    double m_Sus_caster_fl= NaN;            //not consumed
    double m_Sus_r_fl     = NaN;            //to tire
    double m_Sus_vx_fl    = NaN;            //to tire
    double m_Sus_vy_fl    = NaN;            //to tire
    double m_Sus_vz_fl    = NaN;            //to tire
	double m_Sus_str_fr   = NaN;            //to tire
    double m_Sus_gamma_fr = NaN;            //to tire
    double m_Sus_caster_fr= NaN;            //not consumed
    double m_Sus_r_fr     = NaN;            //to tire
    double m_Sus_vx_fr    = NaN;            //to tire
    double m_Sus_vy_fr    = NaN;            //to tire
    double m_Sus_vz_fr    = NaN;            //to tire
    double m_Sus_str_rl   = NaN;            //to tire
    double m_Sus_gamma_rl = NaN;            //to tire
    double m_Sus_caster_rl= NaN;            //not consumed
    double m_Sus_r_rl     = NaN;            //to tire
    double m_Sus_vx_rl    = NaN;            //to tire
    double m_Sus_vy_rl    = NaN;            //to tire
    double m_Sus_vz_rl    = NaN;            //to tire
	double m_Sus_str_rr   = NaN;            //to tire
    double m_Sus_gamma_rr = NaN;            //to tire
    double m_Sus_caster_rr= NaN;            //not consumed
    double m_Sus_r_rr     = NaN;            //to tire
    double m_Sus_vx_rr    = NaN;            //to tire
    double m_Sus_vy_rr    = NaN;            //to tire
    double m_Sus_vz_rr    = NaN;            //to tire

    //sus fm pull
    double m_Sus_TirFx_fl  = NaN;           //from tire
    double m_Sus_TirFy_fl  = NaN;           //from tire
    double m_Tir_Mx_fl     = NaN;           //from tire
    double m_Tir_My_fl     = NaN;           //from tire
    double m_Tir_Mz_fl     = NaN;           //from tire
	double m_Sus_TirFx_fr  = NaN;           //from tire
    double m_Sus_TirFy_fr  = NaN;           //from tire
    double m_Tir_Mx_fr     = NaN;           //from tire
    double m_Tir_My_fr     = NaN;           //from tire
    double m_Tir_Mz_fr     = NaN;           //from tire
    double m_Sus_TirFx_rl  = NaN;           //from tire
    double m_Sus_TirFy_rl  = NaN;           //from tire
    double m_Tir_Mx_rl     = NaN;           //from tire
    double m_Tir_My_rl     = NaN;           //from tire
    double m_Tir_Mz_rl     = NaN;           //from tire
	double m_Sus_TirFx_rr  = NaN;           //from tire
    double m_Sus_TirFy_rr  = NaN;           //from tire
    double m_Tir_Mx_rr     = NaN;           //from tire
    double m_Tir_My_rr     = NaN;           //from tire
    double m_Tir_Mz_rr     = NaN;           //from tire

    //sus fm push
    double m_Sus_VehFx_fl  =NaN;            //to vehicle body
    double m_Sus_VehFy_fl  =NaN;            //to vehicle body 
    double m_Sus_VehFz_fl  =NaN;            //to vehicle body
    double m_Sus_VehMx_fl  =NaN;            //to vehicle body
    double m_Sus_VehMy_fl  =NaN;            //to vehicle body
    double m_Sus_VehMz_fl  =NaN;            //to vehicle body
    double m_Sus_Fz_fl     =NaN;            //to wheel and tire
	double m_Sus_VehFx_fr  =NaN;            //to vehicle body 
    double m_Sus_VehFy_fr  =NaN;            //to vehicle body 
    double m_Sus_VehFz_fr  =NaN;            //to vehicle body 
    double m_Sus_VehMx_fr  =NaN;            //to vehicle body
    double m_Sus_VehMy_fr  =NaN;            //to vehicle body
    double m_Sus_VehMz_fr  =NaN;            //to vehicle body
    double m_Sus_Fz_fr     =NaN;            //to wheel and tire
    double m_Sus_VehFx_rl  =NaN;            //to vehicle body 
    double m_Sus_VehFy_rl  =NaN;            //to vehicle body 
    double m_Sus_VehFz_rl  =NaN;            //to vehicle body
    double m_Sus_VehMx_rl  =NaN;            //to vehicle body
    double m_Sus_VehMy_rl  =NaN;            //to vehicle body
    double m_Sus_VehMz_rl  =NaN;            //to vehicle body
    double m_Sus_Fz_rl     =NaN;            //to wheel and tire
	double m_Sus_VehFx_rr  =NaN;            //to vehicle body 
    double m_Sus_VehFy_rr  =NaN;            //to vehicle body 
    double m_Sus_VehFz_rr  =NaN;            //to vehicle body
    double m_Sus_VehMx_rr  =NaN;            //to vehicle body
    double m_Sus_VehMy_rr  =NaN;            //to vehicle body
    double m_Sus_VehMz_rr  =NaN;            //to vehicle body
    double m_Sus_Fz_rr     =NaN;            //to wheel and tire
};

}//end of name space 
#endif //INTERFACE_SUS_2IND_HPP
