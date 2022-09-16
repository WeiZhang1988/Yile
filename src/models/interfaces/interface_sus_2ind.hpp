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
	double m_Veh_hgt_cg   = NaN;
    double m_Veh_r        = NaN;
    double m_Strg_str_fl  = NaN; 
    double m_Tir_Pz_fl    = NaN;
    double m_Tir_vz_fl    = NaN;
    double m_Tir_Re_fl    = NaN;
    double m_Veh_Pz_fl    = NaN;
    double m_Veh_vx_fl    = NaN;
    double m_Veh_vy_fl    = NaN;
    double m_Veh_vz_fl    = NaN;
    double m_Strg_str_fr  = NaN;
    double m_Tir_Pz_fr    = NaN;
    double m_Tir_vz_fr    = NaN;
    double m_Tir_Re_fr    = NaN;
    double m_Veh_Pz_fr    = NaN;
    double m_Veh_vx_fr    = NaN;
    double m_Veh_vy_fr    = NaN;
    double m_Veh_vz_fr    = NaN;
    double m_Strg_str_rl  = NaN;
    double m_Tir_Pz_rl    = NaN;
    double m_Tir_vz_rl    = NaN;
    double m_Tir_Re_rl    = NaN;
    double m_Veh_Pz_rl    = NaN;
    double m_Veh_vx_rl    = NaN;
    double m_Veh_vy_rl    = NaN;
    double m_Veh_vz_rl    = NaN;
    double m_Strg_str_rr  = NaN;
    double m_Tir_Pz_rr    = NaN;
    double m_Tir_vz_rr    = NaN;
    double m_Tir_Re_rr    = NaN;
    double m_Veh_Pz_rr    = NaN;
    double m_Veh_vx_rr    = NaN;
    double m_Veh_vy_rr    = NaN;
    double m_Veh_vz_rr    = NaN;

    //sus pv push
    double m_Sus_str_fl   = NaN;
    double m_Sus_gamma_fl = NaN;
    double m_Sus_caster_fl= NaN;
    double m_Sus_r_fl     = NaN;
    double m_Sus_vx_fl    = NaN;
    double m_Sus_vy_fl    = NaN;
    double m_Sus_vz_fl    = NaN;
	double m_Sus_str_fr   = NaN;
    double m_Sus_gamma_fr = NaN;
    double m_Sus_caster_fr= NaN;
    double m_Sus_r_fr     = NaN;
    double m_Sus_vx_fr    = NaN;
    double m_Sus_vy_fr    = NaN;
    double m_Sus_vz_fr    = NaN;
    double m_Sus_str_rl   = NaN;
    double m_Sus_gamma_rl = NaN;
    double m_Sus_caster_rl= NaN;
    double m_Sus_r_rl     = NaN;
    double m_Sus_vx_rl    = NaN;
    double m_Sus_vy_rl    = NaN;
    double m_Sus_vz_rl    = NaN;
	double m_Sus_str_rr   = NaN;
    double m_Sus_gamma_rr = NaN;
    double m_Sus_caster_rr= NaN;
    double m_Sus_r_rr     = NaN;
    double m_Sus_vx_rr    = NaN;
    double m_Sus_vy_rr    = NaN;
    double m_Sus_vz_rr    = NaN;

    //sus fm pull
    double m_Sus_TirFx_fl  = NaN; 
    double m_Sus_TirFy_fl  = NaN; 
    double m_Tir_Mx_fl     = NaN; 
    double m_Tir_My_fl     = NaN; 
    double m_Tir_Mz_fl     = NaN;
	double m_Sus_TirFx_fr  = NaN; 
    double m_Sus_TirFy_fr  = NaN; 
    double m_Tir_Mx_fr     = NaN; 
    double m_Tir_My_fr     = NaN; 
    double m_Tir_Mz_fr     = NaN;
    double m_Sus_TirFx_rl  = NaN; 
    double m_Sus_TirFy_rl  = NaN; 
    double m_Tir_Mx_rl     = NaN; 
    double m_Tir_My_rl     = NaN; 
    double m_Tir_Mz_rl     = NaN;
	double m_Sus_TirFx_rr  = NaN; 
    double m_Sus_TirFy_rr  = NaN; 
    double m_Tir_Mx_rr     = NaN; 
    double m_Tir_My_rr     = NaN; 
    double m_Tir_Mz_rr     = NaN; 

    //sus fm push
    double m_Sus_VehFx_fl  =NaN; 
    double m_Sus_VehFy_fl  =NaN; 
    double m_Sus_VehFz_fl  =NaN; 
    double m_Sus_VehMx_fl  =NaN; 
    double m_Sus_VehMy_fl  =NaN; 
    double m_Sus_VehMz_fl  =NaN; 
    double m_Sus_Fz_fl     =NaN;
	double m_Sus_VehFx_fr  =NaN; 
    double m_Sus_VehFy_fr  =NaN; 
    double m_Sus_VehFz_fr  =NaN; 
    double m_Sus_VehMx_fr  =NaN; 
    double m_Sus_VehMy_fr  =NaN; 
    double m_Sus_VehMz_fr  =NaN; 
    double m_Sus_Fz_fr     =NaN;
    double m_Sus_VehFx_rl  =NaN; 
    double m_Sus_VehFy_rl  =NaN; 
    double m_Sus_VehFz_rl  =NaN; 
    double m_Sus_VehMx_rl  =NaN; 
    double m_Sus_VehMy_rl  =NaN; 
    double m_Sus_VehMz_rl  =NaN; 
    double m_Sus_Fz_rl     =NaN;
	double m_Sus_VehFx_rr  =NaN; 
    double m_Sus_VehFy_rr  =NaN; 
    double m_Sus_VehFz_rr  =NaN; 
    double m_Sus_VehMx_rr  =NaN; 
    double m_Sus_VehMy_rr  =NaN; 
    double m_Sus_VehMz_rr  =NaN; 
    double m_Sus_Fz_rr     =NaN;
};

}//end of name space 
#endif //INTERFACE_SUS_2IND_HPP
