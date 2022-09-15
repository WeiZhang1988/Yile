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
#ifndef INTERFACE_TIRE_4FIALA_HPP
#define INTERFACE_TIRE_4FIALA_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_tir_4fiala.hpp"

namespace NMSPC{
class Int_Tir_4Fiala {
public:
    //tir pv pull
	double m_Tir_omega_fl = NaN;
    double m_Tir_rhoz_fl = NaN;
    double m_Tir_Re_fl = NaN;
    double m_Sus_vx_fl = NaN;
    double m_Sus_vy_fl = NaN;
    double m_Sus_vz_fl = NaN;
	double m_Sus_gamma_fl = NaN;
    double m_Sus_str_fl = NaN;
    double m_Sus_r_fl = NaN;
    double m_Tir_omega_fr = NaN;
    double m_Tir_rhoz_fr = NaN;
    double m_Tir_Re_fr = NaN;
    double m_Sus_vx_fr = NaN;
    double m_Sus_vy_fr = NaN;
    double m_Sus_vz_fr = NaN;
	double m_Sus_gamma_fr = NaN;
    double m_Sus_str_fr = NaN;
    double m_Sus_r_fr = NaN;
    double m_Tir_omega_rl = NaN;
    double m_Tir_rhoz_rl = NaN;
    double m_Tir_Re_rl = NaN;
    double m_Sus_vx_rl = NaN;
    double m_Sus_vy_rl = NaN;
    double m_Sus_vz_rl = NaN;
	double m_Sus_gamma_rl = NaN;
    double m_Sus_str_rl = NaN;
    double m_Sus_r_rl = NaN; 
    double m_Tir_omega_rr = NaN;
    double m_Tir_rhoz_rr = NaN;
    double m_Tir_Re_rr = NaN;
    double m_Sus_vx_rr = NaN;
    double m_Sus_vy_rr = NaN;
    double m_Sus_vz_rr = NaN;
	double m_Sus_gamma_rr = NaN;
    double m_Sus_str_rr = NaN;
    double m_Sus_r_rr = NaN;
    //tir pv push nothing
    //tir fm pull
    double m_Sus_Fz_fl = NaN;
    double m_Gnd_scale_fl = NaN;
    double m_Tir_Prs_fl = NaN;
    double m_Air_Tamb_fl = NaN; 
    double m_Sus_Fz_fr = NaN;
    double m_Gnd_scale_fr = NaN;
    double m_Tir_Prs_fr = NaN;
    double m_Air_Tamb_fr = NaN; 
    double m_Sus_Fz_rl = NaN;
    double m_Gnd_scale_rl = NaN;
    double m_Tir_Prs_rl = NaN;
    double m_Air_Tamb_rl = NaN; 
    double m_Sus_Fz_rr = NaN;
    double m_Gnd_scale_rr = NaN;
    double m_Tir_Prs_rr = NaN;
    double m_Air_Tamb_rr = NaN;
    //tir fm push
    double m_Sus_TirFx_fl = NaN;
    double m_Sus_TirFy_fl = NaN;
    double m_Sus_TirFz_fl = NaN;
    double m_Tir_Fx_fl = NaN;
    double m_Tir_Fy_fl = NaN;
    double m_Tir_Fz_fl = NaN;
    double m_Tir_Mx_fl = NaN;
    double m_Tir_My_fl = NaN;
    double m_Tir_Mz_fl = NaN;
    double m_Sus_TirFx_fr = NaN;
    double m_Sus_TirFy_fr = NaN;
    double m_Sus_TirFz_fr = NaN;
    double m_Tir_Fx_fr = NaN;
    double m_Tir_Fy_fr = NaN;
    double m_Tir_Fz_fr = NaN;
    double m_Tir_Mx_fr = NaN;
    double m_Tir_My_fr = NaN;
    double m_Tir_Mz_fr = NaN;
    double m_Sus_TirFx_rl = NaN;
    double m_Sus_TirFy_rl = NaN;
    double m_Sus_TirFz_rl = NaN;
    double m_Tir_Fx_rl = NaN;
    double m_Tir_Fy_rl = NaN;
    double m_Tir_Fz_rl = NaN;
    double m_Tir_Mx_rl = NaN;
    double m_Tir_My_rl = NaN;
    double m_Tir_Mz_rl = NaN;
    double m_Sus_TirFx_rr = NaN;
    double m_Sus_TirFy_rr = NaN;
    double m_Sus_TirFz_rr = NaN;
    double m_Tir_Fx_rr = NaN;
    double m_Tir_Fy_rr = NaN;
    double m_Tir_Fz_rr = NaN;
    double m_Tir_Mx_rr = NaN;
    double m_Tir_My_rr = NaN;
    double m_Tir_Mz_rr = NaN;
};

}//end of name space 
#endif //INTERFACE_TIRE_4FIALA_HPP
