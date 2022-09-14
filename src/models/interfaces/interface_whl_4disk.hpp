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
#ifndef INTERFACE_WHEEL_4DISK_HPP
#define INTERFACE_WHEEL_4DISK_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_whl_4disk.hpp"

namespace NMSPC{
class Int_Whl_4Disk {
public:
    //whl pv pull
	double m_Gnd_Pz_fl = NaN;
    double m_Gnd_Pz_fr = NaN;
    double m_Gnd_Pz_rl = NaN;
    double m_Gnd_Pz_rr = NaN;
    //whl pv push
    double m_Tir_omega_fl = NaN;
    double m_Tir_Pz_fl = NaN;
    double m_Tir_vz_fl = NaN;
    double m_Tir_rhoz_fl = NaN;
    double m_Tir_Re_fl = NaN;
    double m_Tir_omega_fr = NaN;
    double m_Tir_Pz_fr = NaN;
    double m_Tir_vz_fr = NaN;
    double m_Tir_rhoz_fr = NaN;
    double m_Tir_Re_fr = NaN;
    double m_Tir_omega_rl = NaN;
    double m_Tir_Pz_rl = NaN;
    double m_Tir_vz_rl = NaN;
    double m_Tir_rhoz_rl = NaN;
    double m_Tir_Re_rl = NaN;
    double m_Tir_omega_rr = NaN;
    double m_Tir_Pz_rr = NaN;
    double m_Tir_vz_rr = NaN;
    double m_Tir_rhoz_rr = NaN;
    double m_Tir_Re_rr = NaN;
    //whl fm pull
    double m_Axl_Trq_fl = NaN;
    double m_Brk_Prs_fl = NaN;
    double m_Tir_Fx_fl = NaN;
    double m_Tir_My_fl = NaN;
    double m_Tir_Fz_fl = NaN;
    double m_Sus_Fz_fl = NaN;
    double m_Axl_Trq_fr = NaN;
    double m_Brk_Prs_fr = NaN;
    double m_Tir_Fx_fr = NaN;
    double m_Tir_My_fr = NaN;
    double m_Tir_Fz_fr = NaN;
    double m_Sus_Fz_fr = NaN;
    double m_Axl_Trq_rl = NaN;
    double m_Brk_Prs_rl = NaN;
    double m_Tir_Fx_rl = NaN;
    double m_Tir_My_rl = NaN;
    double m_Tir_Fz_rl = NaN;
    double m_Sus_Fz_rl = NaN;
    double m_Axl_Trq_rr = NaN;
    double m_Brk_Prs_rr = NaN;
    double m_Tir_Fx_rr = NaN;
    double m_Tir_My_rr = NaN;
    double m_Tir_Fz_rr = NaN;
    double m_Sus_Fz_rr = NaN;
    //whl fm push
    double m_Brk_Trq_fl = NaN;
    double m_Brk_Trq_fr = NaN;
    double m_Brk_Trq_rl = NaN;
    double m_Brk_Trq_rr = NaN;
};

}//end of name space 
#endif //INTERFACE_WHEEL_4DISK_HPP
