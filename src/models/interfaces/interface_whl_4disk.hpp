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
	double m_Gnd_Pz_fl = NaN;               //from Env.Gnd
    double m_Gnd_Pz_fr = NaN;               //from Env.Gnd
    double m_Gnd_Pz_rl = NaN;               //from Env.Gnd
    double m_Gnd_Pz_rr = NaN;               //from Env.Gnd
    //whl pv push
    double m_Tir_omega_fl = NaN;            //to tire
    double m_Sus_TirPz_fl = NaN;            //to suspension
    double m_Sus_Tirvz_fl = NaN;            //to suspension
    double m_Tir_Pz_fl = NaN;               //not consumed
    double m_Tir_vz_fl = NaN;               //not consumed
    double m_Tir_rhoz_fl = NaN;             //to tire
    double m_Tir_Re_fl = NaN;               //to tire and suspension
    double m_Tir_omega_fr = NaN;            //to tire
    double m_Sus_TirPz_fr = NaN;            //to suspension
    double m_Sus_Tirvz_fr = NaN;            //to suspension
    double m_Tir_Pz_fr = NaN;               //not consumed
    double m_Tir_vz_fr = NaN;               //not consumed
    double m_Tir_rhoz_fr = NaN;             //to tire
    double m_Tir_Re_fr = NaN;               //to tire and suspension
    double m_Tir_omega_rl = NaN;            //to tire
    double m_Sus_TirPz_rl = NaN;            //to suspension
    double m_Sus_Tirvz_rl = NaN;            //to suspension
    double m_Tir_Pz_rl = NaN;               //not consumed
    double m_Tir_vz_rl = NaN;               //not consumed
    double m_Tir_rhoz_rl = NaN;             //to tire
    double m_Tir_Re_rl = NaN;               //to tire and suspension
    double m_Tir_omega_rr = NaN;            //to tire
    double m_Sus_TirPz_rr = NaN;            //to suspension
    double m_Sus_Tirvz_rr = NaN;            //to suspension
    double m_Tir_Pz_rr = NaN;               //not consuemd
    double m_Tir_vz_rr = NaN;               //not consuemd
    double m_Tir_rhoz_rr = NaN;             //to tire
    double m_Tir_Re_rr = NaN;               //to tire and suspension
    //whl fm pull
    double m_Axl_Trq_fl = NaN;              //from driveline
    double m_Brk_Prs_fl = NaN;              //from brake
    double m_Tir_Fx_fl = NaN;               //from tire
    double m_Tir_My_fl = NaN;               //from tire
    double m_Tir_Fz_fl = NaN;               //from tire
    double m_Sus_Fz_fl = NaN;               //from suspension
    double m_Axl_Trq_fr = NaN;              //from driveline
    double m_Brk_Prs_fr = NaN;              //from brake
    double m_Tir_Fx_fr = NaN;               //from tire
    double m_Tir_My_fr = NaN;               //from tire
    double m_Tir_Fz_fr = NaN;               //from tire
    double m_Sus_Fz_fr = NaN;               //from suspension
    double m_Axl_Trq_rl = NaN;              //from drive line
    double m_Brk_Prs_rl = NaN;              //from brake
    double m_Tir_Fx_rl = NaN;               //from tire
    double m_Tir_My_rl = NaN;               //from tire
    double m_Tir_Fz_rl = NaN;               //from tire
    double m_Sus_Fz_rl = NaN;               //from suspension
    double m_Axl_Trq_rr = NaN;              //from driveline
    double m_Brk_Prs_rr = NaN;              //from brake
    double m_Tir_Fx_rr = NaN;               //from tire
    double m_Tir_My_rr = NaN;               //from tire
    double m_Tir_Fz_rr = NaN;               //from tire
    double m_Sus_Fz_rr = NaN;               //from suspension
    //whl fm push
    double m_Brk_Trq_fl = NaN;              //not consumed
    double m_Brk_Trq_fr = NaN;              //not consumed
    double m_Brk_Trq_rl = NaN;              //not consumed
    double m_Brk_Trq_rr = NaN;              //not consumed
};

}//end of name space 
#endif //INTERFACE_WHEEL_4DISK_HPP
