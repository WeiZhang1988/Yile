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
#ifndef INTERFACE_WHEEL_4DISK_HPP
#define INTERFACE_WHEEL_4DISK_HPP
#include "components/common.hpp"
#include "subsystems/subsystem_whl_4disk.hpp"

namespace NMSPC{
class Int_Whl_4Disk {
public:
    //whl pv pull
	real_Y m_Gnd_Pz_fl = NaN;               //from Env.Gnd
    real_Y m_Gnd_Pz_fr = NaN;               //from Env.Gnd
    real_Y m_Gnd_Pz_rl = NaN;               //from Env.Gnd
    real_Y m_Gnd_Pz_rr = NaN;               //from Env.Gnd
    //whl pv push
    real_Y m_Tir_omega_fl = NaN;            //to tire
    real_Y m_Sus_TirPz_fl = NaN;            //to suspension
    real_Y m_Sus_Tirvz_fl = NaN;            //to suspension
    real_Y m_Tir_Pz_fl = NaN;               //not consumed
    real_Y m_Tir_vz_fl = NaN;               //not consumed
    real_Y m_Tir_rhoz_fl = NaN;             //to tire
    real_Y m_Tir_Re_fl = NaN;               //to tire and suspension
    real_Y m_Tir_omega_fr = NaN;            //to tire
    real_Y m_Sus_TirPz_fr = NaN;            //to suspension
    real_Y m_Sus_Tirvz_fr = NaN;            //to suspension
    real_Y m_Tir_Pz_fr = NaN;               //not consumed
    real_Y m_Tir_vz_fr = NaN;               //not consumed
    real_Y m_Tir_rhoz_fr = NaN;             //to tire
    real_Y m_Tir_Re_fr = NaN;               //to tire and suspension
    real_Y m_Tir_omega_rl = NaN;            //to tire
    real_Y m_Sus_TirPz_rl = NaN;            //to suspension
    real_Y m_Sus_Tirvz_rl = NaN;            //to suspension
    real_Y m_Tir_Pz_rl = NaN;               //not consumed
    real_Y m_Tir_vz_rl = NaN;               //not consumed
    real_Y m_Tir_rhoz_rl = NaN;             //to tire
    real_Y m_Tir_Re_rl = NaN;               //to tire and suspension
    real_Y m_Tir_omega_rr = NaN;            //to tire
    real_Y m_Sus_TirPz_rr = NaN;            //to suspension
    real_Y m_Sus_Tirvz_rr = NaN;            //to suspension
    real_Y m_Tir_Pz_rr = NaN;               //not consuemd
    real_Y m_Tir_vz_rr = NaN;               //not consuemd
    real_Y m_Tir_rhoz_rr = NaN;             //to tire
    real_Y m_Tir_Re_rr = NaN;               //to tire and suspension
    //whl fm pull
    real_Y m_Axl_Trq_fl = NaN;              //from driveline
    real_Y m_Brk_Prs_fl = NaN;              //from brake
    real_Y m_Tir_Fx_fl = NaN;               //from tire
    real_Y m_Tir_My_fl = NaN;               //from tire
    real_Y m_Tir_Fz_fl = NaN;               //from tire
    real_Y m_Sus_Fz_fl = NaN;               //from suspension
    real_Y m_Axl_Trq_fr = NaN;              //from driveline
    real_Y m_Brk_Prs_fr = NaN;              //from brake
    real_Y m_Tir_Fx_fr = NaN;               //from tire
    real_Y m_Tir_My_fr = NaN;               //from tire
    real_Y m_Tir_Fz_fr = NaN;               //from tire
    real_Y m_Sus_Fz_fr = NaN;               //from suspension
    real_Y m_Axl_Trq_rl = NaN;              //from drive line
    real_Y m_Brk_Prs_rl = NaN;              //from brake
    real_Y m_Tir_Fx_rl = NaN;               //from tire
    real_Y m_Tir_My_rl = NaN;               //from tire
    real_Y m_Tir_Fz_rl = NaN;               //from tire
    real_Y m_Sus_Fz_rl = NaN;               //from suspension
    real_Y m_Axl_Trq_rr = NaN;              //from driveline
    real_Y m_Brk_Prs_rr = NaN;              //from brake
    real_Y m_Tir_Fx_rr = NaN;               //from tire
    real_Y m_Tir_My_rr = NaN;               //from tire
    real_Y m_Tir_Fz_rr = NaN;               //from tire
    real_Y m_Sus_Fz_rr = NaN;               //from suspension
    //whl fm push
    real_Y m_Brk_Trq_fl = NaN;              //not consumed
    real_Y m_Brk_Trq_fr = NaN;              //not consumed
    real_Y m_Brk_Trq_rl = NaN;              //not consumed
    real_Y m_Brk_Trq_rr = NaN;              //not consumed
};

}//end of name space 
#endif //INTERFACE_WHEEL_4DISK_HPP
