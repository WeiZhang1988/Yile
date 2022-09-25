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
#ifndef SUBSYSTEM_TIRE_4FIALA_HPP
#define SUBSYSTEM_TIRE_4FIALA_HPP
#include "components/common.hpp"
#include "components/tire_fiala.hpp"
namespace NMSPC{
class Subsys_Tire_4Fiala {
public:
    static const int m_con_states_num = Tire_Fiala::m_con_states_num + Tire_Fiala::m_con_states_num + Tire_Fiala::m_con_states_num + Tire_Fiala::m_con_states_num;
	static const int m_derivatives_num = m_con_states_num;
	static const int m_dis_states_num = Tire_Fiala::m_dis_states_num + Tire_Fiala::m_dis_states_num + Tire_Fiala::m_dis_states_num + Tire_Fiala::m_dis_states_num;

    void add_tirs (std::shared_ptr<Tire_Fiala> sptr_tir_fl, std::shared_ptr<Tire_Fiala> sptr_tir_fr,\
    std::shared_ptr<Tire_Fiala> sptr_tir_rl, std::shared_ptr<Tire_Fiala> sptr_tir_rr) 
    {m_sptr_tir_fl=sptr_tir_fl; m_sptr_tir_fr=sptr_tir_fr;m_sptr_tir_rl=sptr_tir_rl; m_sptr_tir_rr=sptr_tir_rr;}

    void push_con_states (d_vec &con_states);
	void pull_con_states (const d_vec &con_states);

	void pull_pv (const double &Tir_omega_fl, const double &Tir_rhoz_fl, const double &Tir_Re_fl, const double &Sus_vx_fl, const double &Sus_vy_fl, const double &Sus_vz_fl,\
	const double &Sus_gamma_fl, const double &Sus_str_fl, const double &Sus_r_fl, \
    const double &Tir_omega_fr, const double &Tir_rhoz_fr, const double &Tir_Re_fr, const double &Sus_vx_fr, const double &Sus_vy_fr, const double &Sus_vz_fr,\
	const double &Sus_gamma_fr, const double &Sus_str_fr, const double &Sus_r_fr, \
    const double &Tir_omega_rl, const double &Tir_rhoz_rl, const double &Tir_Re_rl, const double &Sus_vx_rl, const double &Sus_vy_rl, const double &Sus_vz_rl,\
	const double &Sus_gamma_rl, const double &Sus_str_rl, const double &Sus_r_rl, \
    const double &Tir_omega_rr, const double &Tir_rhoz_rr, const double &Tir_Re_rr, const double &Sus_vx_rr, const double &Sus_vy_rr, const double &Sus_vz_rr,\
	const double &Sus_gamma_rr, const double &Sus_str_rr, const double &Sus_r_rr);
    void push_pv () {};
    void pull_fm (const double &Sus_Fz_fl, const double &Gnd_scale_fl, const double &Tir_Prs_fl, const double &Air_Tamb_fl, \
    const double &Sus_Fz_fr, const double &Gnd_scale_fr, const double &Tir_Prs_fr, const double &Air_Tamb_fr, \
    const double &Sus_Fz_rl, const double &Gnd_scale_rl, const double &Tir_Prs_rl, const double &Air_Tamb_rl, \
    const double &Sus_Fz_rr, const double &Gnd_scale_rr, const double &Tir_Prs_rr, const double &Air_Tamb_rr);
	void push_fm (double &Sus_TirFx_fl, double &Sus_TirFy_fl, double &Sus_TirFz_fl, double &Tir_Fx_fl, double &Tir_Fy_fl, double &Tir_Fz_fl, double &Tir_Mx_fl, double &Tir_My_fl, double &Tir_Mz_fl, \
    double &Sus_TirFx_fr, double &Sus_TirFy_fr, double &Sus_TirFz_fr, double &Tir_Fx_fr, double &Tir_Fy_fr, double &Tir_Fz_fr, double &Tir_Mx_fr, double &Tir_My_fr, double &Tir_Mz_fr, \
    double &Sus_TirFx_rl, double &Sus_TirFy_rl, double &Sus_TirFz_rl, double &Tir_Fx_rl, double &Tir_Fy_rl, double &Tir_Fz_rl, double &Tir_Mx_rl, double &Tir_My_rl, double &Tir_Mz_rl, \
    double &Sus_TirFx_rr, double &Sus_TirFy_rr, double &Sus_TirFz_rr, double &Tir_Fx_rr, double &Tir_Fy_rr, double &Tir_Fz_rr, double &Tir_Mx_rr, double &Tir_My_rr, double &Tir_Mz_rr);

	void push_drv (d_vec &derivatives);

    private:
    std::shared_ptr<Tire_Fiala> m_sptr_tir_fl, m_sptr_tir_fr, m_sptr_tir_rl, m_sptr_tir_rr;
    d_vec m_tir_con_states_fl = d_vec(Tire_Fiala::m_con_states_num, NaN);
    d_vec m_tir_con_states_fr = d_vec(Tire_Fiala::m_con_states_num, NaN);
    d_vec m_tir_con_states_rl = d_vec(Tire_Fiala::m_con_states_num, NaN);
    d_vec m_tir_con_states_rr = d_vec(Tire_Fiala::m_con_states_num, NaN);

    d_vec m_tir_drvs_fl = d_vec(Tire_Fiala::m_derivatives_num, NaN);
    d_vec m_tir_drvs_fr = d_vec(Tire_Fiala::m_derivatives_num, NaN);
    d_vec m_tir_drvs_rl = d_vec(Tire_Fiala::m_derivatives_num, NaN);
    d_vec m_tir_drvs_rr = d_vec(Tire_Fiala::m_derivatives_num, NaN);
};

} //end of name space
#endif //SUBSYSTEM_TIRE_4FIALA_HPP