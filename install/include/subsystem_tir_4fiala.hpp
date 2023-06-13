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

    Subsys_Tire_4Fiala (real_Y Lrelx_f=0.05, real_Y Lrely_f=0.15, \
	real_Y alpha_min_f=-1.50806269049027, real_Y alpha_max_f=1.47424202250808, real_Y mu_min_f=0.5, \
	real_Y mu_max_f=0.8, real_Y aMy_f=8e-4, real_Y bMy_f=1e-3, \
	real_Y cMy_f=1.6e-4, real_Y alphaMy_f=-3e-3, real_Y betaMy_f=0.97, \
	real_Y Fz_min_f=100.0, real_Y Fz_max_f=1e4, real_Y cKappa_f=1e7, \
	real_Y cAlpha_f=4.5e4, real_Y bMz_f=0.0, real_Y width_f=0.209045013245853, \
	real_Y cGamma_f=0.0, \
	real_Y init_kappa_f=0.0, real_Y init_alpha_f=0.0, \
	real_Y init_Mroll_f=0.0, \
    real_Y Lrelx_r=0.15, real_Y Lrely_r=0.01, \
	real_Y alpha_min_r=-1.50806269049027, real_Y alpha_max_r=1.47424202250808, real_Y mu_min_r=0.8, \
	real_Y mu_max_r=1.0, real_Y aMy_r=8e-4, real_Y bMy_r=1e-3, \
	real_Y cMy_r=1.6e-4, real_Y alphaMy_r=-3e-3, real_Y betaMy_r=0.97, \
	real_Y Fz_min_r=100.0, real_Y Fz_max_r=1e4, real_Y cKappa_r=1e7, \
	real_Y cAlpha_r=4.5e4, real_Y bMz_r=0.0, real_Y width_r=0.209045013245853, \
	real_Y cGamma_r=0.0, \
	real_Y init_kappa_r=0.0, real_Y init_alpha_r=0.0, \
	real_Y init_Mroll_r=0.0) {
        m_tir_fl = Tire_Fiala(Lrelx_f, Lrely_f, \
        alpha_min_f, alpha_max_f, mu_min_f, mu_max_f, \
        aMy_f, bMy_f, cMy_f, alphaMy_f, betaMy_f, Fz_min_f, Fz_max_f, \
        cKappa_f, cAlpha_f, bMz_f, width_f, cGamma_f, \
        init_kappa_f, init_alpha_f, init_Mroll_f);
        m_tir_fr = Tire_Fiala(Lrelx_f, Lrely_f, \
        alpha_min_f, alpha_max_f, mu_min_f, mu_max_f, \
        aMy_f, bMy_f, cMy_f, alphaMy_f, betaMy_f, Fz_min_f, Fz_max_f, \
        cKappa_f, cAlpha_f, bMz_f, width_f, cGamma_f, \
        init_kappa_f, init_alpha_f, init_Mroll_f);
        m_tir_rl = Tire_Fiala(Lrelx_r, Lrely_r, \
        alpha_min_r, alpha_max_r, mu_min_r, mu_max_r, \
        aMy_r, bMy_r, cMy_r, alphaMy_r, betaMy_r, Fz_min_r, Fz_max_r, \
        cKappa_r, cAlpha_r, bMz_r, width_r, cGamma_r, \
        init_kappa_r, init_alpha_r, init_Mroll_r);
        m_tir_rr = Tire_Fiala(Lrelx_r, Lrely_r, \
        alpha_min_r, alpha_max_r, mu_min_r, mu_max_r, \
        aMy_r, bMy_r, cMy_r, alphaMy_r, betaMy_r, Fz_min_r, Fz_max_r, \
        cKappa_r, cAlpha_r, bMz_r, width_r, cGamma_r, \
        init_kappa_r, init_alpha_r, init_Mroll_r);
    };

    void push_con_states (d_vec &con_states);
	void pull_con_states (const d_vec &con_states);

	void pull_pv (const real_Y &Tir_omega_fl, const real_Y &Tir_rhoz_fl, const real_Y &Tir_Re_fl, const real_Y &Sus_vx_fl, const real_Y &Sus_vy_fl, const real_Y &Sus_vz_fl,\
	const real_Y &Sus_gamma_fl, const real_Y &Sus_str_fl, const real_Y &Sus_r_fl, \
    const real_Y &Tir_omega_fr, const real_Y &Tir_rhoz_fr, const real_Y &Tir_Re_fr, const real_Y &Sus_vx_fr, const real_Y &Sus_vy_fr, const real_Y &Sus_vz_fr,\
	const real_Y &Sus_gamma_fr, const real_Y &Sus_str_fr, const real_Y &Sus_r_fr, \
    const real_Y &Tir_omega_rl, const real_Y &Tir_rhoz_rl, const real_Y &Tir_Re_rl, const real_Y &Sus_vx_rl, const real_Y &Sus_vy_rl, const real_Y &Sus_vz_rl,\
	const real_Y &Sus_gamma_rl, const real_Y &Sus_str_rl, const real_Y &Sus_r_rl, \
    const real_Y &Tir_omega_rr, const real_Y &Tir_rhoz_rr, const real_Y &Tir_Re_rr, const real_Y &Sus_vx_rr, const real_Y &Sus_vy_rr, const real_Y &Sus_vz_rr,\
	const real_Y &Sus_gamma_rr, const real_Y &Sus_str_rr, const real_Y &Sus_r_rr);
    void push_pv () {};
    void pull_fm (const real_Y &Sus_Fz_fl, const real_Y &Gnd_scale_fl, const real_Y &Tir_Prs_fl, const real_Y &Air_Tamb_fl, \
    const real_Y &Sus_Fz_fr, const real_Y &Gnd_scale_fr, const real_Y &Tir_Prs_fr, const real_Y &Air_Tamb_fr, \
    const real_Y &Sus_Fz_rl, const real_Y &Gnd_scale_rl, const real_Y &Tir_Prs_rl, const real_Y &Air_Tamb_rl, \
    const real_Y &Sus_Fz_rr, const real_Y &Gnd_scale_rr, const real_Y &Tir_Prs_rr, const real_Y &Air_Tamb_rr);
	void push_fm (real_Y &Sus_TirFx_fl, real_Y &Sus_TirFy_fl, real_Y &Sus_TirFz_fl, real_Y &Tir_Fx_fl, real_Y &Tir_Fy_fl, real_Y &Tir_Fz_fl, real_Y &Tir_Mx_fl, real_Y &Tir_My_fl, real_Y &Tir_Mz_fl, \
    real_Y &Sus_TirFx_fr, real_Y &Sus_TirFy_fr, real_Y &Sus_TirFz_fr, real_Y &Tir_Fx_fr, real_Y &Tir_Fy_fr, real_Y &Tir_Fz_fr, real_Y &Tir_Mx_fr, real_Y &Tir_My_fr, real_Y &Tir_Mz_fr, \
    real_Y &Sus_TirFx_rl, real_Y &Sus_TirFy_rl, real_Y &Sus_TirFz_rl, real_Y &Tir_Fx_rl, real_Y &Tir_Fy_rl, real_Y &Tir_Fz_rl, real_Y &Tir_Mx_rl, real_Y &Tir_My_rl, real_Y &Tir_Mz_rl, \
    real_Y &Sus_TirFx_rr, real_Y &Sus_TirFy_rr, real_Y &Sus_TirFz_rr, real_Y &Tir_Fx_rr, real_Y &Tir_Fy_rr, real_Y &Tir_Fz_rr, real_Y &Tir_Mx_rr, real_Y &Tir_My_rr, real_Y &Tir_Mz_rr);

	void push_drv (d_vec &derivatives);

    private:
    Tire_Fiala m_tir_fl, m_tir_fr, m_tir_rl, m_tir_rr;
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