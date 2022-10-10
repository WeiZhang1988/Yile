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

    Subsys_Tire_4Fiala (double Lrelx_f=0.05, double Lrely_f=0.15, \
	double alpha_min_f=-1.50806269049027, double alpha_max_f=1.47424202250808, double mu_min_f=0.5, \
	double mu_max_f=0.8, double aMy_f=8e-4, double bMy_f=1e-3, \
	double cMy_f=1.6e-4, double alphaMy_f=-3e-3, double betaMy_f=0.97, \
	double Fz_min_f=100.0, double Fz_max_f=1e4, double cKappa_f=1e7, \
	double cAlpha_f=4.5e4, double bMz_f=0.0, double width_f=0.209045013245853, \
	double cGamma_f=0.0, \
	double init_kappa_f=0.0, double init_alpha_f=0.0, \
	double init_Mroll_f=0.0, \
    double Lrelx_r=0.15, double Lrely_r=0.01, \
	double alpha_min_r=-1.50806269049027, double alpha_max_r=1.47424202250808, double mu_min_r=0.8, \
	double mu_max_r=1.0, double aMy_r=8e-4, double bMy_r=1e-3, \
	double cMy_r=1.6e-4, double alphaMy_r=-3e-3, double betaMy_r=0.97, \
	double Fz_min_r=100.0, double Fz_max_r=1e4, double cKappa_r=1e7, \
	double cAlpha_r=4.5e4, double bMz_r=0.0, double width_r=0.209045013245853, \
	double cGamma_r=0.0, \
	double init_kappa_r=0.0, double init_alpha_r=0.0, \
	double init_Mroll_r=0.0) {
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