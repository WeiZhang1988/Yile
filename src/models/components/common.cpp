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
#include "common.hpp"

real_Y NMSPC::saturation(const real_Y &in_data, const real_Y &lower, const real_Y &upper) {
    if (in_data > upper) 
        return upper;
    else if (in_data < lower)
        return lower;
    else
        return in_data;
}

real_Y NMSPC::dead_zone(const real_Y &in_data, const real_Y &lower, const real_Y &upper) {
	if (in_data > upper)
		return in_data - upper;
	else if (in_data < lower)
		return in_data - lower;
	else 
		return 0.0;
}

real_Y NMSPC::div0protect(const real_Y &in_data, const real_Y &thresh) {
	if (in_data >= -thresh && in_data <= thresh) {
		if (in_data >= 0.0) {
			return 2.0 * thresh / (3.0 - pow(in_data / thresh, 2.0));
		} else {
			return -2.0 * thresh / (3.0 - pow(in_data / thresh, 2.0));
		}
	} else {
		return in_data;
	}
}

real_Y NMSPC::div0protect_abs(const real_Y &in_data, const real_Y &thresh) {
	if (in_data >= -thresh && in_data <= thresh) {
		return 2.0 * thresh / (3.0 - pow(in_data / thresh, 2.0));
	} else {
		return abs(in_data);
	}
}

void NMSPC::printvec(const d_vec &in_data, const char &l, const char &r) {
	for (auto item : in_data) {
		std::cout<<l<<item<<r;
	}
	std::cout<<std::endl;
}

real_Y NMSPC::piece_wise_linear::operator() (const real_Y &x_in)  const{
	assert(m_n == m_y.size() && m_n>1);
	if (x_in <= m_x[0]) {
		return m_y[0];
	} else if (x_in > m_x[m_n-1]) {
		return m_y[m_n-1];
	} else {
		for (int i=0;i<m_n-1;i++) {
			if (x_in>m_x[i] && x_in<=m_x[i+1]) {
				if (m_x[i]<m_x[i+1])
					return (x_in-m_x[i+1])/(m_x[i]-m_x[i+1])*m_y[i] + (x_in-m_x[i])/(m_x[i+1]-m_x[i])*m_y[i+1];
				else return NaN;
			}
		}
		return NaN;
	}
}