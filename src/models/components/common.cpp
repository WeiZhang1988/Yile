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

double NMSPC::saturation(const double &in_data, const double &lower, const double &upper) {
    if (in_data > upper) 
        return upper;
    else if (in_data < lower)
        return lower;
    else
        return in_data;
}

double NMSPC::dead_zone(const double &in_data, const double &lower, const double &upper) {
	if (in_data > upper)
		return in_data - upper;
	else if (in_data < lower)
		return in_data - lower;
	else 
		return 0.0;
}

double NMSPC::div0protect(const double &in_data, const double &thresh) {
	if (in_data >= -thresh && in_data <= thresh) {
		if (in_data >= 0.0) {
			return 2.0 * thresh / (3 - pow(in_data / thresh, 2.0));
		} else {
			return -2.0 * thresh / (3 - pow(in_data / thresh, 2.0));
		}
	} else {
		return in_data;
	}
}

double NMSPC::div0protect_abs(const double &in_data, const double &thresh) {
	if (in_data >= -thresh && in_data <= thresh) {
		return 2.0 * thresh / (3 - pow(in_data / thresh, 2.0));
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

double NMSPC::piece_wise_linear::operator() (const double &x_in){
	if (x_in <= m_x[0]) {
		if (m_x[0]<m_x[1])
			return (x_in-m_x[1])/(m_x[0]-m_x[1])*m_y[0] + (x_in-m_x[0])/(m_x[1]-m_x[0])*m_y[1];
		else return NaN;
	} else if (x_in > m_x[m_n-1]) {
		if (m_x[m_n-2]<m_x[m_n-1])
			return (x_in-m_x[m_n-1])/(m_x[m_n-2]-m_x[m_n-1])*m_y[m_n-2] + (x_in-m_x[m_n-2])/(m_x[m_n-1]-m_x[m_n-2])*m_y[m_n-1];
		else return NaN;
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