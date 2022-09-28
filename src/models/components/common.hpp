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
#ifndef COMMON_HPP
#define COMMON_HPP
#define NMSPC Yile
#include <string>
#include <vector>
#include <math.h>
#include <limits>
#include <memory>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <filesystem>
#include <boost/property_tree/json_parser.hpp>

namespace NMSPC{
typedef std::vector< double > d_vec;
typedef std::vector< bool > b_vec;
typedef std::vector< d_vec > d_v_vec;

constexpr double inf = std::numeric_limits<double>::infinity();
constexpr double eps = std::numeric_limits<double>::epsilon();
constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double g = 9.81;
constexpr double pi = 2.0 * acos(0.0);

struct push_back_state_and_time {
	d_v_vec &m_states;
	d_vec &m_times;
	
	push_back_state_and_time(d_v_vec &states,d_vec &times) : m_states(states),m_times(times){}
	
	void operator()(const d_vec &x, double t) {
		m_states.push_back(x);
		m_times.push_back(t);
	}
};

extern double saturation(const double &in_data, const double &lower, const double &upper);
extern double dead_zone(const double &in_data, const double &lower, const double &upper);
extern double div0protect(const double &in_data, const double &thresh);
extern double div0protect_abs(const double &in_data, const double &thresh);
extern void printvec(const d_vec &in_data,const char &l, const char &r);

class piece_wise_linear {
public:
	piece_wise_linear(const d_vec &x, const d_vec &y):m_x(x),m_y(y) {
		m_n = m_x.size();
	}
	double operator() (const double &x_in) const;
private:
	d_vec m_x, m_y;
	int m_n;
};


}//end of name space 
#endif //COMMON_HPP
