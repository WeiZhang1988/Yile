// =============================================================================
// PROJECT YILE - 
//
// Copyright (c) 2023
// All rights reserved.
//
// Use of this source code is governed by a GPL-3.0 license that can be found
// in the LICENSE file
//
// Authors of this file	Wei ZHANG wei_zhang_1988@outlook.com
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

//typedef _Float64x real_Y;
typedef double real_Y;
namespace NMSPC{
typedef std::vector< real_Y > d_vec;
typedef std::vector< bool >   b_vec;
typedef std::vector< d_vec >  d_v_vec;

constexpr real_Y inf = std::numeric_limits<real_Y>::infinity();
constexpr real_Y eps = std::numeric_limits<real_Y>::epsilon();
constexpr real_Y NaN = std::numeric_limits<real_Y>::quiet_NaN();
constexpr real_Y g = 9.81;
constexpr real_Y pi = 2.0 * acos(0.0);
struct push_back_state_and_time {
	d_v_vec &m_states;
	d_vec &m_times;
	
	push_back_state_and_time(d_v_vec &states,d_vec &times) : m_states(states),m_times(times){}
	
	void operator()(const d_vec &x, real_Y t) {
		m_states.push_back(x);
		m_times.push_back(t);
	}
};
extern real_Y saturation(const real_Y &in_data, const real_Y &lower, const real_Y &upper);
extern real_Y dead_zone(const real_Y &in_data, const real_Y &lower, const real_Y &upper);
extern real_Y div0protect(const real_Y &in_data, const real_Y &thresh);
extern real_Y div0protect_abs(const real_Y &in_data, const real_Y &thresh);
extern void printvec(const d_vec &in_data,const char &l, const char &r);

class piece_wise_linear {
public:
	piece_wise_linear(const d_vec &x, const d_vec &y):m_x(x),m_y(y) {
		m_n = m_x.size();
	}
	real_Y operator() (const real_Y &x_in) const;
private:
	d_vec m_x, m_y;
	int m_n;
};


}//end of name space 
#endif //COMMON_HPP
