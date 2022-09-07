#ifndef SYSTEM_CONTINUOUS_STATES_HPP
#define SYSTEM_CONTINUOUS_STATES_HPP
#include "common.hpp"

class System_Con_States {
	public:
		System_States() : m_states_num(0), m_accs_num(0) {};
        ~System_States();
      
        d_vec m_states;
        d_vec m_accs;
        
        int m_states_num;
        int m_accs_num

};


#endif //SYSTEM_CONTINUOUS_STATES_HPP
