#ifndef FSMDEBUG_H1
#define FSMDEBUG_H1

#include "FSMState.h"
#include "../common/Checkjoint.h"

class FSMState_Debug: public FSMState
{
    public:
        FSMState_Debug(ControlFSMData *data);
     
        void enter(){}
        void run();
        void exit() {}
        FSMStateName checkTransition()
        {
            return FSMStateName::DEBUG;
        }
    private:
        Checkjoint runobject;

  
    
};

#endif