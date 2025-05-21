#ifndef FSM_H
#define FSM_H

#include "FSMState.h"
#include "FSMState_Passive.h"
#include "FSMState_Walking.h"
#include "FSMState_Pstand.h"
#include "FSMState_Tstand.h"
#include "../common/enumClass.h"
#include "FSMState_Debug.h"
#include "../../ConvexMPC/ConvexMPCLocomotion.h"

struct FSMStateList{
    FSMState *invalid;
    FSMState_Passive *passive;
    FSMState_Walking *walking;
    FSMState_Debug *debuging;
    FSMState_PStand *stand;
    FSMState_Tstand *tstand;

    void deletePtr(){
        delete invalid;
        delete passive;
        delete walking;
        delete debuging;
        delete stand;
        delete tstand;
    }  
    
};

class FSM{
    public:
        FSM(ControlFSMData *data);
        ~FSM();
        void initialize();
        void run();
    private:
        FSMState* getNextState(FSMStateName stateName);
        bool checkSafty();
        ControlFSMData *_data;
        FSMState *_currentState;
        FSMState *_nextState;
        FSMStateName _nextStateName;
        FSMStateList _stateList;
        FSMMode _mode;
        std::unique_ptr<ConvexMPCLocomotion> MPChandle;
        long long _startTime;
        int count;
};

#endif