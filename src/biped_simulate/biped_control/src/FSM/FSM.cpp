#include "../../include/FSM/FSM.h"
#include <iostream>
//#include "../../include/FSM/FSMState_Debug.h"

FSM::FSM(ControlFSMData *data)
    :_data(data)
{
     MPChandle=std::make_unique<ConvexMPCLocomotion>(0.001,40,data,true);
    _stateList.invalid = nullptr;
    _stateList.stand=nullptr;
    _stateList.debuging=nullptr;
    _stateList.passive = new FSMState_Passive(_data);
    _stateList.walking = new FSMState_Walking(_data,*MPChandle);
    _stateList.tstand=new FSMState_Tstand(_data,*MPChandle);
  //  _stateList.stand=new FSMState_PStand(_data);
   //_stateList.debuging=new FSMState_Debug(_data);
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize()
{
    count = 0;
    _currentState = _stateList.walking;
    _currentState -> enter();
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;
}

void FSM::run()
{
    _data->sendRecv();

    if(!checkSafty())
    {
        _data->_interface->setPassive();
    }

    if(_mode == FSMMode::NORMAL)
    {
        _currentState->run();
        _nextStateName = _currentState->checkTransition();
        if(_nextStateName != _currentState->_stateName)
        {
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
        }
    }
    else if(_mode == FSMMode::CHANGE)
    {
        // std::cout << "change state" << std::endl;
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();       
    }

    count++;
}

FSMState* FSM::getNextState(FSMStateName stateName)
{
    switch(stateName)
    {
        case FSMStateName::INVALID:
            return _stateList.invalid;
        break;
        case FSMStateName::PASSIVE:
            return _stateList.passive;
        break;
        case FSMStateName::WALKING:
            return _stateList.walking;
        case FSMStateName::DEBUG:
            return _stateList.debuging;
        break;
        default:
            return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty()
{
    std::cout<<"状态机的rbody值:\n"<<_data->_stateEstimator->getResult().rBody;
    if(_data->_stateEstimator->getResult().rBody(2,2) < 0.5)
    {
        return false;
    }
    else
    {
        return true;
    }
}