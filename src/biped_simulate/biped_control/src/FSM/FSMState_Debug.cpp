#include "../../include/FSM/FSMState_Debug.h"

FSMState_Debug::FSMState_Debug(ControlFSMData *data):FSMState(data, FSMStateName::DEBUG, "debuging"),
runobject(data->_interface,data->_legController,data->_lowCmd,data->_lowState,data->_stateEstimator)
{
}

void FSMState_Debug::run()
{
    _data->_legController->updateData(_data->_lowState);
    _data->_stateEstimator->run(); 
    
    runobject.checkgait1();
   // runobject.staystill();

    _data->_legController->updateCommand(_data->_lowCmd); 

    cout<<"\n------------------------------\n";

}

