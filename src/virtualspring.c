/*
 * virtualspring.c
 *
 *  Created on: 14 Jul 2016
 *      Author: fwidmaier
 */
// **************************************************************************
// the includes
#include "virtualspring.h"
#include <amd_motorware_ext/utils.h>


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions


VIRTUALSPRING_Handle VIRTUALSPRING_init(void *pMemory, const size_t numBytes)
{
    VIRTUALSPRING_Handle vsHandle;

    if(numBytes < sizeof(VIRTUALSPRING_Obj))
        return((VIRTUALSPRING_Handle) NULL);

    // assign the handle
    vsHandle = (VIRTUALSPRING_Handle) pMemory;

    return(vsHandle);
}


void VIRTUALSPRING_setup(VIRTUALSPRING_Handle vsHandle,
        int16_t spring_stiffness, _iq maxIqRef_A, _iq maxPosition_mrev)
{
    VIRTUALSPRING_Obj *obj = (VIRTUALSPRING_Obj *) vsHandle;

    obj->enabled = false;
    obj->flagResetOffset = false;
    obj->encoderOffset = 0;
    obj->equilibriumPosition = 0;
    obj->deflection = 0;
    obj->stiffness = spring_stiffness;
    obj->maxCurrentRef = maxIqRef_A;
    obj->maxPosition_mrev = maxPosition_mrev;

    return;
}


void VIRTUALSPRING_run(VIRTUALSPRING_Handle vsHandle, _iq motorPosition_mrev)
{
    VIRTUALSPRING_Obj *spring = (VIRTUALSPRING_Obj *) vsHandle;

    // First check if the spring is enabled.
    if (!spring->enabled)
    	return;

    // reset positon offset if flag is set
    if (spring->flagResetOffset) {
        spring->encoderOffset = motorPosition_mrev;
        spring->equilibriumPosition = 0;
        spring->flagResetOffset = false;
    }

    // Subtract offset from position to get deflection. Handle overflow of
    // Pos_mrev.
    //spring->deflection = motorPosition_mrev
    //        - spring->encoderOffset
    //        + spring->equilibriumPosition;
    spring->deflection = UTILS_removePositionOffset(motorPosition_mrev,
    		spring->encoderOffset,
			spring->maxPosition_mrev);
    // this offset has to be added, so remove negative
    spring->deflection = UTILS_removePositionOffset(spring->deflection,
    		-spring->equilibriumPosition,
			spring->maxPosition_mrev);

    return;
}


_iq VIRTUALSPRING_getIqRef_A(VIRTUALSPRING_Handle vsHandle)
{
    VIRTUALSPRING_Obj *spring = (VIRTUALSPRING_Obj *) vsHandle;
    _iq IqRef_A;

    IqRef_A = -spring->stiffness * spring->deflection;
    // cap current to max value
    IqRef_A = _IQsat(IqRef_A, spring->maxCurrentRef, -spring->maxCurrentRef);

    return IqRef_A;
}
