// BSD 3-Clause License
//
// Copyright (c) 2019, Max Planck Gesellschaft, New York University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

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
