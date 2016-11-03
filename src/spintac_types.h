/**
 * \brief Definitions of types used for the SpinTAC modules
 *
 * Move this to a separate file to avoid circular includes in the current setup.
 */
/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, LineStream Technologies Incorporated
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
* *  Neither the names of Texas Instruments Incorporated, LineStream
 *    Technologies Incorporated, nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#ifndef SRC_SPINTAC_TYPES_H_
#define SRC_SPINTAC_TYPES_H_

#include "sw/modules/spintac/src/32b/spintac_pos_conv.h"

// **************************************************************************
// the typedefs

//! \brief Defines the velocity components of SpinTAC (ST)
//!
//! This structure is quite stupid with only one element. Keep it nonetheless so
//! that it remains compatible with the GUI, which expects the motor position in
//! st_obj.vel.conv.Pos_mrev
typedef struct _VEL_Params_t
{
    ST_PosConv_t conv;    //!< the position converter (ST_PosConv) object
} VEL_Params_t;


//! \brief Defines the SpinTAC (ST) object
typedef struct _ST_Obj
{
	//! \brief Contains the PosConv object. Exists for compatibility with GUI.
	VEL_Params_t vel;
	//! \brief The version (ST_Ver) object
	ST_Ver_t version;
    //! \brief Handle for Position Converter (ST_PosConv)
	ST_POSCONV_Handle posConvHandle;
    //! \brief Handle for Version (ST_Ver)
	ST_VER_Handle versionHandle;
} ST_Obj;


//! \brief SpinTAC Handle
typedef struct _ST_Obj_ *ST_Handle;


//! \brief Defines the SpinTAC (ST) global variables
typedef struct _ST_Vars_t
{
	//! \brief Displays the error seen by the Position Converter (ST_PosConv)
    uint16_t PosConvErrorID;
} ST_Vars_t;

#endif /* SRC_SPINTAC_TYPES_H_ */
