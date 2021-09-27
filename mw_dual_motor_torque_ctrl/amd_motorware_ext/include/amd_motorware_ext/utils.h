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

#ifndef AMDMWEXT_UTILS_H_
#define AMDMWEXT_UTILS_H_

#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/est/src/32b/est.h"

#ifdef __cplusplus
extern "C" {
#endif


//! \brief Remove offset from position in an overflow safe way
//!
//! Remove the offset from the position and preserve the range of the result
//! (position is periodic and lies within [-max_pos, +max_pos].  The
//! computations are done such that integer overflows are avoided, i.e. this
//! also works if values are near to the max. value of the used IQ values.
//! Note that this is important as pos_max may go up to 100 while the max. for
//! IQ24 is 127!
//!
//! \param position The actual position in range [-pos_max, +pos_max]
//! \param offset The offset between actual and desired zero position
//! \param pos_max The maximum position.  When going beyond that value, the
//!                position rolls over to -max_pos.
//! \returns Position with offset removed.
_iq UTILS_removePositionOffset(_iq position, _iq offset, _iq pos_max);


//! \brief Computes Torque in Nm
//!
//! This is a modified version of the function from user.c which does not need a
//! CTRL_Handle.
_iq UTILS_computeTorque_Nm(EST_Handle estHandle, MATH_vec2 Idq_pu,
		const _iq torque_Flux_sf, const _iq torque_Ls_sf);


#ifdef __cplusplus
}
#endif // extern "C"

#endif  // ifndef AMDMWEXT_UTILS_H_
