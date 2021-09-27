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

#include <amd_motorware_ext/utils.h>

_iq UTILS_removePositionOffset(_iq position, _iq offset, _iq pos_max)
{
	// Remove offset in a way that preserves the range and does not exceed the
	// max. value in subterms of the calculations (which would cause integer
	// overflows and make it explode).
	if (offset < 0)
	{
		_iq off_p_max = offset + pos_max;
		if (position < off_p_max) {
			position -= offset;
		} else {
			position = (position - pos_max) - off_p_max;
		}
	}
	else if (offset > 0)
	{
		_iq off_m_max = offset - pos_max;
		if (position > off_m_max) {
			position -= offset;
		} else {
			position = (position + pos_max) - off_m_max;
		}
	}

	/*
	 * The above code does the same thing than this one, but avoids overflows by
	 * ensuring that subterms never get greater than max_pos.
	 *
		position = positon - offset
		if (position > mrev_max) {
			position -= 2 * mrev_max;
		} else if (position < -mrev_max) {
			position += 2 * mrev_max;
		}
	*/

	return position;
}


_iq UTILS_computeTorque_Nm(EST_Handle estHandle, MATH_vec2 Idq_pu,
		const _iq torque_Flux_sf, const _iq torque_Ls_sf)
{
  _iq Flux_pu = EST_getFlux_pu(estHandle);
  _iq Id_pu = Idq_pu.value[0];
  _iq Iq_pu = Idq_pu.value[1];
  _iq Ld_minus_Lq_pu = _IQ30toIQ(
		  EST_getLs_d_pu(estHandle) - EST_getLs_q_pu(estHandle));
  _iq Torque_Flux_Iq_Nm = _IQmpy(
		  _IQmpy(Flux_pu, Iq_pu),
		  torque_Flux_sf);
  _iq Torque_Ls_Id_Iq_Nm = _IQmpy(
		  _IQmpy(
				  _IQmpy(Ld_minus_Lq_pu, Id_pu),
				  Iq_pu),
		  torque_Ls_sf);
  _iq Torque_Nm = Torque_Flux_Iq_Nm + Torque_Ls_Id_Iq_Nm;

  return(Torque_Nm);
} // end of USER_computeTorque_Nm() function
