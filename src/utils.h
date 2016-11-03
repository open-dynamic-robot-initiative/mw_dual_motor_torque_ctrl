/*
 * utils.h
 *
 *  Created on: Nov 3, 2016
 *      Author: fwidmaierlocal
 */

#ifndef SRC_UTILS_H_
#define SRC_UTILS_H_


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
static inline _iq removePositionOffset(_iq position, _iq offset, _iq pos_max)
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


#endif /* SRC_UTILS_H_ */
