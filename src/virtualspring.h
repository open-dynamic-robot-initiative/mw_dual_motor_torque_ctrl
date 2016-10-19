/**
 * Virtual Spring Module
 *
 * Adds objects/functions used for the virtual spring mode (torque control
 * based on motor deflection from zero position).
 *
 * Created on: 14 Jul 2016
 * \author Felix Widmaier
 */
#ifndef SENSORED_TORQUE_CTRL_VIRTUALSPRING_H_
#define SENSORED_TORQUE_CTRL_VIRTUALSPRING_H_

// **************************************************************************
// the includes
#include "sw/modules/types/src/types.h"
#include "sw/modules/iqmath/src/32b/IQmathLib.h"


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines


// **************************************************************************
// the typedefs


//! The virtual spring object
typedef struct _VIRTUALSPRING_Obj_ {
    //! Set true to activate virtual spring mode instead of direct current control.
    bool enabled;

    //! When set true, position offset of spring is reset on next iteration.
    bool flagResetOffset;

    //! Constant that influences the stiffness of the virtual spring mode.
    int16_t stiffness;

    //! Maximum current reference used in the virtual spring mode.
    _iq maxCurrentRef;

    //! Maximum position (Pos_mrev) of the encoder (needed to handle overflows)
    _iq maxPosition_mrev;

    //! Offset between encoder position and spring position.
    _iq encoderOffset;

    //! Equilibrium Position of the spring (range: 0.0 to 1.0).
    //! This is an additional offset of at most one revolution that is added to
    //! the Encoder offset.
    _iq equilibriumPosition;

    //! Current deflection of the spring from its zero position.
    _iq deflection;
} VIRTUALSPRING_Obj;

//! \brief Defines the CTRL handle
//!
typedef struct _VIRTUALSPRING_Obj_ *VIRTUALSPRING_Handle;

// **************************************************************************
// the function prototypes

//! Initialize the handle.
extern VIRTUALSPRING_Handle VIRTUALSPRING_init(
        void *pMemory, const size_t numBytes);

//! Setup the object with default values
// FIXME: describe parameters
extern void VIRTUALSPRING_setup(VIRTUALSPRING_Handle vsHandle,
        int16_t spring_stiffness, _iq maxIqRef_A, _iq maxPosition_mrev);

//! Compute the deflection of the spring.
extern void VIRTUALSPRING_run(VIRTUALSPRING_Handle vsHandle,
        _iq motorPosition_mrev);

//! Get the Iq reference value based on spring deflection.
extern _iq VIRTUALSPRING_getIqRef_A(VIRTUALSPRING_Handle vsHandle);

//! \brief Enable or disable the virtual spring.
//! \param vsHandle Virtual spring handle
//! \param enabled Set true to enable, false to disable.
//! \returns True if the state changed (i.e. was previously disabled and is now
//!          enabled or vice-versa), false if not.
inline bool VIRTUALSPRING_setEnabled(VIRTUALSPRING_Handle vsHandle,
		bool enabled)
{
    VIRTUALSPRING_Obj *spring = (VIRTUALSPRING_Obj *) vsHandle;

    bool changed = (spring->enabled != enabled);
    spring->enabled = enabled;

    return changed;
}

//! Check if virtual spring mode is enabled.
inline bool VIRTUALSPRING_isEnabled(VIRTUALSPRING_Handle vsHandle)
{
    VIRTUALSPRING_Obj *spring = (VIRTUALSPRING_Obj *) vsHandle;

    return spring->enabled;
}

//! Schedule reset of position offset (will be executed during next call of
//! VIRTUALSPRING_run())
inline void VIRTUALSPRING_scheduleResetOffset(VIRTUALSPRING_Handle vsHandle)
{
    VIRTUALSPRING_Obj *spring = (VIRTUALSPRING_Obj *) vsHandle;

    spring->flagResetOffset = true;
}


#ifdef __cplusplus
}
#endif // extern "C"

#endif /* SENSORED_TORQUE_CTRL_VIRTUALSPRING_H_ */
