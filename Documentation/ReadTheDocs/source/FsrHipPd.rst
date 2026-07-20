FSR Hip PD Controller
=====================

Purpose
-------

``fsrHipPd`` is a hip joint-space impedance controller for an AK-series CAN
motor when no joint torque sensor is available. It uses the motor-derived joint
position and velocity as feedback and uses calibrated foot FSR events only to
estimate gait phase. The controller returns joint-side desired torque in Nm.

The implemented control law is

.. math::

   \tau = \tau_{ff}(\phi) + K_p(\theta_{ref}(\phi)-\theta) - K_d\omega_f

The positive ``Kp * (reference - position)`` form is intentional for negative
feedback in the project's joint coordinate convention. Motor direction is
handled by the existing ``hipFlipMotorDir`` configuration and must be verified
separately for each side on a restrained bench. No additional current PID loop
runs on the Teensy.

Reference and feed-forward profiles
-----------------------------------

The feed-forward profile is a periodic, piecewise-linear interpolation through
three gait-phase/torque nodes. ``ref0Deg`` is the relative equilibrium angle at
0/100 percent gait and ``ref50Deg`` is the relative equilibrium angle at 50
percent gait; a cosine blend makes the reference continuous. The controller
captures the first fresh joint angle after activation as the neutral position,
so neither reference uses the motor absolute zero as the human neutral pose.

Safety states
-------------

The startup path is ``DISABLED -> WAIT_FEEDBACK -> WAIT_GAIT -> RAMPING ->
ACTIVE``. A new, valid CAN response is required before neutral capture, and a
new ground strike after activation is required before ramping. Invalid
parameters, stale CAN feedback, stale/invalid gait phase, hard position or
velocity limits, consecutive over-current samples, or e-stop activation enter
``FAULT_LATCHED`` and immediately command zero torque. Controller and parameter
changes also command an immediate zero and restart the startup path.

An external trial-start request first clears motor-enable and command fields,
loads every used joint's SD preset, and enters ``trial_on`` only if all loads
succeed. The Teensy returns the resulting status to the communications MCU;
the communications MCU does not mark the start confirmed or send motor enable
and the one-time FSR calibration until the matching tokenized ACK is received.
Delayed ACKs from cancelled requests are ignored, and a Start received while a
Stop ACK is pending is queued behind that acknowledged off boundary. Repeated
confirmed Start requests may retry motor enable but do not restart FSR
calibration. An existing error status rejects startup.

Position limits are relative to the captured neutral angle. Soft position and
velocity limits reduce only commands that continue moving outward; commands
that brake or return toward neutral remain available. Current feedback is used
only for warning-to-trip derating and fault detection. It is not used as a
closed-loop torque measurement.

Fault reset requires a rising edge on ``faultRst`` while the trial is off,
``enable`` is zero, the motor is disabled, e-stop is clear, and all parameters
are valid. The fault disables feedback-producing motor transactions, so frozen
position/current samples are deliberately not accepted as recovery evidence.
After reset, a later activation must receive a new CAN frame before it can
capture a new neutral position. Switching to another controller does not clear
a latched fault or bypass its joint-level zero-torque interlock. A fault also
removes the motor hardware-enable output. An accepted reset clears the
controller latch but keeps power and CAN enable off; a later, separate motor
enable request restores power, queues an all-zero command plus disable, and
only then permits an enable request on the following cycle. Set ``faultRst``
back to zero after a reset attempt.

Parameters
----------

The SD metadata file is ``SDCard/hipControllers/fsrHipPd.csv``. Its shipped
parameter set is entirely zero, including ``enable=0``; it is intentionally not
an operational prescription. It is the only shipped preset (set 0). Missing,
out-of-range, truncated, nonnumeric, or non-finite preset rows are rejected
without committing the requested controller or replacing the previous
parameter array. If any used joint's default preset cannot be loaded when a
trial is requested, the firmware keeps ``trial_off``, clears command fields,
and disables the motors.

.. list-table::
   :header-rows: 1

   * - Index
     - CSV label
     - Meaning / unit
   * - 0
     - ``enable``
     - Controller enable, 0 or 1
   * - 1, 3, 5
     - ``ffPh1``, ``ffPh2``, ``ffPh3``
     - Strictly increasing gait phases, percent, less than 100
   * - 2, 4, 6
     - ``ffNm1``, ``ffNm2``, ``ffNm3``
     - Joint-side feed-forward node torque, Nm
   * - 7, 8
     - ``ref0Deg``, ``ref50Deg``
     - Relative equilibrium angles, degrees
   * - 9
     - ``kp``
     - Stiffness, Nm/rad
   * - 10
     - ``kd``
     - Damping, Nm/(rad/s)
   * - 11
     - ``velAlpha``
     - Velocity EWMA coefficient, greater than 0 and at most 1
   * - 12
     - ``trqLim``
     - Absolute joint torque limit, Nm
   * - 13
     - ``slewLim``
     - Maximum torque change rate, Nm/s
   * - 14, 15
     - ``softADeg``, ``hardADeg``
     - Relative soft/hard position limits, degrees
   * - 16, 17
     - ``softVel``, ``hardVel``
     - Soft/hard joint velocity limits, rad/s
   * - 18, 19
     - ``warnAmp``, ``tripAmp``
     - Current warning/trip thresholds; trip must not exceed the configured motor feedback range
   * - 20
     - ``tripCount``
     - Consecutive new feedback frames at/above trip threshold
   * - 21
     - ``fbToutMs``
     - CAN feedback timeout, ms; valid range 5 to 60000
   * - 22
     - ``gaitToMs``
     - Startup/ground-strike timeout, ms; valid range 1000 to 60000
   * - 23
     - ``rampMs``
     - Startup ramp duration, ms; valid range 1 to 60000
   * - 24
     - ``faultRst``
     - Safe fault-reset command, 0 or 1

Validity rules
--------------

``enable`` and ``faultRst`` must each be exactly zero or one. The three phase
nodes must satisfy ``0 <= ffPh1 < ffPh2 < ffPh3 < 100``. ``kp`` and ``kd``
must be non-negative, ``0 < velAlpha <= 1``, and both ``trqLim`` and
``slewLim`` must be positive. Every feed-forward node magnitude must be no
larger than ``trqLim``.

The angle and velocity limits must satisfy ``0 < soft < hard`` in their
respective units, and each reference-angle magnitude must be no larger than
``softADeg``. Current thresholds must satisfy ``0 < warnAmp < tripAmp`` and
``tripAmp`` must be within the selected motor's feedback encoding range;
``tripCount`` must be an integer from 1 through 1000. The timeout/ramp ranges
are listed in the table above. Because the shipped all-zero row is a safe,
disabled placeholder rather than a valid operating set, populate the complete
set while ``enable=0`` before attempting activation.

Commissioning constraints
-------------------------

Do not treat the protocol's 13.5 value or the configured 1.035 Nm/A constant as
a wearable safety limit. Confirm the exact AK60-6 V1.1 KV80 firmware field
meaning, current scaling, torque constant, feedback sign, and saturation with
independent instrumentation. Validate metadata and FSR phase first with
``NullMotor``/``zeroTorque``; then use a restrained, unoccupied bench to verify
e-stop polarity, motor direction, gearing, feedback freshness, and every hard
fault. Enter all limits and gains while ``enable=0`` and change ``enable`` to
one only after the complete set is valid. A freshly cleared phase estimator
needs three complete step intervals (four ground strikes); choose ``gaitToMs``
with margin for that startup window and the slowest intended cadence. For
powered tuning, introduce damping first, then stiffness, reference motion, and
finally FSR feed-forward, using conservative limits throughout.
