# NovaBase

`NovaMotor` in `wrappers/NovaBase.h` implements `MotorBase` using the Thrifty
Nova's internal encoder. The motor is stored directly as an object, just like
the REV wrappers.

```cpp
#include "wrappers/NovaBase.h"

NovaMotor intake{12}; // CAN ID 12, NEO, bus 0
NovaMotor shooter{13, thrifty::Motor::MINION, 0};

// Run during setup.
intake.SetNeutralMode(MotorBase::NeutralMode::Brake);
intake.SetCurrentLimit(40);
intake.SetConversionFactor(0.1); // 10 motor rotations per output rotation

// Run from your subsystem or command.
intake.Set(0.25);
intake.Stop();
```

## Dependency and build

Written against **ThriftyLib 2027.0.0-alpha-3** and compile-checked against the
actual vendor headers with **WPILib 2027.0.0-alpha-5** headers.

The current project still uses WPILib 2026 and its Java-only ThriftyLib vendordep.
`NovaBase.cpp` is excluded by an include guard while the C++ Nova header is
unavailable. It becomes active automatically when the C++ dependency is installed.
Including `NovaBase.h` in robot code requires that dependency; there is no dummy
motor implementation.

For a SystemCore project with the matching WPILib release, install the official
[C++ vendor dependency](https://software.thethriftybot.com/frcvendor/ThriftyLib-2027.json)
using WPILib's vendor library manager, replacing the old ThriftyLib dependency.
The beta manifest currently supports SystemCore and desktop, but not roboRIO.
For this roboRIO project, install the compatible C++ release when it becomes
available, then rebuild before using `NovaMotor`. Verify the API against that
release; future compatibility has not been tested.

`MotorBase.h` accepts either the 2026 unit headers or the new SystemCore unit
headers. This does not migrate the rest of the robot project to WPILib 2027.

## Differences from REVBase

- Position commands/readings and soft limits use **output degrees**; velocity uses
  **output RPM**. Conversion factors are output rotations per motor rotation.
- Nova supports **PID slots 0 and 1**. Commands without a slot use slot 0.
  Tune PIDF in Nova's native motor rotations and rotations/second; do not copy
  REV gains unchanged. Configure nonzero gains before using closed-loop control.
- Nova has **one enable switch for both soft limits**. Either enable method
  changes both directions. Set both boundaries before enabling them.
  `ApplyConfiguration` rejects unequal forward/reverse enable flags with
  `std::invalid_argument`; it also rejects invalid conversion, current, or
  enabled voltage-compensation values before applying settings.
- `SetWrapping` uses the current internal-encoder position to select the nearest
  equivalent target angle each time `SetPosition` is called. Call it each tick
  when tracking a wrapped target. This is not Nova's onboard absolute-encoder
  wrapping, and position readings remain continuous.
- Current-limit methods configure **stator current**. Disabling the custom limit
  restores the documented 40 A default; it does not remove current protection.
  Supply-current protection is left at its existing controller setting.
- Configuration setters send their changes immediately; use them during setup.
  Configuration and CAN errors are not exposed by the existing `MotorBase` API.
- `SimulationPeriodic` is an empty hook. No mechanism physics model is included.

## Validation

The Nova implementation passes a C++20 syntax check using the genuine ThriftyLib
and WPILib SystemCore headers, with warnings treated as errors. The normal 2026
robot build skips Nova until the compatible C++ dependency is installed. Nova
linking, CAN communication, and physical motor behavior still require validation
on the target controller.

References: [ThriftyLib setup](https://docs.thethriftybot.com/software/thriftylib/latest/overview),
[control types and units](https://docs.thethriftybot.com/software/thriftylib/latest/control-types),
[configuration](https://docs.thethriftybot.com/software/thriftylib/latest/configs).

## Swerve steering

`SwerveModule` now uses `SparkFlexMotor` for drive and `NovaMotor` for steering.
The Nova motor profile is MINION, matching the former steering motor selection.
`SetAbsolutePosition` uses voltage-based PID and accepts absolute sensor degrees;
`getAbsolutePosition` returns sensor degrees, and `SetAbsoluteWrapping` enables
onboard shortest-path control. These absolute methods do not apply the internal
motor encoder conversion factors. Select and calibrate the actual connected
absolute encoder in Thrifty Config before running steering; software inversion
in the module only changes coordinates, not the controller's feedback polarity.
Steering gains must be checked and tuned on the Nova hardware.

The swerve subsystem now includes `NovaBase.h`, so building the complete robot
requires the compatible ThriftyLib C++ release. The missing-header guard in
`NovaBase.cpp` does not bypass that requirement.
