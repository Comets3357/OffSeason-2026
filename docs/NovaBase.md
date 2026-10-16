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
intake.ApplyConfiguration(); // Reapply the internally stored settings.

// Run from your subsystem or command.
intake.Set(0.25);
intake.Stop();
```

## Dependency and build

This project uses **ThriftyLib 2026.3.0** (including its C++ dependency) and
**WPILib 2026.2.1**. `NovaBase.cpp` builds as part of the normal robot program.
Run `./gradlew build --offline` when the dependencies are already cached.

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
  `ApplyConfiguration()` reapplies the stored configuration without an argument;
  `ApplyConfiguration(config)` replaces it with a supplied `MotorConfig`.
  Configuration and CAN errors are not exposed by the existing `MotorBase` API.
- `SimulationPeriodic` is an empty hook. No mechanism physics model is included.

## Validation

The wrapper is checked through the normal debug and release robot builds.
CAN communication, encoder direction, limits, and PID gains still require
validation on the target controller. Existing shooter gain values were retained
during migration; their response is not equivalent to REV gains.

References: [ThriftyLib setup](https://docs.thethriftybot.com/software/thriftylib/latest/overview),
[control types and units](https://docs.thethriftybot.com/software/thriftylib/latest/control-types),
[configuration](https://docs.thethriftybot.com/software/thriftylib/latest/configs).

## Swerve steering

`SwerveModule` now uses `SparkFlexMotor` for drive and `NovaMotor` for steering.
The Nova motor profile is MINION, matching the former steering motor selection.
`SetAbsolutePosition` uses voltage-based PID and accepts output degrees;
`getAbsolutePosition` returns output degrees, and `SetAbsoluteWrapping` enables
onboard shortest-path control. These absolute methods do not apply the internal
motor encoder conversion factors. `SetAbsolutePosition` automatically selects
absolute feedback; `SetPosition` and `SetVelocity` select internal feedback.
The private `SelectFeedback` helper only sends a feedback change when the source
differs, so subsystems do not need to call it.
Select and calibrate the actual connected
absolute encoder in Thrifty Config before running steering; software inversion
in the module only changes coordinates, not the controller's feedback polarity.
Steering gains must be checked and tuned on the Nova hardware.

## Intake, Index, and Shooter

All active motors in these three subsystems use Nova controllers with the NEO
profile. Their constructors apply settings using `ApplyConfiguration`.
The static shooter's flywheel leader is CAN ID 31, with 32 and 33 following it
without inversion. All three use a 60 A stator limit and disabled voltage
compensation. Holding the driver's left trigger commands 50% duty cycle;
release or command interruption stops the leader and its followers.
Pitch is CAN ID 41, with its existing configuration and methods preserved.
There is no yaw motor or yaw motor control.

`SetAbsoluteConversionFactor` takes output rotations per absolute encoder
rotation, independently of `SetConversionFactor` for the internal encoder.
Its default is 1.0, preserving the swerve's sensor-degree behavior.
Shooter pitch uses (1/20)*(15/9). The static shooter has no yaw encoder.

`SetSoftLimitSource(ABS)` makes limits use the absolute encoder and its conversion
factor. Set both limits before enabling them. Soft-limit source is independent
of the PID feedback source. `SetAbsoluteEncoderInverted(true)` selects Nova's
counter-clockwise absolute direction; verify that direction on the hardware.

`SetVoltageCompensation` updates the stored voltage setting. `SetFollow` enables
native CAN following; the index follower follows CAN ID 22 with inversion.
`GetOutputCurrent` reports stator amps and `GetAppliedVoltage` reports volts.

## External CAN absolute encoder

`SetExternalEncoder(encoderId)` pairs a **Thrifty CAN Encoder** on the Nova's bus.
It accepts IDs 0-62. Pass `thrifty::Motor::kNoCanEncoder` to unpair and return to
the wired absolute encoder. The pairing is retained by `ApplyConfiguration()`.

```cpp
//Example only: replace 5 with the encoder's actual CAN ID.
fuelDeploy.SetExternalEncoder(5);
fuelDeploy.SetAbsoluteConversionFactor(18.0 / 28.0);
fuelDeploy.SetSoftLimitSource(thrifty::Motor::FeedbackSensorType::ABS);
fuelDeploy.ApplyConfiguration();

fuelDeploy.SetAbsolutePosition(90_deg);
double angle = fuelDeploy.getAbsolutePosition().value();
```

Absolute position commands select `CAN_ABS` when paired. Absolute position and
velocity getters read the CAN encoder directly, with the existing conversion
factor. `SetPosition` and `SetVelocity` continue to use the internal encoder.
Soft limits configured with `ABS` automatically use `CAN_ABS` while paired.
`SetAbsoluteEncoderInverted` configures the selected encoder's direction, even
if called before pairing; this changes the CAN encoder itself and therefore
also affects other devices using that encoder. Configure its zero offset in
Thrifty Config. Motor-following mode and external encoder control are kept
mutually exclusive by the wrapper.

This does not pair arbitrary third-party CAN encoders or assign IDs to PWM
encoders. Thrifty's documentation specifies Nova firmware **2027.0.0-beta.4 or
newer** with `CAN_ENCODER` support. Unsupported firmware can reject the settings
and leave the motor on internal feedback; a successful build does not confirm
hardware support. No subsystem encoder IDs are assigned automatically.

Source: [Thrifty Encoders & Feedback](https://docs.thethriftybot.com/software/thriftylib/latest/encoders#can-encoder-feedback),
checked against the installed ThriftyLib 2026.3.0 C++ headers.
