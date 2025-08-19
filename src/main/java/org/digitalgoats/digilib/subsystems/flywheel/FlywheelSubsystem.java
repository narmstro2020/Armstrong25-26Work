package org.digitalgoats.digilib.subsystems.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.*;

@Logged
public abstract class FlywheelSubsystem implements Subsystem {

    private final String name;

    protected FlywheelSubsystem(String name) {
        this.name = name;
    }

    @Override
    public final String getName() {
        return name;
    }

    @Logged
    protected final MutAngularVelocity angularVelocity = RadiansPerSecond.zero().mutableCopy();

    @Logged
    protected final MutAngularAcceleration angularAcceleration = RadiansPerSecondPerSecond.zero().mutableCopy();

    @Logged
    protected final MutVoltage voltage = Volts.zero().mutableCopy();

    @Logged
    protected final MutCurrent current = Amps.zero().mutableCopy();

    @Logged
    protected final MutAngularVelocity angularVelocitySetpoint = RadiansPerSecond.zero().mutableCopy();

    @Logged
    protected final MutAngularAcceleration angularAccelerationSetpoint = RadiansPerSecondPerSecond.zero().mutableCopy();

    @Logged
    protected final MutVoltage voltageSetpoint = Volts.zero().mutableCopy();

    @Logged
    protected final MutCurrent currentSetpoint = Amps.zero().mutableCopy();

    public final AngularVelocity getAngularVelocity() {
        return angularVelocity;
    }

    public final AngularAcceleration getAngularAcceleration() {
        return angularAcceleration;
    }

    public final MutVoltage getVoltage() {
        return voltage;
    }

    public final MutCurrent getCurrent() {
        return current;
    }

    public abstract void applyAngularVelocity(AngularVelocity velocity);

    public abstract void applyVoltage(Voltage voltage);

    public abstract void applyCurrent(Current current);

    @Override
    public void periodic() {

    }

    // Trigger Factories Go Here.

    // Command Factories Go Here.

    // SysIdCommand Factories Go Here.

    // Static Creation Factories go Here.


}
