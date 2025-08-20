package org.digitalgoats.digilib.subsystems.flywheel;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.*;

public abstract class FlywheelSubsystem implements Subsystem {

    private final String name;
    protected final MutAngularVelocity angularVelocity = RadiansPerSecond.zero().mutableCopy();
    protected final MutAngularAcceleration angularAcceleration = RadiansPerSecondPerSecond.zero().mutableCopy();
    protected final MutVoltage voltage = Volts.zero().mutableCopy();
    protected final MutCurrent current = Amps.zero().mutableCopy();
    protected final MutAngularVelocity angularVelocitySetpoint = RadiansPerSecond.zero().mutableCopy();
    protected final MutVoltage voltageSetpoint = Volts.zero().mutableCopy();
    protected final MutCurrent currentSetpoint = Amps.zero().mutableCopy();
    private final DoubleLogEntry flywheelVelocityLogEntry;
    private final DoubleLogEntry flywheelAccelerationLogEntry;
    private final DoubleLogEntry flywheelVoltageLogEnry;
    private final DoubleLogEntry flywheelCurrentLogEntry;


    protected FlywheelSubsystem(String name) {
        this.name = name;
        this.register();
        flywheelVelocityLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "/" + name + "/velocity [radps]");
        flywheelAccelerationLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "/" + name + "/acceleration [radps2]");
        flywheelVoltageLogEnry = new DoubleLogEntry(DataLogManager.getLog(), "/" + name + "/voltage [volts]");
        flywheelCurrentLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "/" + name + "/current [amps]");

    }

    @Override
    public final String getName() {
        return name;
    }


    public final AngularVelocity getAngularVelocity() {
        return angularVelocity;
    }

    public final AngularAcceleration getAngularAcceleration() {
        return angularAcceleration;
    }

    public final Voltage getVoltage() {
        return voltage;
    }

    public final Current getCurrent() {
        return current;
    }

    public abstract void applyAngularVelocity(AngularVelocity velocity);

    public abstract void applyVoltage(Voltage voltage);

    public abstract void applyCurrent(Current current);

    public abstract void stopFlywheel();

    @Override
    public void periodic() {
        flywheelVelocityLogEntry.append(angularVelocity.baseUnitMagnitude());
        flywheelAccelerationLogEntry.append(angularAcceleration.baseUnitMagnitude());
        flywheelVoltageLogEnry.append(voltage.baseUnitMagnitude());
        flywheelCurrentLogEntry.append(current.baseUnitMagnitude());

    }

    // Trigger Factories Go Here.

    // Command Factories Go Here.
    public Command setVelocity(AngularVelocity velocitySetpoint) {
        return runOnce(() -> applyAngularVelocity(velocitySetpoint))
                .andThen(Commands.idle(this)).withName(getName() + ": Velocity Set to: " + velocitySetpoint);
    }

    public Command stop() {
        return run(this::stopFlywheel).withName(getName() + ": Flywheel Stopped");
    }
    // SysIdCommand Factories Go Here.


}
