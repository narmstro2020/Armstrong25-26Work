package org.digitalgoats.digilib.subsystems.flywheel;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public final class TalonFxVoltageFlywheelSubsystem extends TalonFxFlywheelSubsystem {

    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    @SafeVarargs
    private TalonFxVoltageFlywheelSubsystem(
            String name,
            boolean enableFOC,
            DCMotor gearbox,
            FlywheelConfig flywheelConfig,
            TalonFX primary,
            Pair<TalonFX, Boolean>... followers) {
        super(name, primary, followers);
        motionMagicVelocityVoltage.withEnableFOC(enableFOC);
        voltageOut.withEnableFOC(enableFOC);
    }

    @Override
    public void applyAngularVelocity(AngularVelocity angularVelocity) {
        angularVelocitySetpoint.mut_replace(angularVelocity);
        primary.setControl(motionMagicVelocityVoltage.withVelocity(angularVelocitySetpoint));
    }

    @Override
    public void applyVoltage(Voltage voltage) {
        voltageSetpoint.mut_replace(voltage);
        primary.setControl(voltageOut.withOutput(voltageSetpoint));
    }

}
