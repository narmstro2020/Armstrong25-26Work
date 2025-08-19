package org.digitalgoats.digilib.subsystems.flywheel;

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

final class TalonFxCurrentFlywheelSubsystem extends TalonFxFlywheelSubsystem {

    private final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityTorqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(0.0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

    @SafeVarargs
    TalonFxCurrentFlywheelSubsystem(
            String name,
            DCMotor gearbox,
            FlywheelConfig flywheelConfig,
            TalonFX primary,
            Pair<TalonFX, Boolean>... followers) {
        super(name, primary, followers);
    }

    @Override
    public void applyAngularVelocity(AngularVelocity angularVelocity) {
        angularVelocitySetpoint.mut_replace(angularVelocity);
        primary.setControl(motionMagicVelocityTorqueCurrentFOC.withVelocity(angularVelocitySetpoint));
    }

    @Override
    public void applyVoltage(Voltage voltage) {
        voltageSetpoint.mut_replace(voltage);
        primary.setControl(voltageOut.withOutput(voltageSetpoint));
    }

    @Override
    public void stopFlywheel() {

    }


}
