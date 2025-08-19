package org.digitalgoats.digilib.subsystems.flywheel;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.digitalgoats.robot2026.Robot;

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

        if (Robot.isSimulation()) {
            startSimThread();
        }

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

    private void startSimThread() {
        SimGearbox simGearbox = new SimGearbox();

    }

    @SafeVarargs
    public static FlywheelSubsystem flywheelFromKrakenX60(
            String name,
            boolean enableFOC,
            FlywheelConfig flywheelConfig,
            TalonFX primary,
            Pair<TalonFX, Boolean>... followers) {
        return new TalonFxVoltageFlywheelSubsystem(
                name,
                enableFOC,
                enableFOC
                        ? DCMotor.getKrakenX60Foc(1 + followers.length)
                        : DCMotor.getKrakenX60(1 + followers.length),
                flywheelConfig,
                primary,
                followers);
    }

    @SafeVarargs
    public static FlywheelSubsystem flywheelFromKrakenX60FOC(
            String name,
            FlywheelConfig flywheelConfig,
            TalonFX primary,
            Pair<TalonFX, Boolean>... followers) {
        return new TalonFxCurrentFlywheelSubsystem(
                name,
                DCMotor.getKrakenX60(1 + followers.length),
                flywheelConfig,
                primary,
                followers);
    }
}
