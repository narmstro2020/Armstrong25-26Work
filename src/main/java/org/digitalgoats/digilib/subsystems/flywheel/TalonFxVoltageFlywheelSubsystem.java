package org.digitalgoats.digilib.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.digitalgoats.digilib.sims.gearboxes.SimGearbox;
import org.digitalgoats.digilib.sims.gearboxes.ctre.SimTalonFXVoltageGearbox;
import org.digitalgoats.robot2026.Robot;

import static edu.wpi.first.units.Units.Volts;

public final class TalonFxVoltageFlywheelSubsystem extends TalonFxFlywheelSubsystem {

    private final MotionMagicVelocityVoltage motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);
    private double lastSimTime;

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

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Voltage.PeakForwardVoltage = flywheelConfig.maxVoltage();
        talonFXConfiguration.Voltage.PeakReverseVoltage = -flywheelConfig.maxVoltage();
        talonFXConfiguration.CurrentLimits.StatorCurrentLimit = flywheelConfig.maxCurrent();
        talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfiguration.Feedback.SensorToMechanismRatio = flywheelConfig.gearing();
        talonFXConfiguration.Feedback.RotorToSensorRatio = 1;
        talonFXConfiguration.MotionMagic.MotionMagicAcceleration = flywheelConfig.maxVoltage() / flywheelConfig.ka();
        talonFXConfiguration.Slot0.kS = flywheelConfig.ks();
        talonFXConfiguration.Slot0.kV = flywheelConfig.kv() * 2 * Math.PI;
        talonFXConfiguration.Slot0.kA = flywheelConfig.ka() * 2 * Math.PI;
        talonFXConfiguration.Slot0.kP = flywheelConfig.kp() * 2 * Math.PI;
        talonFXConfiguration.Slot0.kD = flywheelConfig.kd() * 2 * Math.PI;
        primary.getConfigurator().apply(talonFXConfiguration);

        if (Robot.isSimulation()) {
            var plant = LinearSystemId.createDCMotorSystem(flywheelConfig.kv(), flywheelConfig.ka());
            startSimThread(plant,
                    gearbox,
                    Volts.of(flywheelConfig.ks()),
                    flywheelConfig.maxVoltage(),
                    flywheelConfig.maxCurrent());
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

    private void startSimThread(LinearSystem<N2, N1, N2> plant,
                                DCMotor gearbox,
                                Voltage ks,
                                double maxVoltage, double maxCurrent) {

        SimGearbox simGearbox = new SimTalonFXVoltageGearbox(
                primary,
                plant,
                gearbox,
                ks,
                maxVoltage,
                maxCurrent);
        lastSimTime = Timer.getFPGATimestamp();
        Notifier simNotifier = new Notifier(() -> {
            double currentTimeSeconds = Timer.getFPGATimestamp();
            double dtSeconds = currentTimeSeconds - lastSimTime;
            lastSimTime = currentTimeSeconds;
            simGearbox.update(dtSeconds);
        });
        simNotifier.startPeriodic(0.001);
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


}
