package org.digitalgoats.digilib.sims.gearboxes.ctre;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

public final class SimTalonFXCurrentGearbox extends SimTalonFXGearbox {

    private final Voltage ks;
    private final double maxInputCurrent;
    private final double maxVoltage;

    public SimTalonFXCurrentGearbox(
            TalonFX primary,
            LinearSystem<N2, N1, N2> plant,
            double gearing,
            DCMotor gearbox,
            Voltage ks,
            double maxVoltage,
            double maxInputCurrent,
            double... measurementStdDevs) {
        super(primary,
                plant,
                gearing,
                KilogramSquareMeters.of(
                        gearing * gearbox.KtNMPerAmp /
                                plant.getB(1, 0)),
                gearbox,
                measurementStdDevs);
        this.ks = ks;
        this.maxVoltage = maxVoltage;
        this.maxInputCurrent = maxInputCurrent;
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        double inputCurrent = primary.getTorqueCurrent().getValueAsDouble();
        inputCurrent = MathUtil.clamp(inputCurrent, -maxInputCurrent, maxInputCurrent);
        double voltageUsed = gearbox.getVoltage(gearbox.getTorque(inputCurrent), currentXhat.get(1, 0) * gearing);
        voltageUsed = MathUtil.clamp(voltageUsed, -maxVoltage, maxVoltage);
        inputCurrent = gearbox.getCurrent(currentXhat.get(1, 0) * gearing, voltageUsed);

        if (Math.abs(inputCurrent) > ks.baseUnitMagnitude()) {
            inputCurrent += -Math.signum(currentXhat.get(1, 0)) * ks.baseUnitMagnitude();
        } else if (Math.abs(currentXhat.get(1, 0)) >= 0.0 && Math.abs(inputCurrent) <= ks.baseUnitMagnitude()) {
            inputCurrent = -ks.baseUnitMagnitude() * Math.signum(currentXhat.get(1, 0));
        } else {
            inputCurrent = 0.0;

        }

        u.set(0, 0, inputCurrent);
        return super.updateX(currentXhat, u, dtSeconds);
    }
}