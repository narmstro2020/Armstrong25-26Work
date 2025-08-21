package org.digitalgoats.digilib.sims.gearboxes.ctre;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

public final class SimTalonFXVoltageGearbox extends SimTalonFXGearbox {

    private final Voltage ks;
    private final double maxInputVoltage;
    private final double maxCurrent;

    public SimTalonFXVoltageGearbox(
            TalonFX primary,
            LinearSystem<N2, N1, N2> plant,
            DCMotor gearbox,
            Voltage ks,
            double maxInputVoltage,
            double maxCurrent,
            double... measurementStdDevs) {
        super(primary,
                plant,
                -gearbox.KvRadPerSecPerVolt * plant.getA(1, 1) / plant.getB().get(1, 0),
                KilogramSquareMeters.of((-gearbox.KvRadPerSecPerVolt * plant.getA(1, 1) / plant.getB().get(1, 0)) * gearbox.KtNMPerAmp / gearbox.rOhms / plant.getB(1, 0)),
                gearbox,
                measurementStdDevs);
        this.ks = ks;
        this.maxInputVoltage = maxInputVoltage;
        this.maxCurrent = maxCurrent;
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        double appliedVoltage = primary.getMotorVoltage().getValueAsDouble();
        double inputVoltage = MathUtil.clamp(appliedVoltage, -maxInputVoltage, maxInputVoltage);

        if (Math.abs(inputVoltage) >= ks.baseUnitMagnitude()) {
            inputVoltage += -Math.signum(currentXhat.get(1, 0)) * ks.baseUnitMagnitude();
        } else if (Math.abs(currentXhat.get(1, 0)) > 0.0 && Math.abs(inputVoltage) < ks.baseUnitMagnitude()) {
            inputVoltage = -ks.baseUnitMagnitude() * Math.signum(currentXhat.get(1, 0));
        } else {
            inputVoltage = 0.0;
        }

        u.set(0, 0, inputVoltage);
        var newStates = super.updateX(currentXhat, u, dtSeconds);

        if (Math.abs(inputVoltage) <= ks.baseUnitMagnitude() && Math.signum(newStates.get(1, 0)) != Math.signum(currentXhat.get(1, 0))) {
            newStates.set(1, 0, 0);
            return newStates;
        }else{
            return newStates;
        }
    }
}