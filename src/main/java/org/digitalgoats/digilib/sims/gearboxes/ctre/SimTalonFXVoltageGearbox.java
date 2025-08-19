package org.digitalgoats.digilib.sims.gearboxes.ctre;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

public final class SimTalonFXVoltageGearbox extends SimTalonFXGearbox {

    private final MutVoltage voltage = Volts.zero().mutableCopy();
    private final MutCurrent current = Amps.zero().mutableCopy();
    private final Voltage ks;
    private final double maxInputVoltage;

    public SimTalonFXVoltageGearbox(
            TalonFX primary,
            LinearSystem<N2, N1, N2> plant,
            DCMotor gearbox,
            Voltage ks,
            double maxInputVoltage,
            double... measurementStdDevs) {
        super(primary, plant, gearbox, measurementStdDevs);
        this.ks = ks;
        this.maxInputVoltage = maxInputVoltage;
    }

    @Override
    public Voltage getVoltage() {
        return voltage;
    }

    @Override
    public Current getCurrent() {
        return current;
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        return NumericalIntegration.rkdp((_x, _u) -> {
                    var c = MatBuilder.fill(Nat.N2(), Nat.N1(), 0, -Math.signum(_x.get(1, 0)) * ks.baseUnitMagnitude() * m_plant.getB(1, 0));
                    return m_plant.getA().times(_x).plus(m_plant.getB().times(_u)).plus(c);
                },
                currentXhat,
                u, dtSeconds);
    }

    @Override
    public void update(double dtSeconds) {
        super.update(dtSeconds);


    }
}
