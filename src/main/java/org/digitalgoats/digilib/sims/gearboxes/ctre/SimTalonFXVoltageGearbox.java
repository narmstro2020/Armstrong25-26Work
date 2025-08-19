package org.digitalgoats.digilib.sims.gearboxes.ctre;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public final class SimTalonFXVoltageGearbox extends SimTalonFXGearbox {

    private final MutVoltage voltage = Volts.zero().mutableCopy();
    private final MutCurrent current = Amps.zero().mutableCopy();
    private final Voltage ks;
    private final double maxInputVoltage;
    private final double maxCurrent;
    private static final double VELOCITY_THRESHOLD = 0.01; // rad/s threshold for static friction

    public SimTalonFXVoltageGearbox(
            TalonFX primary,
            LinearSystem<N2, N1, N2> plant,
            DCMotor gearbox,
            Voltage ks,
            double maxInputVoltage,
            double maxCurrent,
            double... measurementStdDevs) {
        super(primary, plant, gearbox, measurementStdDevs);
        this.ks = ks;
        this.maxInputVoltage = maxInputVoltage;
        this.maxCurrent = maxCurrent;
    }

    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        double inputVoltage = primary.getMotorVoltage().getValueAsDouble();
        inputVoltage = MathUtil.clamp(inputVoltage, -maxInputVoltage, maxInputVoltage);
        double currentDraw = gearbox.getCurrent(currentXhat.get(1, 0) * gearing, inputVoltage);
        currentDraw = MathUtil.clamp(currentDraw, -maxCurrent, maxCurrent);
        inputVoltage = gearbox.getVoltage(gearbox.getTorque(currentDraw), currentXhat.get(1, 0) * gearing);
        u.set(0, 0, inputVoltage);

        final double ksValue = ks.baseUnitMagnitude();
        final double bValue = m_plant.getB(1, 0);

        return NumericalIntegration.rkdp((_x, _u) -> {
                    double vel = _x.get(1, 0);
                    double appliedVoltage = _u.get(0, 0);

                    // Calculate the base dynamics
                    Matrix<N2, N1> xDot = m_plant.getA().times(_x).plus(m_plant.getB().times(_u));

                    // Apply static friction compensation
                    // Use a smooth transition near zero velocity to avoid discontinuity
                    if (Math.abs(vel) > VELOCITY_THRESHOLD) {
                        // Normal static friction based on direction of motion
                        double frictionCompensation = -Math.signum(vel) * ksValue * bValue;
                        xDot.set(1, 0, xDot.get(1, 0) + frictionCompensation);
                    } else if (Math.abs(appliedVoltage) > ksValue) {
                        // Near zero velocity - only apply friction if voltage overcomes static friction
                        // This prevents oscillation at zero
                        double frictionCompensation = -Math.signum(appliedVoltage) * ksValue * bValue;
                        xDot.set(1, 0, xDot.get(1, 0) + frictionCompensation);
                    } else {
                        // Voltage doesn't overcome static friction, so velocity stays at zero
                        xDot.set(1, 0, 0.0);
                    }

                    return xDot;
                },
                currentXhat,
                u, dtSeconds);
    }

    @Override
    public void update(double dtSeconds) {
        super.update(dtSeconds);
        var acceleration = gearbox.getTorque(primary.getTorqueCurrent().getValueAsDouble()) / J.baseUnitMagnitude();
        primarySim.setSupplyVoltage(RobotController.getBatteryVoltage());
        primarySim.setRawRotorPosition(m_y.get(0, 0) * gearing / 2 / Math.PI);
        primarySim.setRotorVelocity(m_y.get(1, 0) * gearing / 2 / Math.PI);
        primarySim.setRotorAcceleration(acceleration * gearing / 2 / Math.PI);
    }
}