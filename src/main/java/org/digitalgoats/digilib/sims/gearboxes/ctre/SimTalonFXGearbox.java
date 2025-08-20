package org.digitalgoats.digilib.sims.gearboxes.ctre;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.digitalgoats.digilib.sims.gearboxes.SimGearbox;

public abstract class SimTalonFXGearbox extends SimGearbox {

    protected final TalonFX primary;
    protected final TalonFXSimState primarySim;

    protected SimTalonFXGearbox(
            TalonFX primary,
            LinearSystem<N2, N1, N2> plant,
            double gearing,
            MomentOfInertia J,
            DCMotor gearbox,
            double... measurementStdDevs) {
        super(plant, gearing, J, gearbox, measurementStdDevs);
        this.primary = primary;
        this.primarySim = primary.getSimState();
    }

    @Override
    public final void update(double dtSeconds) {
        super.update(dtSeconds);
        SmartDashboard.putNumber(this.getClass().getName() + "newX", m_y.get(0, 0));
        SmartDashboard.putNumber(this.getClass().getName() + "newV", m_y.get(1, 0));
        var acceleration = gearbox.getTorque(primary.getTorqueCurrent().getValueAsDouble()) / J.baseUnitMagnitude();
        SmartDashboard.putNumber(this.getClass().getName() + "newA", acceleration);
        primarySim.setSupplyVoltage(RobotController.getBatteryVoltage());
        primarySim.setRawRotorPosition(m_y.get(0, 0) * gearing / 2 / Math.PI);
        primarySim.setRotorVelocity(m_y.get(1, 0) * gearing / 2 / Math.PI);
        primarySim.setRotorAcceleration(acceleration * gearing / 2 / Math.PI);
    }
}
