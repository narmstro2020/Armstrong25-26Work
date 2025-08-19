package org.digitalgoats.digilib.sims.gearboxes;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public abstract class SimGearbox extends LinearSystemSim<N2, N1, N2> {

    protected final DCMotor gearbox;
    protected final double gearing;


    protected SimGearbox(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double... measurementStdDevs) {
        super(plant, measurementStdDevs);
        this.gearbox = gearbox;
        gearing = -gearbox.KvRadPerSecPerVolt * plant.getA(1, 1) / plant.getB().get(1, 0);
    }

    public final void setPosition(Angle angle){
        m_x.set(0, 0, angle.baseUnitMagnitude());
    }

    public abstract Voltage getVoltage();

    public abstract Current getCurrent();



}
