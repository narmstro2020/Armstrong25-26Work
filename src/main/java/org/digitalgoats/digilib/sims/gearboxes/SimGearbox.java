package org.digitalgoats.digilib.sims.gearboxes;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

public abstract class SimGearbox extends LinearSystemSim<N2, N1, N2> {

    protected final DCMotor gearbox;
    protected final double gearing;
    protected final MomentOfInertia J;


    protected SimGearbox(LinearSystem<N2, N1, N2> plant,
                         double gearing,
                         MomentOfInertia J,
                         DCMotor gearbox, double... measurementStdDevs) {
        super(plant, measurementStdDevs);
        this.gearbox = gearbox;
        this.gearing = gearing;
        this.J = J;

    }

    public final void setPosition(Angle angle) {
        m_x.set(0, 0, angle.baseUnitMagnitude());
    }

}
