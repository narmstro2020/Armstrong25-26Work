package org.digitalgoats.digilib.sims.gearboxes.ctre;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import org.digitalgoats.digilib.sims.gearboxes.SimGearbox;

public abstract class SimTalonFXGearbox extends SimGearbox {

    protected final TalonFX primary;
    protected final TalonFXSimState primarySim;

    protected SimTalonFXGearbox(
            TalonFX primary,
            LinearSystem<N2, N1, N2> plant,
            DCMotor gearbox,
            double... measurementStdDevs) {
        super(plant, gearbox, measurementStdDevs);
        this.primary = primary;
        this.primarySim = primary.getSimState();
    }
}
