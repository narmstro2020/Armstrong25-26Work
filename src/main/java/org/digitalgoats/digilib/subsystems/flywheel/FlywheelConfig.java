package org.digitalgoats.digilib.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;

public record FlywheelConfig(
        String name,
        double ks,
        double kv,
        double ka,
        double gearing,
        DCMotor gearbox) {
}
