package org.digitalgoats.digilib.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;

public record FlywheelConfig(
        String name,
        double ks,
        double kv,
        double ka,
        double kp,
        double kd,
        double gearing,
        DCMotor gearbox,
        double maxVoltage,
        double maxCurrent) {
}
