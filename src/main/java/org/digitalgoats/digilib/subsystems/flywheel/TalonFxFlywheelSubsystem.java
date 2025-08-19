package org.digitalgoats.digilib.subsystems.flywheel;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Current;

import java.util.Arrays;

public abstract class TalonFxFlywheelSubsystem extends FlywheelSubsystem {

    protected final TalonFX primary;
    protected final TalonFX[] followers;
    private final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0.0);

    public TalonFxFlywheelSubsystem(
            String name,
            final TalonFX primary,
            final Pair<TalonFX, Boolean>[] followers) {
        super(name);
        this.primary = primary;
        Arrays.stream(followers).forEach(f -> {
            Follower follower = new Follower(primary.getDeviceID(), f.getSecond());
            f.getFirst().setControl(follower);
        });
        this.followers = Arrays.stream(followers).map(Pair::getFirst).toArray(TalonFX[]::new);

    }

    @Override
    public void periodic() {
        voltage.mut_replace(primary.getMotorVoltage().getValue());
        double currentNow = 0.0;
        for (TalonFX follower : followers) {
            currentNow += follower.getTorqueCurrent().getValueAsDouble();
        }
        current.mut_setMagnitude(currentNow);
        angularVelocity.mut_replace(primary.getVelocity().getValue());
        angularAcceleration.mut_replace(primary.getAcceleration().getValue());
    }

    @Override
    public final void applyCurrent(Current current) {
        currentSetpoint.mut_replace(current);
        primary.setControl(torqueCurrentFOC.withOutput(currentSetpoint));
    }
}
