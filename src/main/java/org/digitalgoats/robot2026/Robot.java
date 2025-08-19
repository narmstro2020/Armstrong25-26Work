// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.digitalgoats.robot2026;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.digitalgoats.digilib.subsystems.flywheel.FlywheelConfig;
import org.digitalgoats.digilib.subsystems.flywheel.FlywheelSubsystem;
import org.digitalgoats.digilib.subsystems.flywheel.TalonFxVoltageFlywheelSubsystem;

import static edu.wpi.first.units.Units.RPM;
import static org.digitalgoats.digilib.subsystems.flywheel.TalonFxVoltageFlywheelSubsystem.*;

@Logged
public class Robot extends TimedRobot {


    public Robot() {

        FlywheelConfig flywheelConfig = new FlywheelConfig(
                "Flywheel",
                0.12,
                0.228137,
                0.001411,
                12.0,
                DCMotor.getKrakenX60(1),
                12.0,
                40.0);

        TalonFX talonFX = new TalonFX(14);

        FlywheelSubsystem flywheelSubsystem = flywheelFromKrakenX60("Flywheel", false, flywheelConfig, talonFX);

        CommandXboxController controller = new CommandXboxController(0);
        controller.a().onTrue(flywheelSubsystem.setVelocity(RPM.of(500.0)));
        controller.a().onFalse(flywheelSubsystem.stop());


        EpilogueConfiguration epilogueConfiguration = new EpilogueConfiguration();
        epilogueConfiguration.backend = new FileBackend(DataLogManager.getLog());
        epilogueConfiguration.root = "Telemetry";
        epilogueConfiguration.minimumImportance = Logged.Importance.DEBUG;
        DriverStation.startDataLog(DataLogManager.getLog());
        Epilogue.bind(this);


    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }


}
