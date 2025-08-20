// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.digitalgoats.robot2026;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.digitalgoats.digilib.subsystems.flywheel.FlywheelConfig;
import org.digitalgoats.digilib.subsystems.flywheel.FlywheelSubsystem;
import org.digitalgoats.digilib.subsystems.flywheel.TalonFxCurrentFlywheelSubsystem;

import static edu.wpi.first.units.Units.RPM;
import static org.digitalgoats.digilib.subsystems.flywheel.TalonFxCurrentFlywheelSubsystem.*;
import static org.digitalgoats.digilib.subsystems.flywheel.TalonFxVoltageFlywheelSubsystem.*;

public class Robot extends TimedRobot {

    public Robot() {

        DriverStation.startDataLog(DataLogManager.getLog());
        DataLogManager.start();

        FlywheelConfig flywheelVoltageConfig = new FlywheelConfig(
                "Flywheel",
                0.12,
                0.228137,
                0.001411,
                0.11,
                0.0000,
                12.0,
                DCMotor.getKrakenX60(1),
                12.0,
                60.0);

        TalonFX talonVoltageFX = new TalonFX(14);

        FlywheelSubsystem flywheelVoltageSubsystem = flywheelFromKrakenX60(
                "Flywheel-Voltage",
                false, flywheelVoltageConfig, talonVoltageFX);

        FlywheelConfig flywheelCurrentConfig = new FlywheelConfig(
                "Flywheel-Current",
                0.12 / DCMotor.getKrakenX60Foc(1).rOhms,
                0.0,
                0.001411 / DCMotor.getKrakenX60Foc(1).rOhms,
                0.1,
                0.0000,
                12.0,
                DCMotor.getKrakenX60Foc(1),
                12.0,
                60.0);

        TalonFX talonCurrentFX = new TalonFX(15);

        FlywheelSubsystem flywheelCurrentSubsystem = flywheelFromKrakenX60FOC(
                "Flywheel-Current",
                flywheelCurrentConfig, talonCurrentFX);

        CommandXboxController controller = new CommandXboxController(0);
        controller.a().onTrue(flywheelVoltageSubsystem.setVelocity(RPM.of(250)));
        controller.a().onFalse(flywheelVoltageSubsystem.stop());

        controller.b().onTrue(flywheelCurrentSubsystem.setVelocity(RPM.of(250)));
        controller.b().onFalse(flywheelCurrentSubsystem.stop());

        SmartDashboard.putData("Commands", CommandScheduler.getInstance());


    }


    @Override
    public void robotPeriodic() { CommandScheduler.getInstance().run(); }


}
