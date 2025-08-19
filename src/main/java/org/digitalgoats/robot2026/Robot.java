// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.digitalgoats.robot2026;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.EpilogueConfiguration;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged
public class Robot extends TimedRobot {


    public Robot() {

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
