// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.digitalgoats.robot2026;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {



    public Robot() {


    }


    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }


}
