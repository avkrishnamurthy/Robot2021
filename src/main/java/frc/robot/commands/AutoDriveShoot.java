// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.commands.scoop.ScoopExpel;
import frc.robot.commands.scoop.ScoopLowGoalPos;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Scoop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDriveShoot extends SequentialCommandGroup {
  /** Creates a new AutoDriveShoot. */
  public AutoDriveShoot(DriveTrain driveTrain, Scoop scoop) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ScoopLowGoalPos(scoop), 
                new DriveForward(driveTrain,AutonomousConstants.kAutoDriveDistanceMeters,AutonomousConstants.kAutoDriveSpeed),
                new ScoopExpel(scoop).withTimeout(1));
  }
}
