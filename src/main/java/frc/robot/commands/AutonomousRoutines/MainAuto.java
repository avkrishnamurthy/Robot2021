// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousRoutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MainAutoComponents.AutoLowGoalBackUpIntakePosition;
import frc.robot.commands.MainAutoComponents.TrenchRun;
import frc.robot.commands.driveTrain.MainAutoDrive.AutoDriveToTrench;
import frc.robot.commands.driveTrain.MainAutoDrive.StartToLowGoal;
import frc.robot.commands.scoop.ScoopExpel;
import frc.robot.commands.scoop.ScoopLowGoalPos;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Scoop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MainAuto extends SequentialCommandGroup {
  /** Creates a new StartShoot. */
  public MainAuto(DriveTrain driveTrain, Scoop scoop) {
    addCommands(
      new StartToLowGoal(driveTrain),
      new ScoopLowGoalPos(scoop),
      new ScoopExpel(scoop).withTimeout(1),
      new AutoLowGoalBackUpIntakePosition(driveTrain, scoop),
      new AutoDriveToTrench(driveTrain),
      new TrenchRun(driveTrain, scoop));
  }
}
