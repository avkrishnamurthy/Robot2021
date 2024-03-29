// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MainAutoComponents;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.driveTrain.MainAutoDrive.LowGoalBack;
import frc.robot.commands.scoop.ScoopIntakePos;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Scoop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoLowGoalBackUpIntakePosition extends ParallelCommandGroup {
  /** Creates a new AutoLowBackUpIntakePosition. */
  public AutoLowGoalBackUpIntakePosition(DriveTrain driveTrain, Scoop scoop) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LowGoalBack(driveTrain), new ScoopIntakePos(scoop));
  }
}
