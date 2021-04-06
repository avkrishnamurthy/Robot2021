// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBounce extends SequentialCommandGroup {
  DriveTrain m_driveTrain;
  /** Creates a new AutoBounce. */
  public AutoBounce(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BounceP1(driveTrain),
      new BounceP2(driveTrain),
      new BounceP3(driveTrain).withTimeout(0.9),
      new BounceP4(driveTrain),
      new BounceP5(driveTrain).withTimeout(0.8),
      new BounceP6(driveTrain).withTimeout(1.25),
      new BounceP7(driveTrain),
      new BounceP8(driveTrain).withTimeout(0.95),
      new BounceP9(driveTrain)
    );
  }
}
