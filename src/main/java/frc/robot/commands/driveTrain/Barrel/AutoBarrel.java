// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain.Barrel;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBarrel extends SequentialCommandGroup {
  /** Creates a new AutoBarrel. */
  public AutoBarrel(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BarrelP1(driveTrain).withTimeout(1.08),
      new BarrelP2(driveTrain),
      new BarrelP3(driveTrain).withTimeout(0.95),
      new BarrelP4(driveTrain),
      new BarrelP5(driveTrain).withTimeout(0.86),
      new BarrelP6(driveTrain),
      new BarrelP7(driveTrain).withTimeout(2.2)
    );
  }
}
