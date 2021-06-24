// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain.Slalom;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSlalom extends SequentialCommandGroup {
  /** Creates a new AutoSlalom. */
  public AutoSlalom(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new SlalomP1(driveTrain), 
    new SlalomP2(driveTrain),
    new SlalomP3(driveTrain).withTimeout(0.985), //0.98
    new SlalomP4(driveTrain),
    new SlalomP5(driveTrain),
    new SlalomP3(driveTrain).withTimeout(0.2),
    new SlalomP6(driveTrain),
    new SlalomP3(driveTrain).withTimeout(0.95), //0.97
    new SlalomP7(driveTrain),
    new SlalomP8(driveTrain),
    new SlalomP9(driveTrain).withTimeout(0.2));
  }
}
