// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.driveTrain.AngleThirdBall;
import frc.robot.commands.driveTrain.RedPathAFirstBall;
import frc.robot.commands.driveTrain.RedPathASecondBall;
import frc.robot.commands.driveTrain.SecondBall;
import frc.robot.commands.driveTrain.SlalomP10;
import frc.robot.commands.driveTrain.ThirdBall;
import frc.robot.commands.driveTrain.TrenchPath;
import frc.robot.commands.driveTrain.TurnToFinish;
import frc.robot.commands.scoop.ScoopIntakePos;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Scoop;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RedPathAGalacticSearch extends SequentialCommandGroup {
  /** Creates a new RedPathAGalacticSearch. */
  public RedPathAGalacticSearch(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //addCommands(new RedPathAIntakeFirstBall(driveTrain, scoop));
    addCommands(
    new RedPathAFirstBall(driveTrain),
    new RedPathASecondBall(driveTrain), 
    new SecondBall(driveTrain), 
    new AngleThirdBall(driveTrain), 
    new ThirdBall(driveTrain), 
    new TurnToFinish(driveTrain),
    new TrenchPath(driveTrain).withTimeout(1.4),
    new SlalomP10(driveTrain).withTimeout(0.2));
  }
}
