// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SecondBall extends CommandBase {
  DriveTrain m_driveTrain;
  /** Creates a new SecondBall. */
  public SecondBall(DriveTrain driveTrain) {
    this.m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
    m_driveTrain.resetGyro();
    m_driveTrain.curvatureDrive(-0.8, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.curvatureDrive(-0.8, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.curvatureDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.getRightEncoder() >= (2048.0 * 7.5*4.3);// * 3.39053);
  }
}
