// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveForward extends CommandBase {
  private final DriveTrain m_driveTrain;
  private final double m_meterDistance;
  private final double m_speed;
  /** Creates a new DriveForward. */
  public DriveForward(DriveTrain driveTrain, double meterDistance, double speed) {
    this.m_driveTrain = driveTrain;
    this.m_meterDistance = meterDistance;
    this.m_speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.resetEncoders();
    m_driveTrain.curvatureDrive(-m_speed, 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.curvatureDrive(-m_speed, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.curvatureDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    //return Math.abs(m_driveTrain.getAverageEncoderDistance()) >= m_meterDistance;
    return (m_driveTrain.isLeftAbovePos(-20000));// && m_driveTrain.isRightAtPos(-20000));
  }
}
