// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driveTrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class CurvatureDrive extends CommandBase {
  private final DriveTrain m_driveTrain;
  private final DoubleSupplier m_y;
  private final DoubleSupplier m_x;
  private final BooleanSupplier m_z;
  /** Creates a new CurvatureDrive. */
  public CurvatureDrive(DriveTrain driveTrain, DoubleSupplier y, DoubleSupplier x, BooleanSupplier z) {
    this.m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    m_y = y;
    m_x = x;
    m_z = z;

  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double y = RobotContainer.getY();
    // double x = RobotContainer.getX();
    // boolean z = RobotContainer.getQuickTurn();

    m_driveTrain.curvatureDrive(m_y.getAsDouble(), m_x.getAsDouble(), m_z.getAsBoolean());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
