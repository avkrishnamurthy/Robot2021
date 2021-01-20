// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ElevatorReleaseHook extends CommandBase {
  private final Climb m_climb;
  /** Creates a new ElevatorReleaseHook. */
  public ElevatorReleaseHook(Climb climb) {
    this.m_climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.setElevatorPos(ClimbConstants.climbElevatorReleaseHookPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climb.isElevatorAtPos(ClimbConstants.climbElevatorReleaseHookPosition);
  }
}
