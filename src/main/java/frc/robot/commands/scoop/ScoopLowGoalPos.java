// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ScoopConstants;
import frc.robot.subsystems.Scoop;

public class ScoopLowGoalPos extends CommandBase {
  private final Scoop m_scoop;
  /** Creates a new ScoopLowGoalPos. */
  public ScoopLowGoalPos(Scoop scoop) {
    this.m_scoop = scoop;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scoop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_scoop.setScoopArmPos(ScoopConstants.scoopArmLowGoalPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_scoop.stopScoopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_scoop.isScoopArmAtPos(ScoopConstants.scoopArmLowGoalPosition);
  }
}
