// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoop;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Scoop;

public class ScoopExpel extends CommandBase {
  private final Scoop m_scoop;
  /** Creates a new ScoopExpel. */
  public ScoopExpel(Scoop scoop) {
    this.m_scoop = scoop;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(scoop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_scoop.scoopIntakeExpel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_scoop.scoopIntakeRest();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
