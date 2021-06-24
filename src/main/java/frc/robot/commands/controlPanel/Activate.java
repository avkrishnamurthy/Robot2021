// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.controlPanel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanel;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Activate extends CommandBase {
  private final ControlPanel m_controlPanel;
  /** Creates a new Activate. */
  public Activate(ControlPanel controlPanel) {
    this.m_controlPanel = controlPanel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(controlPanel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_controlPanel.isControlPanelUpOrDown() == DoubleSolenoid.Value.kForward) {
      m_controlPanel.extendControlPanel();
    }
    else if (m_controlPanel.isControlPanelUpOrDown() == DoubleSolenoid.Value.kReverse) {
      m_controlPanel.retractControlPanel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
