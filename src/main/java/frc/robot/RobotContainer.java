/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.climb.LevelClimb;
import frc.robot.commands.climb.LiftOff;
import frc.robot.commands.climb.ManualClimb;
import frc.robot.commands.climb.ManualElevator;
import frc.robot.commands.climb.ManualWinch;
import frc.robot.commands.climb.PresetClimb;
import frc.robot.commands.controlPanel.Activate;
import frc.robot.commands.controlPanel.Position;
import frc.robot.commands.controlPanel.Rotation;
import frc.robot.commands.driveTrain.CurvatureDrive;
import frc.robot.commands.driveTrain.HighGear;
import frc.robot.commands.driveTrain.LowGear;
import frc.robot.commands.scoop.ScoopExpel;
import frc.robot.commands.scoop.ScoopIntake;
import frc.robot.commands.scoop.ScoopIntakePos;
import frc.robot.commands.scoop.ScoopLoadingStationPos;
import frc.robot.commands.scoop.ScoopLowGoalPos;
import frc.robot.commands.scoop.ScoopManual;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ControlPanel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Scoop;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveTrain driveTrain;
  private final Climb climb;
  private final Scoop scoop;
  private final ControlPanel controlPanel;
  static Joystick buttonStick;
  Joystick throttleStick;
  Joystick curveStick;
  JoystickButton isQuickTurnButton;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    driveTrain = new DriveTrain();
    climb = new Climb();
    scoop = new Scoop();
    controlPanel = new ControlPanel();
    throttleStick = new Joystick(OIConstants.throttleJoystickID);
    curveStick = new Joystick(OIConstants.curveJoystickID);
    buttonStick = new Joystick(OIConstants.buttonJoystickID);
    isQuickTurnButton = new JoystickButton(curveStick, OIConstants.isQuickTurnButtonID);
    // Configure the button bindings
    configureButtonBindings();

    driveTrain.setDefaultCommand(new CurvatureDrive(driveTrain, () -> throttleStick.getY(GenericHID.Hand.kLeft),
        () -> -curveStick.getX(GenericHID.Hand.kRight), () -> isQuickTurnButton.get()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //DriveTrain Buttons
    new JoystickButton(throttleStick, OIConstants.driveHighGearButtonID)
        .whenPressed(new HighGear(driveTrain));
    new JoystickButton(throttleStick, OIConstants.driveLowGearButtonID)
        .whenPressed(new LowGear(driveTrain));

    //Climb Buttons
    //Manual
    new JoystickButton(buttonStick, OIConstants.climbElevatorManualButtonID)
        .toggleWhenPressed(new ManualElevator(climb));
    new JoystickButton(buttonStick, OIConstants.climbWinchManualButtonID)
        .toggleWhenPressed(new ManualWinch(climb));
    new JoystickButton(buttonStick, OIConstants.climbManualButtonID)
        .toggleWhenPressed(new ManualClimb(climb));
    //Automated
    // new JoystickButton(buttonStick, OIConstants.climbPresetButtonID)
    //     .whenPressed(new PresetClimb(climb));
    // new JoystickButton(buttonStick, OIConstants.climbUpButtonID)
    //     .whenPressed(new LevelClimb(climb));
    new JoystickButton(buttonStick, OIConstants.climbEndButtonID)
        .whenPressed(new LiftOff(climb));

    //Scoop Arm Buttons
    //Manual
    new JoystickButton(buttonStick, OIConstants.scoopManualButtonID)
        .toggleWhenPressed(new ScoopManual(scoop));
    //Automated
    new JoystickButton(buttonStick, OIConstants.scoopIntakePosButtonID)
        .whileHeld(new ScoopIntakePos(scoop));
    new JoystickButton(buttonStick, OIConstants.scoopLoadingStationButtonID)
        .whileHeld(new ScoopLoadingStationPos(scoop));
    new JoystickButton(buttonStick, OIConstants.scoopLowGoalButtonID)
        .whileHeld(new ScoopLowGoalPos(scoop));

    //Scoop Intake/Expel Buttons
    new JoystickButton(buttonStick, OIConstants.scoopCollectButtonID)
        .whileHeld(new ScoopIntake(scoop));
    new JoystickButton(buttonStick, OIConstants.scoopExpelButtonID)
        .whileHeld(new ScoopExpel(scoop));

    //Control Panel Buttons
    new JoystickButton(buttonStick, OIConstants.controlPanelActivateButtonID)
        .toggleWhenPressed(new Activate(controlPanel));
    // new JoystickButton(buttonStick, OIConstants.controlPanelRotationButtonID)
    //     .whenPressed(new Rotation(controlPanel));
    // new JoystickButton(buttonStick, OIConstants.controlPanelPositionButtonID)
    //     .whenPressed(new Position(controlPanel));
  }

  public static double getClimbY() {
    return buttonStick.getY();
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   //return m_autoCommand;
  // }


}
