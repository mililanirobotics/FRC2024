// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.ManualConveyorCommand;
import frc.robot.commands.TestAutoConveyorCommand;
import frc.robot.commands.TestAutoIntakeCommand;
import frc.robot.commands.TestManualIntakeCommand;
import frc.robot.commands.TestManualScoringCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.RollerIntakeSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //initializing subsystems
  // private final AprilTagsSubsystem aprilTagsSubsystem = new AprilTagsSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final RollerIntakeSubsystem intakeSubsystem = new RollerIntakeSubsystem();
  private final ScoringSubsystem scoringSubsytem = new ScoringSubsystem();
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();

  //initailizing gamepads
  private final GenericHID priamryJoystick = new GenericHID(JoystickConstants.kPrimaryPort);
  private final GenericHID secondaryJoystick = new GenericHID(JoystickConstants.kSecondaryPort);
  private final GenericHID thirdJoystick = new GenericHID(2);

  //initializing limit switch
  private final DigitalInput intakeLimitSwitch = new DigitalInput(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //creating default commands for manually controller the intake, conveyor, and scorer
    intakeSubsystem.setDefaultCommand(new TestManualIntakeCommand(secondaryJoystick, intakeSubsystem));
    conveyorSubsystem.setDefaultCommand(new ManualConveyorCommand(thirdJoystick, conveyorSubsystem));
    scoringSubsytem.setDefaultCommand(new TestManualScoringCommand(priamryJoystick, scoringSubsytem));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //runs the automated scoring command when pressing the A button on the secondary gamepad
    // new JoystickButton(secondaryJoystick, JoystickConstants.kAButtonPort)
    // .onTrue(

    //   IntakeToScoringCommand()
    // );
  }

  /**
   * Returns a sequential command that goes through the motion of scoring (uses limit switches)
   * @return The full scoring sequential command
   */
  public Command IntakeToScoringCommand() {
    return new SequentialCommandGroup(
      new TestAutoIntakeCommand(intakeSubsystem, conveyorSubsystem, intakeLimitSwitch, 1),
      new TestAutoConveyorCommand(conveyorSubsystem, intakeLimitSwitch, 1),
      new PrintCommand("Command Ended")
    );
  }
}
  