// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.IntakeConveyorConstants;

import frc.robot.commands.IdentifyAprilTagCommand;
import frc.robot.commands.IntakeConveyorCommand;
import frc.robot.commands.VerticalConveyorCommand;

import frc.robot.subsystems.AprilTagsSubsystem;
import frc.robot.subsystems.IntakeConveyorSubsystem;
import frc.robot.subsystems.VerticalConveyorSubsystem;

import frc.robot.subsystems.AprilTagsSubsystem.Pipeline;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  // The Robot's subsystems are defined here
  private final AprilTagsSubsystem m_AprilTagsSubsystem = new AprilTagsSubsystem();
  private final IntakeConveyorSubsystem m_IntakeConveyorSubsystem = new IntakeConveyorSubsystem();
  private final VerticalConveyorSubsystem m_VerticalConveyorSubsystem = new VerticalConveyorSubsystem();

  // The Robot's commands are defined here
  private final IdentifyAprilTagCommand identifyAprilTagCommand = new IdentifyAprilTagCommand();

  // IR Break Beam Sensor Object Defined here (May be removed)
  // private final DigitalInput PayloadBeamSensor = new DigitalInput(ConveyorIntakeConstants.kRollerIRBeamPort);

  // Replace with CommandPS4Controller or CommandJoystick if needed (and port)
  private final GenericHID secondaryJoystick = new GenericHID(JoystickConstants.kPrimaryPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
    new JoystickButton(secondaryJoystick, JoystickConstants.kAButtonPort)
      .onTrue(
        new VerticalConveyorCommand(m_VerticalConveyorSubsystem)
      );

    new JoystickButton(secondaryJoystick, JoystickConstants.kXButtonPort)
      .onTrue(
        SensorTestSystem()
      ); 
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto();
  // }

  // Placeholder Methods for use in multiple classes

  /*
   * Conditional Command Group that controls the intake or conveyor in phases, depending on what breakbeam sensor is broken.
   * If the Intake beam isn't broken, run the Intake Conveyor Command. 
   * If the Intake beam is broken, run the Vertical Conveyor Command until the Vertical Conveyor beam is broken.
   * Conditional Command should restart over again once the note passes both sensors.
   */
  public Command SensorTestSystem() {
    return 
    new ConditionalCommand(
      new IntakeConveyorCommand(m_IntakeConveyorSubsystem), 
      new VerticalConveyorCommand(m_VerticalConveyorSubsystem).until(m_VerticalConveyorSubsystem::isVerticalBeamBroken), 
      m_IntakeConveyorSubsystem::isIntakeBeamBroken);
  }

  /*
   * Mockup of the final program's payload command sequence.
   */
  public Command FullPayloadsystem() {
    return 
    new SequentialCommandGroup(
      new ParallelCommandGroup(
        new IntakeConveyorCommand(m_IntakeConveyorSubsystem),
        new VerticalConveyorCommand(m_VerticalConveyorSubsystem)
        )
    );
  }
}
