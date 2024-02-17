// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//subsystems
import frc.robot.subsystems.AprilTagsSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeConveyorSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.LowerExtensionCommand;
import frc.robot.commands.RaiseExtensionCommand;
// import frc.robot.commands.AutomationCommands.AutoScoringCommand;
import frc.robot.commands.ManualControls.ManualIntakeConveyorCommand;
import frc.robot.commands.ManualControls.ManualScoringCommand;
import frc.robot.commands.ManualControls.ManualScoringReverseCommand;
import frc.robot.commands.ManualControls.SwerveControlCommand;
import frc.robot.commands.TestCommands.TestManualIntakeConveyorCommand;
import frc.robot.commands.TestCommands.TestManualScoringCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//path planner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

//general imports
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//constants
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.JoystickConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //initializing subsystems
  private final AprilTagsSubsystem aprilTagsSubsystem = new AprilTagsSubsystem();
  private final IntakeConveyorSubsystem intakeConveyorSubsystem = new IntakeConveyorSubsystem();
  private final ScoringSubsystem scoringSubsytem = new ScoringSubsystem();
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem();
  private final ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem();

  //initailizing gamepads
  private final GenericHID priamryJoystick = new GenericHID(JoystickConstants.kPrimaryGamepadPort);
  private final GenericHID secondaryJoystick = new GenericHID(JoystickConstants.kSecondaryGamepadPort);

  //initializing sendable chooser for auto
  private SendableChooser<Command> autoCommand;
  private boolean isTesting = false; //purely for testing purposes

  private final Field2d field = new Field2d();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(isTesting) {
      //setting default commands for testing
      intakeConveyorSubsystem.setDefaultCommand(new TestManualIntakeConveyorCommand(secondaryJoystick, intakeConveyorSubsystem));
      scoringSubsytem.setDefaultCommand(new TestManualScoringCommand(secondaryJoystick, scoringSubsytem));
    }

    //setting the default command for the swerve drive
    swerveDriveSubsystem.setDefaultCommand(new SwerveControlCommand(
        swerveDriveSubsystem, 
        priamryJoystick
      )
    );
    
    //reigstered paths on Pathplanner
    // NamedCommands.registerCommand("AutoIntakeConveyorCommand", new AutoIntakeConveyorCommand(intakeConveyorSubsystem, 1));
    NamedCommands.registerCommand("maker2", Commands.print("Passed marker 1"));

    //initializing auto chooser in SmartDashboard
    autoCommand = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Path", autoCommand);

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
    autoCommand.addOption("Ur Mom", new PathPlannerAuto("Ur Mom"));
    autoCommand.addOption("Ur Mom X2", onTheFlyCommand(new Pose2d(0, 0, new Rotation2d()), new Pose2d(0, 0, Rotation2d.fromDegrees(90))));

    // //starts the auto-scoring command if the A button is pressed; stops if one of the conditions is met
    // new JoystickButton(secondaryJoystick, JoystickConstants.kAButtonPort).onTrue(
    //   new AutoScoringCommand(scoringSubsytem).until(
    //     () -> secondaryJoystick.getRawButton(JoystickConstants.kRightBumperPort)
    //             || secondaryJoystick.getRawButton(JoystickConstants.kXButtonPort)
    //             || secondaryJoystick.getRawButton(JoystickConstants.kBButtonPort)
    //   )
    // );

    //manually controls the scoring payload with the A and B button (A normal, B reverse)
    new JoystickButton(secondaryJoystick, JoystickConstants.kBButtonPort).whileTrue(
      new ManualScoringCommand(secondaryJoystick, scoringSubsytem)
    );
    new JoystickButton(secondaryJoystick, JoystickConstants.kXButtonPort).whileTrue(
      new ManualScoringReverseCommand(secondaryJoystick, scoringSubsytem)
    );
    
    //manually controls the intake and conveyor with the left Y-joystick
    new Trigger(
      () -> Math.abs(secondaryJoystick.getRawAxis(JoystickConstants.kLeftYJoystickPort)) >= JoystickConstants.kDeadzone
    ).onTrue(
      new ManualIntakeConveyorCommand(secondaryJoystick, intakeConveyorSubsystem)
    );
    
    //manually controls the extensions with the left bumper and trigger
    new JoystickButton(secondaryJoystick, JoystickConstants.kLeftBumperPort).onTrue(
      new RaiseExtensionCommand(extensionSubsystem)
    );
    new Trigger(
      () -> secondaryJoystick.getRawAxis(JoystickConstants.kLeftTriggerPort) >= 0.5
    ).onTrue(
      new LowerExtensionCommand(extensionSubsystem)
    );

    //trigger that schedules the AutoIntakeCommand once the bottom IR sensor is triggered
    // new Trigger(
    //   intakeConveyorSubsystem::getStartSensorReading
    // ).onTrue(
    //   new SequentialCommandGroup(
    //     new RaiseExtensionCommand(extensionSubsystem),
    //     new AutoIntakeConveyorCommand(intakeConveyorSubsystem, 1).until(
    //     () -> secondaryJoystick.getRawAxis(JoystickConstants.kLeftYJoystickPort) >= JoystickConstants.kDeadzone
    //             || secondaryJoystick.getRawButton(JoystickConstants.kRightBumperPort)
    //     ),
    //     new LowerExtensionCommand(extensionSubsystem)
    //   )
    // );
  }

  private Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    return new FollowPathHolonomic(
        path,
        swerveDriveSubsystem::getPose,
        swerveDriveSubsystem::getSpeeds,
        swerveDriveSubsystem::driveRobotRelative,
        AutoConstants.pathFollowingConfig,
        () -> {
            var alliance = DriverStation.getAlliance();
            if(alliance.isPresent()) { 
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        swerveDriveSubsystem
      );
  }

  private Command onTheFlyCommand(Pose2d startPosition, Pose2d endPosition) {
    return Commands.runOnce(
      () -> {
        //gets the current position of the drive
        Pose2d currentPosition = swerveDriveSubsystem.getPose();

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPosition, endPosition); 
        PathPlannerPath path = new PathPlannerPath (
          bezierPoints,
          AutoConstants.pathConstraints,
          new GoalEndState(0, currentPosition.getRotation())  
        );

        path.preventFlipping = true;
        AutoBuilder.followPath(path).schedule();
      }
    );
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoCommand.getSelected();
  }

}
  