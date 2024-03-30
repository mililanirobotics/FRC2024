// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//subsystems
import frc.robot.subsystems.AprilTagsSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeConveyorSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ScoringSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwivelSubsystem;
import frc.robot.commands.LEDS.AlignedSignalCommand;
import frc.robot.commands.LowerExtensionCommand;
import frc.robot.commands.RaiseExtensionCommand;
import frc.robot.commands.AutomationCommands.AutoIntakeConveyorCommand;
import frc.robot.commands.LEDS.ExtendedSignalCommand;
import frc.robot.commands.LEDS.HighNoteSignalCommand;
import frc.robot.commands.LEDS.LowNoteSignalCommand;
import frc.robot.commands.AutomationCommands.AutoScoringCommand;
import frc.robot.commands.AutomationCommands.NoteToScorerCommand;
import frc.robot.commands.ManualControls.ManualIntakeConveyorCommand;
import frc.robot.commands.ManualControls.ManualScoringCommand;
import frc.robot.commands.ManualControls.ManualScoringReverseCommand;
import frc.robot.commands.ManualControls.ManualSwivelCommand;
import frc.robot.commands.ManualControls.SetSwivelToAmpCommand;
import frc.robot.commands.ManualControls.SetSwivelToDriverViewCommand;
import frc.robot.commands.ManualControls.SetSwivelToMiddleCommand;
import frc.robot.commands.ManualControls.SwerveControlGamepadCommand;
import frc.robot.commands.ManualControls.SwerveControlJoystickCommand;
import frc.robot.commands.TestCommands.TestManualIntakeConveyorCommand;
import frc.robot.commands.TestCommands.TestManualScoringCommand;
import frc.robot.commands.VisionCommands.AlignmentTranslationalCommand;
import frc.robot.commands.VisionCommands.AlignmentTurningCommand;
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
import java.util.function.BooleanSupplier;

import org.opencv.photo.AlignExposures;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AprilTagConstants;
//constants
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.ScoringConstants;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //shuffleboard tabs
  private final ShuffleboardTab testTranPos = Shuffleboard.getTab("Test_Tran_Pos");
  private final ShuffleboardTab testTranVel = Shuffleboard.getTab("Test_Tran_Vel");
  private final ShuffleboardTab testRotPos = Shuffleboard.getTab("Test_Rot_Pos");
  private final ShuffleboardTab testRotVel = Shuffleboard.getTab("Test_Rot_Vel");
  private final ShuffleboardTab testPos = Shuffleboard.getTab("Test_Pos");
  private final ShuffleboardTab testGyroData = Shuffleboard.getTab("Test_Gyro_Data");

  //initializing subsystems
  private final AprilTagsSubsystem aprilTagsSubsystem = new AprilTagsSubsystem();
  private final IntakeConveyorSubsystem intakeConveyorSubsystem = new IntakeConveyorSubsystem();
  private final ScoringSubsystem scoringSubsytem = new ScoringSubsystem();
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(
    testTranPos,
    testTranVel,
    testRotPos,
    testRotVel,
    testPos,
    testGyroData
  );

  private final ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final SwivelSubsystem swivelSubsystem = new SwivelSubsystem();

  //initializing sendable chooser for auto
  private SendableChooser<Command> autoCommand;
  private boolean isTesting = false; //purely for testing purposes

  //initailizing gamepads
  private final GenericHID primaryGamepad = new GenericHID(JoystickConstants.kPrimaryGamepadPort);
  private final GenericHID secondaryGamepad = new GenericHID(JoystickConstants.kSecondaryGamepadPort);
  private final GenericHID test = new GenericHID(2);

  // private final GenericHID priamryJoystickL = new GenericHID(JoystickConstants.kPrimaryGamepadPort);
  // private final GenericHID priamryJoystickR = new GenericHID(JoystickConstants.kSecondaryGamepadPort);
  // private final GenericHID secondaryGamepad = new GenericHID(2);

  private final Field2d field = new Field2d();

  private static boolean inRobot;
  private static boolean inScorer;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    inRobot = false;
    inScorer = false;

    if(isTesting) {
      //setting default commands for testing
      // intakeConveyorSubsystem.setDefaultCommand(new TestManualIntakeConveyorCommand(secondaryGamepad, intakeConveyorSubsystem));
      scoringSubsytem.setDefaultCommand(new TestManualScoringCommand(secondaryGamepad, scoringSubsytem));
    }

    //setting the default command for the swerve drive
    // swerveDriveSubsystem.setDefaultCommand(new SwerveControlJoystickCommand(
    //     swerveDriveSubsystem, 
    //     priamryJoystickL,
    //     priamryJoystickR
    //   )
    // );

    swerveDriveSubsystem.setDefaultCommand(new SwerveControlGamepadCommand(
        swerveDriveSubsystem, 
        primaryGamepad
      )
    );

    // ledSubsystem.setDefaultCommand(
    //   new ConditionalCommand(
    //     new ConditionalCommand(
    //       new AlignedSignalCommand(ledSubsystem), 
    //       new ExtendedSignalCommand(ledSubsystem), 
    //       aprilTagsSubsystem::isAmpAligned), 
    //     new ConditionalCommand(
    //       new HighNoteSignalCommand(ledSubsystem), 
    //       new ConditionalCommand(
    //         new LowNoteSignalCommand(ledSubsystem), 
    //         new InstantCommand(this::teleopLEDs, ledSubsystem), 
    //         RobotContainer.isInRobot()
    //       ),  
    //       RobotContainer.isInScorer()
    //     ), 
    //     extensionSubsystem::isExtended
    //   )
    // );
    
    //reigstered paths on Pathplanner
    // NamedCommands.registerCommand("AutoIntakeConveyorCommand", new AutoIntakeConveyorCommand(intakeConveyorSubsystem, 1));
    // NamedCommands.registerCommand("maker2", Commands.print("Passed marker 1"));

    //initializing auto chooser in SmartDashboard
    // autoCommand = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Path", autoCommand);

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
    // autoCommand.addOption("Ur Mom", new PathPlannerAuto("Ur Mom"));
    // autoCommand.addOption("Ur Mom X2", onTheFlyCommand(new Pose2d(0, 0, new Rotation2d()), new Pose2d(0, 0, Rotation2d.fromDegrees(90))));

    //=========================================================================== 
    // primary controls
    //===========================================================================
    new JoystickButton(primaryGamepad, JoystickConstants.kRightBumperPort).onTrue(
      new InstantCommand(() -> intakeConveyorSubsystem.setNoteIn(false), intakeConveyorSubsystem)
    );

    //=========================================================================== 
    // secondary controls
    //===========================================================================

    //starts the auto-scoring command if the A button is pressed; stops if one of the conditions is met
    new JoystickButton(secondaryGamepad, JoystickConstants.kAButtonPort).onTrue(
      new RaiseExtensionCommand(extensionSubsystem).andThen(
        new WaitCommand(0.5)
      ).andThen(
        new AutoScoringCommand(intakeConveyorSubsystem, scoringSubsytem).onlyIf(
          extensionSubsystem::isExtended
        )
      )/*.onlyIf(
        aprilTagsSubsystem::isAmpAligned
      )*/.until(
        () -> secondaryGamepad.getRawButtonPressed(JoystickConstants.kRightBumperPort)
                || secondaryGamepad.getRawButtonPressed(JoystickConstants.kXButtonPort)
                || secondaryGamepad.getRawButtonPressed(JoystickConstants.kBButtonPort)
      ).andThen(
        new LowerExtensionCommand(extensionSubsystem)
      )
    );

    new JoystickButton(primaryGamepad, JoystickConstants.kLeftBumperPort).onTrue(
      new AlignmentTurningCommand(swerveDriveSubsystem, aprilTagsSubsystem)
    );

    new Trigger(
      () -> primaryGamepad.getRawAxis(JoystickConstants.kLeftTriggerPort) >= 0.5
    ).onTrue(
      new AlignmentTranslationalCommand(swerveDriveSubsystem, aprilTagsSubsystem)
    );

    //manually controls the scoring payload with the A and B button (A normal, B reverse)
    new JoystickButton(secondaryGamepad, JoystickConstants.kBButtonPort).onTrue(
      new ManualScoringCommand(secondaryGamepad, scoringSubsytem)
    ); 

    //manually controls the intake and conveyor with the left Y-joystick
    new Trigger(
      () -> Math.abs(secondaryGamepad.getRawAxis(JoystickConstants.kLeftYJoystickPort)) >= JoystickConstants.kDeadzone
    ).onTrue(
      new ManualIntakeConveyorCommand(secondaryGamepad, intakeConveyorSubsystem)
    );

    //manually controls the extensions with the left bumper and trigger
    new Trigger(
      () -> secondaryGamepad.getRawAxis(JoystickConstants.kRightTriggerPort) >= 0.5
    ).onTrue(
      new RaiseExtensionCommand(extensionSubsystem)
    );
    new Trigger(
      () -> secondaryGamepad.getRawAxis(JoystickConstants.kLeftTriggerPort) >= 0.5
    ).onTrue(
      new LowerExtensionCommand(extensionSubsystem)
    );

    //sets the servo to the driver and amp view
    new Trigger(
      () -> secondaryGamepad.getRawAxis(JoystickConstants.kRightYJoystickPort) <= -0.5
    ).onTrue(
      new SetSwivelToAmpCommand(swivelSubsystem).alongWith(
        new InstantCommand(() -> aprilTagsSubsystem.setPipeline(AprilTagConstants.kAmpPipeline), aprilTagsSubsystem)
      )
    );

    new Trigger(
      () -> secondaryGamepad.getRawAxis(JoystickConstants.kRightYJoystickPort) >= 0.5
    ).onTrue(
      new SetSwivelToDriverViewCommand(swivelSubsystem).alongWith(
        new InstantCommand(() -> aprilTagsSubsystem.setPipeline(AprilTagConstants.kDriverPipeline), aprilTagsSubsystem)
      )
    );

    new JoystickButton(secondaryGamepad, JoystickConstants.kLeftBumperPort).onTrue(
      new SetSwivelToMiddleCommand(swivelSubsystem).alongWith(
        new InstantCommand(() -> aprilTagsSubsystem.setPipeline(AprilTagConstants.kDriverPipeline), aprilTagsSubsystem)
      )
    );

    // new JoystickButton(secondaryJoystick, JoystickConstants.kAButtonPort).whileTrue(
    //   new HighNoteSignalCommand(ledSubsystem)
    // );
    // new JoystickButton(secondaryJoystick, JoystickConstants.kYButtonPort).whileTrue(
    //   new ExtendedSignalCommand(ledSubsystem)
    // );
    // new JoystickButton(secondaryJoystick, JoystickConstants.kXButtonPort).whileTrue(
    //   new LowNoteSignalCommand(ledSubsystem)
    // );
    // new JoystickButton(secondaryJoystick, JoystickConstants.kBButtonPort).whileTrue(
    //   new MidNoteSignalCommand(ledSubsystem)
    // );

    //=========================================================================== 
    // sensor triggers
    //===========================================================================

    
  

    //trigger that schedules the AutoIntakeCommand once the bottom IR sensor is triggered
    // new Trigger(
    //   intakeConveyorSubsystem::getStartSensorReading
    // ).onTrue(
    //   new SequentialCommandGroup(
    //     new LowerExtensionCommand(extensionSubsystem),
    //     new AutoIntakeConveyorCommand(intakeConveyorSubsystem).until(
    //     () -> secondaryJoystick.getRawAxis(JoystickConstants.kLeftYJoystickPort) >= JoystickConstants.kDeadzone
    //             || secondaryJoystick.getRawButton(JoystickConstants.kRightBumperPort)
    //     ),
    //     new RaiseExtensionCommand(extensionSubsystem)
    //   )
    // );

    //maybe works??
    new Trigger(
      intakeConveyorSubsystem::getStartSensorReadingReverse
    ).onTrue(
      new AutoIntakeConveyorCommand(intakeConveyorSubsystem)
      .alongWith(
        new NoteToScorerCommand(scoringSubsytem)
      )
      .unless(
        intakeConveyorSubsystem::isNoteIn
      )
      .until(
        () -> secondaryGamepad.getRawAxis(JoystickConstants.kLeftYJoystickPort) >= JoystickConstants.kDeadzone
                || secondaryGamepad.getRawButton(JoystickConstants.kRightBumperPort)
      )
    );

    // //maybe works???
    // new Trigger(
    //   intakeConveyorSubsystem::getStartSensorReadingReverse
    // ).onTrue(
    //   new AutoIntakeConveyorCommand(intakeConveyorSubsystem, scoringSubsytem).unless(intakeConveyorSubsystem::isNoteIn)
    // );
  }

  // private Command followPathCommand(String pathName) {
  //   PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

  //   return new FollowPathHolonomic(
  //       path,
  //       swerveDriveSubsystem::getPose,
  //       swerveDriveSubsystem::getSpeeds,
  //       swerveDriveSubsystem::driveRobotRelative,
  //       AutoConstants.pathFollowingConfig,
  //       () -> {
  //           var alliance = DriverStation.getAlliance();
  //           if(alliance.isPresent()) { 
  //               return alliance.get() == DriverStation.Alliance.Red;
  //           }
  //           return false;
  //       },
  //       swerveDriveSubsystem
  //     );
  // }

  // private Command onTheFlyCommand(Pose2d startPosition, Pose2d endPosition) {
  //   return Commands.runOnce(
  //     () -> {
  //       //gets the current position of the drive
  //       Pose2d currentPosition = swerveDriveSubsystem.getPose();

  //       List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPosition, endPosition); 
  //       PathPlannerPath path = new PathPlannerPath (
  //         bezierPoints,
  //         AutoConstants.pathConstraints,
  //         new GoalEndState(0, currentPosition.getRotation())  
  //       );

  //       path.preventFlipping = true;
  //       AutoBuilder.followPath(path).schedule();
  //     }
  //   );
  // }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoCommand.getSelected();
  }

  //WPLIB test
  public Command swerveCommand() {
    //creating the trajectory config
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig( 
      AutoConstants.kAutoDriveMaxMetersPerSecond,
      AutoConstants.kAutoDriveMaxAcceleration
    )
    .setKinematics(SwerveModuleConstants.kinematics);

    //generating trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)), 
      List.of(
        new Translation2d(1, 0),
        new Translation2d(1, -1)
      ),
      new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
      trajectoryConfig
    );

    //creating PID controllers
    PIDController XController = new PIDController(
      AutoConstants.kPXController, 
      AutoConstants.kIXController, 
      AutoConstants.kDXController
    );

    PIDController yController = new PIDController(
      AutoConstants.kPYController, 
      AutoConstants.kIYController, 
      AutoConstants.kDYController
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 
      AutoConstants.kIThetaController, 
      AutoConstants.kDThetaController, 
      new TrapezoidProfile.Constraints(
        AutoConstants.kAutoDriveMaxRadiansPerSecond, 
        AutoConstants.kAutoDriveMaxAngularAcceleration
      )
    );

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //constructing command
    SwerveControllerCommand swerveCommand = new SwerveControllerCommand(
      trajectory, 
      swerveDriveSubsystem::getPose,
      SwerveModuleConstants.kinematics,
      XController,
      yController,
      thetaController,
      swerveDriveSubsystem::setModuleStates,
      swerveDriveSubsystem
    );

    //returning the command
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveDriveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveCommand,
      new InstantCommand(() -> swerveDriveSubsystem.shutdown())
    );
  }

  public void disabledLEDs() {
    ledSubsystem.disabled();
  }

  public void teleopLEDs() {
    ledSubsystem.teleoperation();
  }

  public void autoLEDs() {
    ledSubsystem.autonomous();
  }

  /**
   * Returns whether the note has transitioned into the scorer
   * @return If the note is in the scorer or not
   */
  // public static BooleanSupplier isInScorer() {
  //   return new BooleanSupplier() {
  //     inScorer;
  //   };
  // }

  // /**
  //  * Updates the value of the boolean inScorer based on the input
  //  * @param inScorer Whether the note is in the scorer
  //  */
  // public static void setInScorer(boolean isInScorer) {
  //   inScorer = isInScorer;
  // }

  // public static BooleanSupplier isInRobot() {
  //   return inRobot;
  // }

  // public static void setInRobot(boolean isInRobot) {
  //   inRobot = isInRobot;
  // }

}
  