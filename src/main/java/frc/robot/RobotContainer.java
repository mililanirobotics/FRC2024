// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//subsystems
import frc.robot.subsystems.SwerveDriveSubsystem;
//path planner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
//general imports
import java.util.List;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
//constants
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.SwerveControlCommand;

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
  private final SwerveDriveSubsystem swerveDriveSubsystem = new SwerveDriveSubsystem(
    testTranPos,
    testTranVel,
    testRotPos,
    testRotVel,
    testPos,
    testGyroData
  );

  //initailizing gamepads
  private final GenericHID priamryJoystick = new GenericHID(JoystickConstants.kPrimaryGamepadPort);

  //initializing sendable chooser for auto
  private SendableChooser<Command> autoCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //setting the default command for the swerve drive
    swerveDriveSubsystem.setDefaultCommand(
      // new StraightenSwerveCommand(swerveDriveSubsystem)
      // .andThen(
        new SwerveControlCommand(
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
    return new ProxyCommand(() -> autoCommand.getSelected());
  }

}
  