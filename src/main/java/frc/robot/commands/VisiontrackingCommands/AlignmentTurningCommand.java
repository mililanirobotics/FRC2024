package frc.robot.commands.VisiontrackingCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.AprilTagsSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AlignmentTurningCommand extends Command{
    
    // Declared Subsystems
    private SwerveDriveSubsystem m_SwerveDriveSubsystem;
    private AprilTagsSubsystem m_AprilTagsSubsystem;

    // Declared PID
    private PIDController AlignPID;

    private double initialAngle;
    public double currentAngle;

    public AlignmentTurningCommand(SwerveDriveSubsystem swerveDriveSubsystem, AprilTagsSubsystem aprilTagsSubsystem) {
        m_SwerveDriveSubsystem = swerveDriveSubsystem;
        m_AprilTagsSubsystem = aprilTagsSubsystem;

        AlignPID = new PIDController(
            AutoConstants.kPController, AutoConstants.kIController, AutoConstants.kDController
        );
        AlignPID.enableContinuousInput(-180, 180);
        AlignPID.setTolerance(AutoConstants.kTolerance);

        addRequirements(m_SwerveDriveSubsystem, m_AprilTagsSubsystem);
    }

    @Override
    public void initialize() {
        initialAngle = Math.toRadians(m_AprilTagsSubsystem.getHorizontalOffset());
        System.out.println("Initial Angle: "+initialAngle);
    }

    /*
     * Method should use limelight to track down the camera, and turn the robot until it is aligned with the Apriltag (Alignment position needs to be determined, but will end up having center of camera oriented on the apriltag on alignment)
     */
    @Override
    public void execute () {
        currentAngle = Math.toRadians(m_AprilTagsSubsystem.getHorizontalOffset());

        
        ChassisSpeeds targetSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, AlignPID.calculate(currentAngle, 0), new Rotation2d(0));

        SwerveModuleState[] moduleStates = SwerveModuleConstants.kinematics.toSwerveModuleStates(targetSpeed);
        m_SwerveDriveSubsystem.setModuleStates(moduleStates);
    }


    @Override
    public void end(boolean interrupted) {
        m_SwerveDriveSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(currentAngle - initialAngle) < 0.5;
    }
}
