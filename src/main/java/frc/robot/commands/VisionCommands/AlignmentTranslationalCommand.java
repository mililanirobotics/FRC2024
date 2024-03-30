package frc.robot.commands.VisionCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.AprilTagsSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AlignmentTranslationalCommand extends Command{
    
    // Declared Subsystems
    private SwerveDriveSubsystem m_SwerveDriveSubsystem;
    private AprilTagsSubsystem m_AprilTagsSubsystem;

    // Declared PID
    private PIDController translationalPID;

    private double translationalSpeed;
    public double currentAngle;

    public AlignmentTranslationalCommand(SwerveDriveSubsystem swerveDriveSubsystem, AprilTagsSubsystem aprilTagsSubsystem) {
        m_SwerveDriveSubsystem = swerveDriveSubsystem;
        m_AprilTagsSubsystem = aprilTagsSubsystem;

        translationalPID = new PIDController(
            AprilTagConstants.kPValue, AprilTagConstants.kIValue, AprilTagConstants.kDValue
        );
        
        translationalPID.enableContinuousInput(-Math.PI, Math.PI);
        translationalPID.setTolerance(AprilTagConstants.kTolerance);

        addRequirements(m_SwerveDriveSubsystem, m_AprilTagsSubsystem);
    }

    /*
     * Method should use limelight to track down the camera, and turn the robot until it is aligned with the Apriltag (Alignment position needs to be determined, but will end up having center of camera oriented on the apriltag on alignment)
     */
    @Override
    public void execute () {
        currentAngle = Math.toRadians(m_AprilTagsSubsystem.getHorizontalOffset());

        translationalSpeed = translationalPID.calculate(currentAngle, 0) * (-3);

        /*
         * Minimal speed buffer for turning
         */
        if (Math.abs(translationalSpeed) < 0.1) {
            translationalSpeed = Math.copySign(0.1, translationalSpeed);
        }

        ChassisSpeeds targetSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(0, translationalSpeed, 0, Rotation2d.fromDegrees(0));

        SwerveModuleState[] moduleStates = SwerveModuleConstants.kinematics.toSwerveModuleStates(targetSpeed);
        m_SwerveDriveSubsystem.setModuleStates(moduleStates);

        System.out.println("Current Angle: " +currentAngle);
    }


    @Override
    public void end(boolean interrupted) {
        System.out.println("Aligned");
        m_SwerveDriveSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(currentAngle) <= AprilTagConstants.kTolerance;
    }
}