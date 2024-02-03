package frc.robot.commands.VisiontrackingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagsSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoAlignmentCommand extends Command{
    
    // Declared Subsystems
    private SwerveDriveSubsystem m_SwerveDriveSubsystem;
    private AprilTagsSubsystem m_AprilTagsSubsystem;

    public AutoAlignmentCommand(SwerveDriveSubsystem swerveDriveSubsystem, AprilTagsSubsystem aprilTagsSubsystem) {
        m_SwerveDriveSubsystem = swerveDriveSubsystem;
        m_AprilTagsSubsystem = aprilTagsSubsystem;

        addRequirements(m_SwerveDriveSubsystem, m_AprilTagsSubsystem);
    }

    /*
     * Method should use limelight to track down the camera, and strafe the robot until it is aligned with the Apriltag (Alignment position needs to be determined, but will end up having center of camera oriented on the apriltag on alignment)
     */
    @Override
    public void execute () {
        if (m_AprilTagsSubsystem.getHorizontalOffset() >= 1) {
           
        }
    }

}
