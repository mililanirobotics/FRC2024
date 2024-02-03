package frc.robot.commands.VisiontrackingCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagsSubsystem;
import frc.robot.RobotContainer;

/*
 * Filler Command for the detection of the apriltag
 */
public class IdentifyAprilTagCommand extends Command{
    // Declaring Subsystems
    private AprilTagsSubsystem m_AprilTagsSubsystem;



    public IdentifyAprilTagCommand(AprilTagsSubsystem aprilTagsSubsystem) {
        m_AprilTagsSubsystem = aprilTagsSubsystem;
    }

    @Override
    public boolean isFinished() {
        return m_AprilTagsSubsystem.isTargetFound();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Target is found lmao");
    }
}
