package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.HangSubsystem;
import frc.robot.subsystems.IntakeConveyorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.AprilTagsSubsystem;


public class RobotStatus implements Sendable {
    private HangSubsystem m_HangSubsystem;
    private IntakeConveyorSubsystem m_IntakeConveyorSubsystem;
    private ExtensionSubsystem m_ExtensionSubsystem;
    private AprilTagsSubsystem m_AprilTagsSubsystem;


    public RobotStatus(HangSubsystem m_HangSubsystem, IntakeConveyorSubsystem m_IntakeConveyorSubsystem,
     ExtensionSubsystem m_ExtensionSubsystem, AprilTagsSubsystem m_AprilTagsSubsystem) {
        this.m_HangSubsystem = m_HangSubsystem;
        this.m_IntakeConveyorSubsystem = m_IntakeConveyorSubsystem;
        this.m_ExtensionSubsystem = m_ExtensionSubsystem;
        this.m_AprilTagsSubsystem = m_AprilTagsSubsystem;

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("RobotStatus");
        builder.addBooleanProperty("PayloadDeployed", () -> m_ExtensionSubsystem.isExtended(), null);
        builder.addBooleanProperty("HangDeployed", () -> m_HangSubsystem.getHangState(), null);
        builder.addBooleanProperty("IsAligned", () -> m_AprilTagsSubsystem.isAmpAligned(), null);
        builder.addIntegerProperty("NotePosition", () -> m_IntakeConveyorSubsystem.getNotePosition(), null);
        // builder.addBooleanProperty("PayloadDeployed", () -> false, null);
        // builder.addBooleanProperty("HangDeployed", () -> true, null);
        // builder.addBooleanProperty("IsAligned", () -> false, null);
        // builder.addIntegerProperty("NotePosition", () -> 0, null);
    }
}