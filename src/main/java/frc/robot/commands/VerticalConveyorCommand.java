package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VerticalConveyorConstants;
import frc.robot.subsystems.VerticalConveyorSubsystem;

public class VerticalConveyorCommand extends Command {
    // Declaring subsystems
    public VerticalConveyorSubsystem m_VerticalConveyorSubsystem;

    public VerticalConveyorCommand(VerticalConveyorSubsystem verticalConveyorSubsystem) {
        m_VerticalConveyorSubsystem = verticalConveyorSubsystem;

        addRequirements(m_VerticalConveyorSubsystem);
    }

    @Override 
    public void execute() {
        m_VerticalConveyorSubsystem.setVerticalSpeed(VerticalConveyorConstants.kConveyorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_VerticalConveyorSubsystem.shutdown();
    }

    @Override
    public boolean isFinished() {
        return m_VerticalConveyorSubsystem.isVerticalBeamBroken();
    } 
}
