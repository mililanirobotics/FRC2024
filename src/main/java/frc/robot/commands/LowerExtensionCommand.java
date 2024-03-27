package frc.robot.commands;

import frc.robot.subsystems.ExtensionSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;

public class LowerExtensionCommand extends Command {
    //declaring subsystems
    private ExtensionSubsystem m_extensionSubsystem;

    //constructor
    public LowerExtensionCommand(ExtensionSubsystem extensionSubsystem) {
        m_extensionSubsystem = extensionSubsystem;
        addRequirements(m_extensionSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("Lower extension command started");
        m_extensionSubsystem.retract();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Lower extension command finished");
    }

    //in progress
    @Override
    public boolean isFinished() {
        return m_extensionSubsystem.getLeftState() == Value.kReverse && m_extensionSubsystem.getRightState() == Value.kReverse;
    }
}