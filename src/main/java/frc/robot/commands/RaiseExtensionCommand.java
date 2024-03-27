package frc.robot.commands;

import frc.robot.subsystems.ExtensionSubsystem;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;

public class RaiseExtensionCommand extends Command {
    //declaring subsystems
    private ExtensionSubsystem m_extensionSubsystem;

    //constructor
    public RaiseExtensionCommand(ExtensionSubsystem extensionSubsystem) {
        m_extensionSubsystem = extensionSubsystem;
        addRequirements(m_extensionSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("Raise extension command started");
        m_extensionSubsystem.extend();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Raise extension command finished");
    }

    //in progress
    @Override
    public boolean isFinished() {
        return m_extensionSubsystem.getLeftState() == Value.kForward && m_extensionSubsystem.getRightState() == Value.kForward;
    }
}