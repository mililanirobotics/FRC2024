package frc.robot.commands.LEDS;

import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.animations;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public class BaseSignalCommand extends Command {
    private final LEDSubsystem m_LedSubsystem;

    public BaseSignalCommand(LEDSubsystem m_LedSubsystem) {
        this.m_LedSubsystem = m_LedSubsystem;
        addRequirements(m_LedSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_LedSubsystem.clear();
        m_LedSubsystem
            .setColor(new Color(255, 165, 0))
            .setAnimation(animations.SET_ALL);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
