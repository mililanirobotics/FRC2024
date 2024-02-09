package frc.robot.commands.LEDS;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LEDSubsystem.animations;

public class Signal2Command extends Command {
    private final LEDSubsystem m_LedSubsystem;

    public Signal2Command(LEDSubsystem m_LedSubsystem) {
        this.m_LedSubsystem = m_LedSubsystem;
        addRequirements(m_LedSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_LedSubsystem.clear();
        m_LedSubsystem
            .setOffset(8)
            .setColor(new Color(160, 32, 240))
            .setAnimation(animations.FIRE_ANIM);
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
