package frc.robot.commands.ManualControls;

import frc.robot.Constants.SwivelConstants;
import frc.robot.subsystems.SwivelSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SetSwivelToAmpCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private SwivelSubsystem m_swivelSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetSwivelToAmpCommand(SwivelSubsystem swivelSubsystem) {
    m_swivelSubsystem = swivelSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swivelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Setting servo to amp view");
    m_swivelSubsystem.setAngle(SwivelConstants.kAmpView);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Servo in amp view");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_swivelSubsystem.getAngle() == SwivelConstants.kAmpView;
  }
}