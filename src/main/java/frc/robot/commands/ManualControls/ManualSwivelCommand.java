package frc.robot.commands.ManualControls;

import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.SwivelSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ManualSwivelCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private SwivelSubsystem m_subsystem;
  private GenericHID gamepad;

  private int angle;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualSwivelCommand(SwivelSubsystem subsystem, GenericHID gamepad) {
    m_subsystem = subsystem;
    this.gamepad = gamepad;
    angle = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_subsystem.setSpeed(gamepad.getRawAxis(1));

    if (gamepad.getRawButtonPressed(1)) {
        angle += 5;
    }

    if (gamepad.getRawButtonPressed(2)) {
      angle += 10;
    }

    if (gamepad.getRawButtonPressed(3)) {
        angle -= 5;
    }

    if (gamepad.getRawButtonPressed(4)) {
        angle -= 10;
    }

    System.out.println("Angle: "+angle);
    if(gamepad.getRawButtonPressed(JoystickConstants.kRightBumperPort)) {
      System.out.println("Turning");
        m_subsystem.setAngle(angle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}