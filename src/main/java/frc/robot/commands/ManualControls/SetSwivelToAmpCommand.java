package frc.robot.commands.ManualControls;

//subsystems
import frc.robot.subsystems.SwivelSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;
//constants
import frc.robot.Constants.SwivelConstants;

/**
 * Sets the angle/view of the servo to the amp 
 */
public class SetSwivelToAmpCommand extends Command {
  //declaring subsystems
  private SwivelSubsystem m_swivelSubsystem;

  //constructor
  public SetSwivelToAmpCommand(SwivelSubsystem swivelSubsystem) {
    //initializing subsystems 
    m_swivelSubsystem = swivelSubsystem;
    addRequirements(m_swivelSubsystem);
  }

  @Override
  public void initialize() {
    //printing an initialize statement 
    System.out.println("Setting servo to amp view");
    //setting the angle of the servo
    m_swivelSubsystem.setAngle(SwivelConstants.kAmpView);
  }

  @Override
  public void end(boolean interrupted) {
    //printing an update
    System.out.println("Servo in amp view");
  }

  @Override
  public boolean isFinished() {
    //ends the command once the servo's angle matches that of the constant
    return m_swivelSubsystem.getAngle() == SwivelConstants.kAmpView;
  }
}