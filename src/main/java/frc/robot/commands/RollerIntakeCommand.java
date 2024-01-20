package frc.robot.commands;

//subsystems and commands
import frc.robot.subsystems.RollerIntakeSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

//general imports
import frc.robot.Constants.JoystickConstants;

public class RollerIntakeCommand extends CommandBase {
    //declaring subsystems
    private RollerIntakeSubsystem m_rollerSubsystem;

    //declaring the joystick used
    private GenericHID joystick;

    //speed
    private double greenSpeed;
    private double blueSpeed;

    //constructor
    public RollerIntakeCommand(GenericHID joystick, RollerIntakeSubsystem rollerIntakeSubsystem) {
        this.joystick = joystick;
        m_rollerSubsystem = rollerIntakeSubsystem;
        blueSpeed = 0;
        greenSpeed = 0;

        addRequirements(m_rollerSubsystem);
    }
    
    @Override
    public void execute() { 
        if(joystick.getRawButtonPressed(JoystickConstants.kAButtonPort)) {
            greenSpeed += 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBButtonPort)) {
            greenSpeed -= 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kYButtonPort)) {
            blueSpeed += 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kXButtonPort)) {
            blueSpeed -= 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kRightBumperPort)) {
            blueSpeed = 0;
            greenSpeed = 0;
        }

        System.out.println(blueSpeed);
        SmartDashboard.putNumber("Green speed", greenSpeed);
        SmartDashboard.putNumber("Blue speed", blueSpeed);
        SmartDashboard.updateValues();

        m_rollerSubsystem.setIndependentSpeed(blueSpeed, greenSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        m_rollerSubsystem.setIndependentSpeed(0, 0);
    }

    //in progress
    @Override
    public boolean isFinished() {
        return false;
    }
}