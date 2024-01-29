package frc.robot.commands;

//subsystems and commands
import frc.robot.subsystems.RollerIntakeSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//general imports
import frc.robot.Constants.JoystickConstants;

public class TestManualIntakeCommand extends Command {
    //declaring subsystems
    private RollerIntakeSubsystem m_rollerSubsystem;

    //declaring the joystick used
    private GenericHID joystick;

    //declaring physical measurements 
    private double intakeSpeed;

    //constructor
    public TestManualIntakeCommand(GenericHID joystick, RollerIntakeSubsystem rollerIntakeSubsystem) {
        this.joystick = joystick;
        intakeSpeed = 0;

        //initializing subsystems
        m_rollerSubsystem = rollerIntakeSubsystem;
        addRequirements(m_rollerSubsystem);
    }
    
    @Override
    public void execute() { 
    //sets the conveyor speed based on the secondary gamepad's left joystick value
        if(joystick.getRawButtonPressed(JoystickConstants.kAButtonPort)) {
            intakeSpeed += 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBButtonPort)) {
            intakeSpeed += 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kXButtonPort)) {
            intakeSpeed -= 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kYButtonPort)) {
            intakeSpeed -= 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBackButtonPort)) {
            intakeSpeed = 0;
        }

        //sets the speeds
        if(joystick.getRawButtonPressed(JoystickConstants.kStartButtonPort)) {
            m_rollerSubsystem.setIntakeSpeed(intakeSpeed);
        }

        // intakeSpeed = joystick.getRawAxis(JoystickConstants.kLeftYJoystickPort);
        // m_rollerSubsystem.setIntakeSpeed(intakeSpeed);


        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
        SmartDashboard.updateValues();
    }

    @Override
    public void end(boolean interrupted) {
        m_rollerSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        return false;
    }
}