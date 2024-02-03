package frc.robot.commands.TestCommands;

import frc.robot.subsystems.IntakeRollerSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//general imports
import frc.robot.Constants.JoystickConstants;

public class TestManualIntakeCommand extends Command {
    //declaring subsystems
    private IntakeRollerSubsystem m_conveyorSubsystem;

    //declaring the joystick used
    private GenericHID joystick;

    //speed of the conveyor
    private double conveyorSpeed;

    //constructor
    public TestManualIntakeCommand(GenericHID joystick, IntakeRollerSubsystem conveyorSubsystem) {
        this.joystick = joystick;
        m_conveyorSubsystem = conveyorSubsystem;
        conveyorSpeed = 0;

        addRequirements(m_conveyorSubsystem);
    }
    
    @Override
    public void execute() { 
        //sets the conveyor speed based on the secondary gamepad's left joystick value
        if(joystick.getRawButtonPressed(JoystickConstants.kAButtonPort)     
        ) {
            conveyorSpeed += 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBButtonPort)
        ) {
            conveyorSpeed += 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kXButtonPort)) {
            conveyorSpeed -= 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kYButtonPort)) {
            conveyorSpeed -= 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBackButtonPort)) {
            conveyorSpeed = 0;
        }

        //sets the speeds
        if(joystick.getRawButtonPressed(JoystickConstants.kStartButtonPort)) {
            m_conveyorSubsystem.setConveyorSpeed(conveyorSpeed);
        }
        
        // conveyorSpeed = joystick.getRawAxis(JoystickConstants.kRightYJoystickPort);
        // m_conveyorSubsystem.setConveyorSpeed(conveyorSpeed);f

        SmartDashboard.putNumber("Conveyor Speed", conveyorSpeed);
        SmartDashboard.updateValues();
    }

    @Override
    public void end(boolean interrupted) {
        m_conveyorSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        return false;
    }
}