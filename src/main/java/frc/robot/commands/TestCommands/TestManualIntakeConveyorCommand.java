package frc.robot.commands.TestCommands;

import frc.robot.subsystems.IntakeConveyorSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//general imports
import frc.robot.Constants.JoystickConstants;

public class TestManualIntakeConveyorCommand extends Command {
    //declaring subsystems
    private IntakeConveyorSubsystem m_intakeConveyorSubsystem;

    //declaring the joystick used
    private GenericHID joystick;

    //speed of the conveyor
    private double intakeConveyorSpeed;

    //constructor
    public TestManualIntakeConveyorCommand(GenericHID joystick, IntakeConveyorSubsystem intakeConveyorSubsystem) {
        this.joystick = joystick;
        m_intakeConveyorSubsystem = intakeConveyorSubsystem;
        intakeConveyorSpeed = 0;

        addRequirements(m_intakeConveyorSubsystem);
    }
    
    @Override
    public void execute() { 
        //sets the conveyor speed based on the secondary gamepad's left joystick value
        if(joystick.getRawButtonPressed(JoystickConstants.kAButtonPort)     
        ) {
            intakeConveyorSpeed += 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBButtonPort)
        ) {
            intakeConveyorSpeed += 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kXButtonPort)) {
            intakeConveyorSpeed -= 0.1;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kYButtonPort)) {
            intakeConveyorSpeed -= 0.05;
        }
        else if(joystick.getRawButtonPressed(JoystickConstants.kBackButtonPort)) {
            intakeConveyorSpeed = 0;
        }

        //sets the speeds
        if(joystick.getRawButtonPressed(JoystickConstants.kStartButtonPort)) {
            m_intakeConveyorSubsystem.setSpeeds(intakeConveyorSpeed);
        }

        //puts speeds on SmartDashboard
        SmartDashboard.putNumber("Conveyor and Intake Speed", intakeConveyorSpeed);
        SmartDashboard.updateValues();
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeConveyorSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        return false;
    }
}