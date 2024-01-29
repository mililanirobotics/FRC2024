package frc.robot.commands;

import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class TestAutoConveyorCommand extends Command {
    //declaring subsystems
    private ConveyorSubsystem m_conveyorSubsystem;

    //declaring limit switch
    private DigitalInput m_conveyorSwitch;

    //declaring used values 
    private double conveyorPower;
    private boolean didContact;

    //constructor
    public TestAutoConveyorCommand(ConveyorSubsystem conveyorSubsystem, DigitalInput limitSwitch, double conveyorPower) {
        //initially set to false due to the note not contacting the switch yet
        didContact = false;
        this.conveyorPower = conveyorPower;
        
        //initializing hardware
        m_conveyorSwitch = limitSwitch;

        //initializing subsystems
        m_conveyorSubsystem = conveyorSubsystem;
        addRequirements(m_conveyorSubsystem);
    }
    
    @Override
    public void initialize() {
        System.out.println("Conveyor command started");
        m_conveyorSubsystem.setConveyorSpeed(conveyorPower);

        //sets the condition to false everytime the command is ran
        if(didContact) {
            didContact = false;
        }
    }

    @Override
    public void execute() {
        //updates the boolean once the limit switch is triggered
        if(m_conveyorSwitch.get()) {
            didContact = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Conveyor command finished");
        m_conveyorSubsystem.shutdown();
    }

    //in progress
    @Override
    public boolean isFinished() {
        //only ends the command once the switch is released 
        return didContact && !m_conveyorSwitch.get();
    }
}