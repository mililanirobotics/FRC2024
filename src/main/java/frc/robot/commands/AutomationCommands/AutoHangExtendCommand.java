package frc.robot.commands.AutomationCommands;

//subsystems
import frc.robot.subsystems.HangSubsystem;
//commands
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Automatically extends the hang to the correct height for hanging on the chain
 */
public class AutoHangExtendCommand extends Command {
    //declaring subsystems
    private HangSubsystem m_hangSubsystem;

    //constructor
    public AutoHangExtendCommand(HangSubsystem hangSubsystem) {
        //initializing subsystems
        m_hangSubsystem = hangSubsystem;
        addRequirements(m_hangSubsystem);
    }
    
    @Override
    public void initialize() {
        //printing initialize statement 
        System.out.println("Lifting hang");
        //setting hang power to max
        m_hangSubsystem.setSpeed(1);
    }

    @Override
    public void end(boolean interrupted) {
        //shutting down hang and setting its state to extended
        m_hangSubsystem.shutdown();
        m_hangSubsystem.setHangState(true);
    }

    @Override
    public boolean isFinished() {
        //stops the hang once the IR sensor holes are lined up
        return m_hangSubsystem.getHangSensorReading();
    }
}