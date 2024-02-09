// package frc.robot.commands.AutomationCommands;

// import frc.robot.Constants.ScoringConstants;
// import frc.robot.subsystems.ScoringSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AutoScoringCommand extends Command {
//     //declaring subsystems
//     private ScoringSubsystem m_scoringSubsystem;
//     //declaring variables
//     private boolean didContact;

//     //constructor
//     public AutoScoringCommand(ScoringSubsystem scoringSubsystem) {
//         //sets the condition to false everytime the command is ran
//         didContact = false;
//         //initializing subsystems
//         m_scoringSubsystem = scoringSubsystem;
//         addRequirements(m_scoringSubsystem);
//     }
    
//     @Override
//     public void initialize() {
//         System.out.println("Scoring command started");
//         m_scoringSubsystem.setSpeed(ScoringConstants.kBotRollerSpeed, ScoringConstants.kTopRollerSpeed);
//     }

//     @Override
//     public void execute() {
//         //updates the boolean once the IR sensor is triggered
//         if(m_scoringSubsystem.getStopSensorReading()) {
//             didContact = true;
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_scoringSubsystem.shutdown();
//     }

//     //in progress
//     @Override
//     public boolean isFinished() {
//         //stops the command once the note has fully passed the IR sensor
//         return didContact && !m_scoringSubsystem.getStopSensorReading();
//     }
// }