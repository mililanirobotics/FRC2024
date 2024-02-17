// package frc.robot.commands.AutomationCommands;

// import frc.robot.subsystems.IntakeConveyorSubsystem;
// import edu.wpi.first.wpilibj2.command.Command;

// public class AutoIntakeConveyorCommand extends Command {
//     //declaring subsystems
//     private IntakeConveyorSubsystem m_intakeConveyorSubsystem;
//     //declaring variables
//     private boolean passedFront;
//     private boolean passedBack;
//     private boolean seenMiddle;

//     //constructor
//     public AutoIntakeConveyorCommand(IntakeConveyorSubsystem intnakeConveyorSubsystem) {
//         //sets the condition to false everytime the command is ran
//         //initializing subsystems
//         m_intakeConveyorSubsystem = intnakeConveyorSubsystem;
//         addRequirements(m_intakeConveyorSubsystem);
//     }
    
//     @Override
//     public void initialize() {
//         passedFront = false;
//         passedBack = false;
//         seenMiddle = false;
//         System.out.println("Scoring command started");
//         m_intakeConveyorSubsystem.setSpeeds(1);
//     }

//     @Override
//     public void execute() {
//         //updates the boolean once the IR sensor is triggered
//         if(!m_intakeConveyorSubsystem.getStopSensorReading()) {
//             passedFront = true;
//         }
//         if (passedFront && m_intakeConveyorSubsystem.getStopSensorReading()) {
//             seenMiddle = true;
//         }
//         if(passedFront && seenMiddle && !m_intakeConveyorSubsystem.getStopSensorReading()) {
//             passedBack = true;
//         }
//         System.out.println("Sensor Reading: "+m_intakeConveyorSubsystem.getStopSensorReading());
//         System.out.println("Passed Front: "+passedFront);
//         System.out.println("Passed Middle: "+seenMiddle);
//         System.out.println("Passed Back: "+passedBack);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         m_intakeConveyorSubsystem.shutdown();
//     }

//     //in progress
//     @Override
//     public boolean isFinished() {
//         //stops the command once the note has fully passed the IR sensor
//         return passedBack && m_intakeConveyorSubsystem.getStopSensorReading();
//     }
// }