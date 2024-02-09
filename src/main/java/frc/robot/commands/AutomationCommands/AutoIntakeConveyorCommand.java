// package frc.robot.commands.AutomationCommands;

// import frc.robot.subsystems.IntakeConveyorSubsystem;
// //subsystems and commands
// import edu.wpi.first.wpilibj2.command.Command;

// public class AutoIntakeConveyorCommand extends Command {
//     //declaring subsystems
//     private IntakeConveyorSubsystem m_intakeConveyorSubsystem;
//     //declaring variables
//     private boolean didContact;
//     private double power;

//     //constructor
//     public AutoIntakeConveyorCommand(IntakeConveyorSubsystem intakeConveyorSubsystem, double power) {
//         //sets the condition to false everytime the command is ran
//         didContact = false;
//         this.power = power;
//         //initializing subsystems
//         m_intakeConveyorSubsystem = intakeConveyorSubsystem;
//         addRequirements(m_intakeConveyorSubsystem);
//     }
    
//     @Override
//     public void initialize() {
//         System.out.println("IntakeConveyor command started");
//         System.out.println("Sensor Triggered: "+m_intakeConveyorSubsystem.getStopSensorReading());

//         //runs both the conveyor and the intake to help guide the piece
//         m_intakeConveyorSubsystem.setSpeeds(power);

//         //sets the condition to false everytime the command is ran
//         if(didContact) {
//             didContact = false;
//         }
//     }

//     @Override
//     public void execute() {
//         //updates the condition to true once the note has contacted the switch
//         if(m_intakeConveyorSubsystem.getStopSensorReading()) {
//             didContact = true;
//         }
//     }

//     @Override
//     public void end(boolean interrupted) {
//         System.out.println("IntakeConveyor command finished");
//         m_intakeConveyorSubsystem.setInScorer(true);
//         m_intakeConveyorSubsystem.shutdown();
//     }

//     //in progress
//     @Override
//     public boolean isFinished() {
//         //finishes the command once the note fully passes the sensor
//         return didContact && !m_intakeConveyorSubsystem.getStopSensorReading();
//     }
// }