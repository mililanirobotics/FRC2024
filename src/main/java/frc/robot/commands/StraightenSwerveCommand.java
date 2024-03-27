// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.SwerveModuleConstants;
// import frc.robot.subsystems.SwerveDriveSubsystem;
// import frc.robot.subsystems.SwerveModule;



// public class StraightenSwerveCommand extends Command {
//     // Declaring the Subsystem
//     private SwerveDriveSubsystem m_swerveDriveSubsystem;
//     private SwerveModule[] m_modules;

//     public StraightenSwerveCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
//         m_swerveDriveSubsystem = swerveDriveSubsystem;
//         m_modules = m_swerveDriveSubsystem.getModules();
//         addRequirements(m_swerveDriveSubsystem);
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         m_modules[SwerveModuleConstants.kLeftFrontIndex].spinTo(0);
//         m_modules[SwerveModuleConstants.kRightFrontIndex].spinTo(0);
//         m_modules[SwerveModuleConstants.kLeftBackIndex].spinTo(0);
//         m_modules[SwerveModuleConstants.kRightBackIndex].spinTo(0);
//     }
    
//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         System.out.println("Command Ended");
//         m_swerveDriveSubsystem.shutdown();
//         m_modules[SwerveModuleConstants.kLeftFrontIndex].resetEncoders();
//         m_modules[SwerveModuleConstants.kRightFrontIndex].resetEncoders();
//         m_modules[SwerveModuleConstants.kLeftBackIndex].resetEncoders();
//         m_modules[SwerveModuleConstants.kRightBackIndex].resetEncoders();
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return 
//         m_modules[SwerveModuleConstants.kLeftFrontIndex].isReset()
//          && m_modules[SwerveModuleConstants.kRightFrontIndex].isReset() 
//          && m_modules[SwerveModuleConstants.kLeftBackIndex].isReset() 
//          && m_modules[SwerveModuleConstants.kRightBackIndex].isReset();
//     }
        
// }
