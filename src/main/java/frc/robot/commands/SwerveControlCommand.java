package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.SlewRateLimiter;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.DriveConstants;


public class SwerveControlCommand extends Command{
    private ChassisSpeeds chassisSpeeds;

    // Declaring the Subsystem
    private SwerveDriveSubsystem m_SwerveDriveSubsystem;

    // SlewRateLimiter limits the rate of acceleration to be gradual and linear
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    private GenericHID gamepad;

    public SwerveControlCommand(SwerveDriveSubsystem swerveDriveSubsystem, GenericHID gamepad) {
        m_SwerveDriveSubsystem = swerveDriveSubsystem;
        this.gamepad = gamepad;

        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAcceleration);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAcceleration);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleRotationMaxAngularAcceleration);

        addRequirements(m_SwerveDriveSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Grabs Joystick Inputs as Speed Inputs
        double xSpeed = gamepad.getRawAxis(JoystickConstants.kLeftYJoystickPort);
        double ySpeed = gamepad.getRawAxis(JoystickConstants.kleftXJoystickPort);
        double turningSpeed = gamepad.getRawAxis(JoystickConstants.kRightXJoystickPort);
        
        // Apply Deadband to prevent motors accidentally spinning
        xSpeed = Math.abs(xSpeed) > JoystickConstants.kDeadzone ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > JoystickConstants.kDeadzone ? ySpeed : 0.0; 
        turningSpeed = Math.abs(turningSpeed) > JoystickConstants.kDeadzone ? turningSpeed : 0.0;

        //Limiting Drive Speeds Acceleration to be linear
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxAcceleration;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxAcceleration;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleRotationMaxAngularAcceleration;

        // Creating desired chassis speeds from joystick inputs.
        chassisSpeeds = ChassisSpeeds.discretize(ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, turningSpeed, m_SwerveDriveSubsystem.getRotation2dDegContinuous()
        ), 0.02);

        // Convert chassis speeds into swerve module states
        SwerveModuleState[] moduleStates = SwerveModuleConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Output each module state to the wheels
        m_SwerveDriveSubsystem.setModuleStates(moduleStates);
        

        // Temporary CANCoder print
        m_SwerveDriveSubsystem.getCANCoderReading();
        SmartDashboard.putNumber("Naxv Yaw", m_SwerveDriveSubsystem.getYaw());
        SmartDashboard.putNumber("Naxv Degrees", m_SwerveDriveSubsystem.getDegrees());
        SmartDashboard.putNumber("Radians", m_SwerveDriveSubsystem.getRad());
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_SwerveDriveSubsystem.shutdown();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
        
}
