package frc.robot.subsystems;

//general imports
import java.lang.Math;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//constants
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
    //Spark max motor controllers
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;
    //encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;
    //PID controller
    private final PIDController rotationPID;
    //angle offsets
    private final CANcoder angleCANCoder;
    private final boolean CANCoderReversed;

    //constructor
    public SwerveModule(int drivePort, int rotationPort, boolean driveReversed, boolean rotationReversed,
            int CANCoderPort, double CANCoderOffset, boolean CANCoderReversed) {
        //initializing absolute encoder parameters 
        this.CANCoderReversed = CANCoderReversed;
        angleCANCoder = new CANcoder(CANCoderPort);
        //angleCANCoder.setPosition(angleCANCoder.getAbsolutePosition().getValueAsDouble());
        // angleCANCoder.setPositionToAbsolute();
        // angleCANCoder.configAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0_to_360);
        // angleCANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        // angleCANCoder.setControl()
        // Configure the CANcoder for basic use
        CANcoderConfiguration configs = new CANcoderConfiguration();
        
        // This CANcoder should report absolute position from [-0.5, 0.5) rotations,
        // with a 0.26 rotation offset, with clockwise being positive
        configs.MagnetSensor
            .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
            .withMagnetOffset(CANCoderOffset)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        
        // Write these configs to the CANcoder
        angleCANCoder.getConfigurator().apply(configs);
        
        //initializing SparkMax
        driveMotor = new CANSparkMax(drivePort, com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationPort, MotorType.kBrushless);
        driveMotor.setInverted(driveReversed);
        rotationMotor.setInverted(rotationReversed);

        //initializing encoders
        driveEncoder = driveMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        rotationEncoder = rotationMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);   

        //converting native units to measurements
        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.kRotationToMeters);
        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.kMetersPerSecond);
        rotationEncoder.setPositionConversionFactor(SwerveModuleConstants.kRotationToRadians);
        rotationEncoder.setVelocityConversionFactor(SwerveModuleConstants.kRadiansPerSecond);

        //initializing PID controller
        rotationPID = new PIDController(
            SwerveModuleConstants.kTurningP, 
            SwerveModuleConstants.kTurningI, 
            SwerveModuleConstants.kTurningD
        );
        //calculates the least turning degrees to setpoint
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    //=========================================================================== 
    // helper methods
    //===========================================================================

    /**
     * Returns the current position of the swerve module's drive motor
     * @return The current displacement of the drive motor in meters
     */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /**
     * Returns the current position of the swerve module's rotation motor
     * @return The current rotation of the rotation motor in radians
     */
    public double getRotationPosition() {
        return rotationEncoder.getPosition();
    }

    /**
     * Returns the current velocity of the swerve module's drive motor
     * @return The current velocity of the drive motor in meters per second
     */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /**
     * Returns the current velocity of the swerve module's rotation motor
     * @return The current velocity of the rotation motor in radians per second
     */
    public double getRotationVelocity() {
        return rotationEncoder.getVelocity();
    }

    /**
     * Returns the current reading of the absolute encoder
     * Indicates which direction the motor should turn based on absoluteEncoderReversed 
     * @return The current reading of the absolute encoder in radians
     */
    public double getCANCoderReading() {
        double angle = (angleCANCoder.getAbsolutePosition().getValueAsDouble());
        return (angle * 2.0 * Math.PI) * (CANCoderReversed ? -1 : 1);   
    }

    /**
     * Resets the encoders to their default position
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(getCANCoderReading());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getRotationPosition()));
    }
    /**
     * Stops the movement of the drive and rotation motor 
     */
    public void shutdown() {
        driveMotor.set(0);
        rotationMotor.set(0);
    }

    /**
     * Sets the optimal swerve module state to a given setpoint 
     * Changes the target drive and rotation speed of the module
     * @param currentState The swerve module state of the motor
     */
    public void setSwerveState(SwerveModuleState currentState) {
        if(Math.abs(currentState.speedMetersPerSecond) < 0.001) {
            shutdown();
            return;
        }

        currentState = SwerveModuleState.optimize(currentState, getModuleState().angle);
        driveMotor.set(currentState.speedMetersPerSecond / DriveConstants.kDriveMaxMetersPerSecond);
        rotationMotor.set(rotationPID.calculate(getRotationPosition(), currentState.angle.getRadians()));
    }
}

