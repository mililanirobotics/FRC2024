package frc.robot.subsystems;

//general imports
import java.lang.Math;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    //determines if the module is straight
    private Trigger isReset;

    //constructor
    public SwerveModule(int drivePort, int rotationPort, boolean driveReversed, boolean rotationReversed,
            int CANCoderPort, double CANCoderOffset, boolean CANCoderReversed) {
        //initializing absolute encoder parameters 
        this.CANCoderReversed = CANCoderReversed;
        angleCANCoder = new CANcoder(CANCoderPort);
      
        // Configure the CANcoder for basic use
        CANcoderConfiguration configs = new CANcoderConfiguration();
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
        
        //reseting faults 
        driveMotor.clearFaults();
        driveMotor.setSmartCurrentLimit(SwerveModuleConstants.kDriveCurrentLimit);
        rotationMotor.clearFaults();
        rotationMotor.setSmartCurrentLimit(SwerveModuleConstants.kRotationCurrentLimit);
        driveMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);
        rotationMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

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
        // return driveMotor.getDeviceId() == SwerveModuleConstants.kLeftFrontWheelPort 
        //         || driveMotor.getDeviceId() == SwerveModuleConstants.kRightFrontWheelPort
        //             ? -driveEncoder.getPosition() : driveEncoder.getPosition();
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

    public double getPower() {
        return driveMotor.getAppliedOutput();
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
    public double getCANCoderReadingCont() {
        double angle = angleCANCoder.getAbsolutePosition().getValueAsDouble();
        return (angle * 2.0 * Math.PI) * (CANCoderReversed ? -1 : 1);   
    }

    /**
     * Returns the current reading of the absolute encoder from -PI to PI
     * @return The angle of the CANCoder 
     */
    public double getCANCoderReading() {
        return getCANCoderReadingCont() >= Math.PI ? getCANCoderReadingCont() - 2 * Math.PI : getCANCoderReadingCont();
    }

    /**
     * Returns the current reading of the absolute encoder in rotations (0 to 1)
     * @return The rotational position of the CANCoder
     */
    public double getAbsoluteRotations() {
        return angleCANCoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * Resets the encoders to their default position
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        rotationEncoder.setPosition(getCANCoderReading());
    }

    public boolean isReset() {
        return getCANCoderReadingCont() < SwerveModuleConstants.kTurningTolerance
                || getCANCoderReadingCont() < SwerveModuleConstants.kTurningTolerance;
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
     * Rotates the module to a desired setpoint from -PI to PI
     * @param setpoint
     */
    public void spinTo(double setpoint) {
        double angle = getCANCoderReading();

        if(Math.abs(angle - setpoint) > SwerveModuleConstants.kTurningTolerance) {
            rotationMotor.set(rotationPID.calculate(setpoint, angle));
        }
        else {
            rotationMotor.set(0);
        }
    }

    /**
     * Sets the optimal swerve module state to a given setpoint 
     * Changes the target drive and rotation speed of the module
     * @param currentState The swerve module state of the motor
     */
    public void setSwerveState(SwerveModuleState currentState, boolean needsAdjustment) {
        if(Math.abs(currentState.speedMetersPerSecond) < 0.001) {
            shutdown();
            return;
        }
        
        if(needsAdjustment) {
            currentState = SwerveModuleState.optimize(currentState, getModuleState().angle);
            driveMotor.set(currentState.speedMetersPerSecond * 1.1 / DriveConstants.kDriveMaxMetersPerSecond);
            rotationMotor.set(rotationPID.calculate(getRotationPosition(), currentState.angle.getRadians()));
        }
        else {
            currentState = SwerveModuleState.optimize(currentState, getModuleState().angle);
            driveMotor.set(currentState.speedMetersPerSecond / DriveConstants.kDriveMaxMetersPerSecond);
            rotationMotor.set(rotationPID.calculate(getRotationPosition(), currentState.angle.getRadians()));
        }
        
    }
}

