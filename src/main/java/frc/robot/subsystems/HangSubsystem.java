package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;

import frc.robot.Constants.HangConstants;
import frc.robot.Constants.IntakeConveyorConstants;

public class HangSubsystem extends SubsystemBase {
    private CANSparkMax hangMotor;
    private RelativeEncoder hangEncoder;

    private DigitalInput hangSensor;
    private boolean isHangDeployed;

    //constructor
    public HangSubsystem() {
        //initializing solenoids
        hangMotor = new CANSparkMax(HangConstants.kHangPort, MotorType.kBrushless);
        hangMotor.setInverted(HangConstants.kHangReverse);

        hangMotor.clearFaults();
        hangMotor.setIdleMode(CANSparkFlex.IdleMode.kBrake);

        hangSensor = new DigitalInput(HangConstants.kHangSensorPort);


        hangEncoder = hangMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
        resetEncoders();
    }

    public boolean getHangState() {
        return isHangDeployed;
    }

    public void setHangState(boolean state) { 
        isHangDeployed = state;
    }

    public boolean getHangSensorReading() {
        return hangSensor.get();
    }

    public boolean getHangSensorReadingReverse() {
        return !hangSensor.get();
    }

    public void setSpeed(double power) {
        hangMotor.set(power);
    } 

    public void resetEncoders() {
        hangEncoder.setPosition(0);
    }

    public void shutdown() {
        hangMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hang Rotations", hangEncoder.getPosition());
        SmartDashboard.putBoolean("Hang", getHangSensorReading());
    }
    
}