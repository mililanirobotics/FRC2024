package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.ScoringConstants;

public class ScoringSubsystem extends SubsystemBase{
    private CANSparkMax topLaunchRoller;
    private CANSparkMax bottomLaunchRoller;
    private DigitalInput scoringSensor;

    public ScoringSubsystem() {
        topLaunchRoller = new CANSparkMax(ScoringConstants.kUpperFlywheelPort, MotorType.kBrushless);
        bottomLaunchRoller = new CANSparkMax(ScoringConstants.kLowerFlywheelPort, MotorType.kBrushless);
        scoringSensor = new DigitalInput(ScoringConstants.kScoringSensorPort);

        topLaunchRoller.setInverted(ScoringConstants.kUpperFlywheelReverse);
        bottomLaunchRoller.setInverted(ScoringConstants.kLowerFlywheelReverse);
    }

    public void setSpeed(double botPower, double topPower) {
      topLaunchRoller.set(topPower);
      bottomLaunchRoller.set(botPower);
    }

    public boolean getSensorReading() {
      return scoringSensor.get();
    }

    public void shutdown() {
      topLaunchRoller.set(0);
      bottomLaunchRoller.set(0);
    }

    @Override
    public void periodic() {

    }
}