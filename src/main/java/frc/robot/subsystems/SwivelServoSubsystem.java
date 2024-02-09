package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwivelServoSubsystem extends SubsystemBase {
    Servo swivelServo;

    public SwivelServoSubsystem() {
        swivelServo = new Servo(0);
    }

    public void AprilTagView() {
        swivelServo.setAngle(104);
    }

    public void DriverView() {
        swivelServo.setAngle(56);
    }

    public void NeutralView() {
        swivelServo.setAngle(90);
    }
}
