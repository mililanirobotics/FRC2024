package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;

public class SwivelSubsystem {
    private Servo swivelServo;

    public SwivelSubsystem() {
        // Initializing Servo
        swivelServo = new Servo(1);

    }
}
