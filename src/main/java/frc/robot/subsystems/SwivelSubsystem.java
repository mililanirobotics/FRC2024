package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwivelSubsystem extends SubsystemBase {
    private Servo swivelewdfdafsf;
    
    public SwivelSubsystem() {
        swivelewdfdafsf = new Servo(0);
        swivelewdfdafsf.setSpeed(1);
    }

    public void setSpeed( double set) {
        swivelewdfdafsf.setSpeed(set);
    }

    public void setAngle(double angle) {
        swivelewdfdafsf.setAngle(angle);
    }

    public double getAngle() {
        return swivelewdfdafsf.getAngle();
    }

    public void setPosition(double pos) {
        swivelewdfdafsf.setPosition(pos);
    }

    public double getPosition() {
        return swivelewdfdafsf.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Angle", getAngle());
        SmartDashboard.putNumber("Pos", getPosition());
    }
}