package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
//general imports
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
//path planner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
//constants
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveDriveSubsystem extends SubsystemBase {
    //Swerve modules
    private SwerveModule leftFrontModule;
    private SwerveModule rightFrontModule;
    private SwerveModule leftBackModule;
    private SwerveModule rightBackModule;
    //array of modules; used to get states later
    private SwerveModule[] modules;
    //Navx and odometry
    private AHRS navX;
    private SwerveDriveOdometry odometry;
    //Field2d 
    private Field2d field;
    //position translational 
    private GenericEntry leftFrontTranPosWidget;
    private GenericEntry leftBackTranPosWidget;
    private GenericEntry rightFrontTranPosWidget;
    private GenericEntry rightBackTranPosWidget;
    //velocity translational
    private GenericEntry leftFrontTranVelWidget;
    private GenericEntry leftBackTranVelWidget;
    private GenericEntry rightFrontTranVelWidget;
    private GenericEntry rightBackTranVelWidget;
    //position rotational 
    private GenericEntry leftFrontRotPosWidget;
    private GenericEntry leftBackRotPosWidget;
    private GenericEntry rightFrontRotPosWidget;
    private GenericEntry rightBackRotPosWidget;
    //velocity rotational
    private GenericEntry leftFrontRotVelWidget;
    private GenericEntry leftBackRotVelWidget;
    private GenericEntry rightFrontRotVelWidget;
    private GenericEntry rightBackRotVelWidget;  
    //gyro data
    private GenericEntry gyroData;

    //constructor
    public SwerveDriveSubsystem(ShuffleboardTab testTranPos, ShuffleboardTab testTranVel, ShuffleboardTab testRotPos, 
                                 ShuffleboardTab testRotVel, ShuffleboardTab testPos, ShuffleboardTab testGyroData) {
        //Front Left Module Initializing
        leftFrontModule = new SwerveModule(
            SwerveModuleConstants.kLeftFrontWheelPort, 
            SwerveModuleConstants.kLeftFrontRotationPort, 
            SwerveModuleConstants.kLeftFrontDriveReversed, 
            SwerveModuleConstants.kLeftFrontRotationReversed, 
            SwerveModuleConstants.kLeftFrontCANCoderPort, 
            SwerveModuleConstants.kLeftFrontCANCoderOffset, 
            SwerveModuleConstants.kLeftFrontCANCoderReversed
        );

        //Front Right Module Initializing
        rightFrontModule = new SwerveModule(
            SwerveModuleConstants.kRightFrontWheelPort, 
            SwerveModuleConstants.kRightFrontRotationPort, 
            SwerveModuleConstants.kRightFrontDriveReversed, 
            SwerveModuleConstants.kRightFrontRotationReversed,   
            SwerveModuleConstants.kRightFrontCANCoderPort, 
            SwerveModuleConstants.kRightFrontCANCoderOffset, 
            SwerveModuleConstants.kRightFrontCANCoderReversed
        );

        //Back Left Module Initializing
        leftBackModule = new SwerveModule(
            SwerveModuleConstants.kLeftBackWheelPort, 
            SwerveModuleConstants.kLeftBackRotationPort, 
            SwerveModuleConstants.kLeftBackDriveReversed, 
            SwerveModuleConstants.kLeftBackRotationReversed, 
            SwerveModuleConstants.kLeftBackCANCoderPort, 
            SwerveModuleConstants.kLeftBackCANCoderOffset, 
            SwerveModuleConstants.kLeftBackCANCoderReversed
        );

        //Back Right Module Initializing
        rightBackModule = new SwerveModule(
            SwerveModuleConstants.kRightBackWheelPort, 
            SwerveModuleConstants.kRightBackRotationPort, 
            SwerveModuleConstants.kRightBackDriveReversed, 
            SwerveModuleConstants.kRightBackRotationReversed, 
            SwerveModuleConstants.kRightBackCANCoderPort, 
            SwerveModuleConstants.kRightBackCANCoderOffset, 

            SwerveModuleConstants.kRightBackCANCoderReversed
        );

        //adding modules to the array
        modules = new SwerveModule[] {
            leftBackModule, rightBackModule, leftFrontModule, leftBackModule
        };

        //initializing and resetting Navx
        navX = new AHRS(SPI.Port.kMXP);
        navX.enableLogging(true);
        navX.reset();

        //initializing odometry that uses continuous 360 degree input
        odometry = new SwerveDriveOdometry(
            SwerveModuleConstants.kinematics, 
            getRotation2dDegContinuous(),
            getModulePosition()
        );

        //initializing AutoBuilder to create path planner autopaths
        //flips the created autopath if on the Red Alliance
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getSpeeds, 
            this::driveRobotRelative, 
            AutoConstants.pathFollowingConfig, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()) { 
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        //Path Planner logging
        field = new Field2d();

        //Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback(
            (pose) -> {
                field.setRobotPose(pose);
            }
        );

        //Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
            field.getObject("target pose").setPose(pose);
        }
        );

        //Logging callback for the active path, this is sent as a list of poses 
        PathPlannerLogging.setLogActivePathCallback(
        (poses) -> {
            field.getObject("path").setPoses(poses);
        }
        );

        testPos.add("Field", field).withSize(5, 2);

        //adding translational position widgets
        leftFrontTranPosWidget = testTranPos.add("left_front_tran_pos", 0).withSize(2, 1).getEntry();
        leftBackTranPosWidget = testTranPos.add("left_back_tran_pos", 0).withSize(2, 1).getEntry();
        rightFrontTranPosWidget = testTranPos.add("right_front_tran_pos", 0).withSize(2, 1).getEntry();
        rightBackTranPosWidget = testTranPos.add("right_back_tran_pos", 0).withSize(2, 1).getEntry();
        //adding translational velocity widgets
        leftFrontTranVelWidget = testTranVel.add("left_front_tran_vel", 0).withSize(2, 1).getEntry();
        leftBackTranVelWidget = testTranVel.add("left_back_tran_vel", 0).withSize(2, 1).getEntry();
        rightFrontTranVelWidget = testTranVel.add("right_front_tran_vel", 0).withSize(2, 1).getEntry();
        rightBackTranVelWidget = testTranVel.add("right_back_tran_vel", 0).withSize(2, 1).getEntry();
        //adding rotational position widgets
        leftFrontRotPosWidget = testRotPos.add("left_front_rot_pos", 0).withSize(2, 1).getEntry();
        leftBackRotPosWidget = testRotPos.add("left_back_rot_pos", 0).withSize(2, 1).getEntry();
        rightFrontRotPosWidget = testRotPos.add("right_front_rot_pos", 0).withSize(2, 1).getEntry();
        rightBackRotPosWidget = testRotPos.add("right_back_rot_pos", 0).withSize(2, 1).getEntry();
        //adding rotational velocity widgets
        leftFrontRotVelWidget = testRotVel.add("left_front_rot_vel", 0).withSize(2, 1).getEntry();
        leftBackRotVelWidget = testRotVel.add("left_back_rot_vel", 0).withSize(2, 1).getEntry();
        rightFrontRotVelWidget = testRotVel.add("right_front_rot_vel", 0).withSize(2, 1).getEntry();
        rightBackRotVelWidget = testRotVel.add("right_back_rot_vel", 0).withSize(2, 1).getEntry();
        //adding gyro widget
        gyroData = testGyroData.add("gyro_data", getYaw()).withSize(2, 1).getEntry();

        SmartDashboard.putNumber("Right_back_power", rightBackModule.getPower());
        SmartDashboard.putNumber("Right_front_power", rightFrontModule.getPower());
        SmartDashboard.putNumber("Left_front_power", leftFrontModule.getPower());
        SmartDashboard.putNumber("Left_back_power", leftBackModule.getPower());
    }

    //=========================================================================== 
    // gyro and accelorometer methods
    //===========================================================================

    /**
     * Resets the current angle of the gyro to 0. 
     * Tells the driver that the gyro is connected via a print statement
    */
    public void zeroOutGyro() {
        System.out.println("Gyro Connected: "+navX.isConnected());
        navX.reset();
    }

    /**
     * Gets the current yaw angle from the navx gyro
     * @return The yaw value recorded
     */
    public double getYaw() {
        return navX.getYaw();
    }

    /**
     * Gets the current pitch angle from the navx gyro
     * @return The pitch value recorded
     */
    public double getPitch() {
        return navX.getPitch();
    }

    /**
     * Gets the current roll angle from the navx gyro
     * @return The roll value recorded
     */
    public double getRoll() {
        return navX.getRoll();
    }

    /**
     * Returns a Rotation2d object from a continuous 0-360 degree rotation
     * @return The continuous degree Rotation2d object
     */
    public Rotation2d getRotation2dDegContinuous() {
        return Rotation2d.fromDegrees(getDegrees());
    }

    /**
     * Returns a Rotation2d object from a continuous 0-2π degree rotation
     * @return The continuous radian Rotation2d object
     */
    public Rotation2d getRotation2dRadContinuous() {
        return Rotation2d.fromRadians(getRad());
    }

    /**
     * Returns a Rotation2d object from a -180 to 180 degree rotation
     * @return The degree Rotation2d object
     */
    public Rotation2d getRotation2dDeg() {
        return Rotation2d.fromDegrees(getYaw());
    }

    /**
     * Returns a Rotation2d object from -π to π radian rotation
     * @return The radian Rotation2d object
     */
    public Rotation2d getRotation2dRad() {
        return Rotation2d.fromRadians(Units.degreesToRadians(getYaw()));
    }

    /**
     * Returns the yaw value of the Navx assuming CCW pos and continuous (in degrees)
     * @return The adjusted degrees
     */
    public double getDegrees() {
        double rawDegrees = -getYaw();
        return rawDegrees < 0 ? rawDegrees + 360 : rawDegrees;
    }

    /**
     * Returns the yaw value of the Navx assuming CCW pos and continuous (in radians)
     * @return The adjusted radians
     */
    public double getRad() {
        double rad = Units.degreesToRadians(-getYaw());
        return rad < 0 ? rad + 2 * Math.PI : rad;
    }

    //=========================================================================== 
    // drive methods
    //===========================================================================

    /**
     * Sets the desired states for the swere modules; modules will automatically go to the desired states
     * @param desiredStates The desired states from the ChassisSpeeds object
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        //sets drive constants to the states
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kDriveMaxMetersPerSecond);
        //sets the states
        leftFrontModule.setSwerveState(desiredStates[0]);
        rightFrontModule.setSwerveState(desiredStates[1]);
        leftBackModule.setSwerveState(desiredStates[2]);
        rightBackModule.setSwerveState(desiredStates[3]);
    }

    /**
     * Returns the current swerve module states
     * @return An array of the current swerve module states
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for(int i = 0; i < states.length; i++) {
            states[i] = modules[i].getModuleState();
        }

        return states;
    }

    /**
    * Returns the current position of the module (displacement)
    * @return The current displacement of the module
    */
    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(
                leftFrontModule.getDrivePosition(), new Rotation2d(leftFrontModule.getRotationPosition())
            ),
            new SwerveModulePosition(
                rightFrontModule.getDrivePosition(), new Rotation2d(rightFrontModule.getRotationPosition())
            ),
            new SwerveModulePosition(
                leftBackModule.getDrivePosition(), new Rotation2d(leftBackModule.getRotationPosition())
            ),
            new SwerveModulePosition(
                rightBackModule.getDrivePosition(), new Rotation2d(rightBackModule.getRotationPosition())
            )
        };
    }

    /**
     * Returns a robot oriented ChassisSpeed object
     * @return The robot oriented ChassisSpeed object
     */
    public ChassisSpeeds getSpeeds() {
        return SwerveModuleConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Sets the swerve states to robot relative control
     * @param robotRelativeSpeed The robot relative ChassisSpeed object
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeed) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeed, getRotation2dDeg());
        targetSpeeds = ChassisSpeeds.discretize(targetSpeeds, 0.02);    

        SwerveModuleState[] targetState = SwerveModuleConstants.kinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetState);
    }

    /**
     * Sets the power of the conveyor motors to 0
     */
    public void shutdown() {
        leftFrontModule.shutdown();
        rightFrontModule.shutdown();
        leftBackModule.shutdown();
        rightBackModule.shutdown();
    }

    //=========================================================================== 
    // misc methods
    //===========================================================================
    
    /**
     * Returns the recorded odometry position of the robot on the field 
     * @return
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry object; sets its position based on the robot's current rotation and position
     * @param pose The current position of the robot
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(getYaw()), getModulePosition(), pose);
    }

    /**
     * Prints the CANCoder readings to SmartDashboard
     */
    public void getCANCoderReading() {
        SmartDashboard.putNumber("Left Front: ", leftFrontModule.getAbsoluteRotations());
        SmartDashboard.putNumber("Right Front: ", rightFrontModule.getAbsoluteRotations());
        SmartDashboard.putNumber("Left Back: ", leftBackModule.getAbsoluteRotations());
        SmartDashboard.putNumber("Right Back: ", rightBackModule.getAbsoluteRotations());
        SmartDashboard.updateValues();
    }

    public void printTestingData() {
        //translational position
        leftFrontTranPosWidget.setDouble(leftFrontModule.getDrivePosition());
        leftBackTranPosWidget.setDouble(leftBackModule.getDrivePosition());
        rightFrontTranPosWidget.setDouble(rightFrontModule.getDrivePosition());
        rightBackTranPosWidget.setDouble(rightBackModule.getDrivePosition());
        //translational velocity
        leftFrontTranVelWidget.setDouble(leftFrontModule.getDriveVelocity());
        leftBackTranVelWidget.setDouble(leftBackModule.getDriveVelocity());
        rightFrontTranVelWidget.setDouble(rightFrontModule.getDriveVelocity());
        rightBackTranVelWidget.setDouble(rightBackModule.getDriveVelocity());
        //rotational position
        leftFrontRotPosWidget.setDouble(leftFrontModule.getRotationPosition());
        leftBackRotPosWidget.setDouble(leftBackModule.getRotationPosition());
        rightFrontRotPosWidget.setDouble(rightFrontModule.getRotationPosition());
        rightBackRotPosWidget.setDouble(rightBackModule.getRotationPosition());
        //rotational velocity
        leftFrontRotVelWidget.setDouble(leftFrontModule.getRotationVelocity());
        leftBackRotVelWidget.setDouble(leftBackModule.getRotationVelocity());
        rightFrontRotVelWidget.setDouble(rightFrontModule.getRotationVelocity());
        rightBackRotVelWidget.setDouble(rightBackModule.getRotationVelocity());
        //odometry 
        // odometryPos.setValue(odometry.getPoseMeters());
        //gyro
        gyroData.setValue(getYaw());
    }

    @Override
    public void periodic() {
        //updates odometry
        odometry.update(
            Rotation2d.fromDegrees(getYaw()),
            getModulePosition()
        );

        //updates the robot's pose on the Field2d widget
        field.setRobotPose(odometry.getPoseMeters());

        //prints the CANcoder readings on all 4 modules 
        printTestingData();
        getCANCoderReading();

        //path planner
        PathPlannerLogging.setLogCurrentPoseCallback(
            (pose) -> {
                field.setRobotPose(pose);
            }
        );

        //Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback(
            (pose) -> {
                field.getObject("target pose").setPose(pose);
            }
        );

        //Logging callback for the active path, this is sent as a list of poses 
        PathPlannerLogging.setLogActivePathCallback(
            (poses) -> {
                field.getObject("path").setPoses(poses);
            }
        );
    }
}
