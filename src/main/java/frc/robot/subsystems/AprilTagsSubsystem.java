package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
//constants
import frc.robot.Constants; 


public class AprilTagsSubsystem extends SubsystemBase{
    
    private NetworkTable table;

    public enum Pipeline {

        /**
         * Pipeline IDs are not set, thinking whether it matters if we need two pipelines for each AMP, driver view needs to be configured.
         */
        RED_AMP(0), BLUE_AMP(1), DRIVER_VIEW(2);
        
        private Pipeline(int PipelineID){
            this.PipelineID = PipelineID;
        }

        private int PipelineID;
    }

    public AprilTagsSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        setPipeline(Pipeline.DRIVER_VIEW);
    }

    /**
     * Tells the user if the limelight has identified any valid targets
     * @return whether or not hte limelight has any valid targets
     */
    public boolean isTargetFound() {
        return table.getEntry("tv").getDouble(0) == 0f;
    }

    /**
     * Returns the horizontal offset from the crosshair to the target
     * @return the horizontal offset from crosshair to the target (-29.8 to +29.8 degrees)
     */
    public double getHorizontalOffset() {
        double horizontalOffset = table.getEntry("tx").getDouble(0.0);
        return horizontalOffset;
    }

    /**
     * Returns the vertical offset from the crosshair to the target
     * @return vertical offset from the target from -24.85 to +24.85
     */
    public double getVerticalOffset() {
        double verticalOffset = table.getEntry("ty").getDouble(0);
        return verticalOffset;  
    }

    /**
     * Sets the current pipeline on the limelight to the desired one
     * @param pipeline sets the limelight's current pipeline
     */
    public void setPipeline(Pipeline pipeline) {
        table.getEntry("pipeline").setValue(pipeline.PipelineID);
    }

}
