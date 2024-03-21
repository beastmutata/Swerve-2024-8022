package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable table;
    private NetworkTableEntry tx, ty, ta, tv;

    public Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        tv = table.getEntry("tv");
    }

   public void trackAprilTag(Swerve drive_swerve) {
    double x = tx.getDouble(0.0); // Horizontal offset from Limelight
    boolean hasTarget = tv.getDouble(0) == 1; // Check if a target is visible

    if (hasTarget) {
        final double kP = -0.1; // Proportional control constant for rotation, adjust based on testing
        double rotationAdjustment = x * kP; // Calculate rotation adjustment

        // Translate rotation adjustment into differential angles for swerve modules
        // This simplistic approach assumes a basic rotational movement can be achieved by inversely adjusting angles
        // For a real implementation, the distribution and magnitude of speed adjustments would need tuning
        double[] angles = new double[4];
        for (int i = 0; i < angles.length; i++) {
            // Assuming modules are positioned such that modules 0 & 1 rotate the robot one way and 2 & 3 the opposite way
            if (i < 2) angles[i] = rotationAdjustment; // Modules 0 & 1
            else angles[i] = -rotationAdjustment; // Modules 2 & 3
        }
        //drive_swerve.setHeadings(angles);
    }
}


}
