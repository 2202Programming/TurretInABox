package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConst;

public class TurretAxes extends SubsystemBase {
    // TODO: stop being lazy and privatize (no don't sell it) motor, create methods

    /**
     * The azimuth axis (horizontal spin).
     */
    public TurretAxis azimuth = new TurretAxis("az", TurretConst.AZIMUTH_GEAR_RATIO);

    /**
     * Creates a new TurretAxes object.
     */
    public TurretAxes() {
        System.out.println("***** Turret Axes Created *****");
    }

    /**
     * Checks whether the position of the azimuth motor is within tolerances.
     * @return Whether the position of the azimuth motor is satisfactory compared to the desired position.
     */
    public boolean checkIsFinished() {
        return (Math.abs(azimuth.getPos()) <= TurretConst.TOLERANCE_AZ);
      }
}
