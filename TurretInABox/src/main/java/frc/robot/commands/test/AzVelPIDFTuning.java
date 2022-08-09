package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.turret.MoveTurretFor;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.PIDFController;

public class AzVelPIDFTuning extends CommandBase {
    final Turret turret;

    final DriverControls dc;
    
    // NT stuff
    final String NT_NAME = "AzVelPIDTuning";
    
    final NetworkTableEntry nt_az_kP;
    final NetworkTableEntry nt_az_kI;
    final NetworkTableEntry nt_az_kD;
    final NetworkTableEntry nt_az_kF;

    /**
     * Creates a new command to move the Azimuth motor at 1 deg/sec with the PID values in the NT.
     * 
     * @param turret The turret to test PIDs for.
     * @param kIAccumReset Whether to reset the integral gain.
     * @param dc The DriverControls.
     */
    public AzVelPIDFTuning(Turret turret, boolean kIAccumReset, DriverControls dc) {
        this.turret = turret;

        this.dc = dc;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_NAME);

        nt_az_kP = table.getEntry("/AzPos_kP");
        nt_az_kI = table.getEntry("/AzPos_kI");
        nt_az_kD = table.getEntry("/AzPos_kD");
        nt_az_kF = table.getEntry("/AzPos_kF");

        turret.setAzVelPID(new PIDFController(
            nt_az_kP.getDouble(1.0),
            nt_az_kI.getDouble(0.0),
            nt_az_kD.getDouble(0.0),
            nt_az_kF.getDouble(0.0)
        ));

        if (kIAccumReset) {
            turret.resetAzVelIGain();
        }

        new MoveTurretFor(turret, 1.0, 0.0, Double.MAX_VALUE);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("**** Final tested PIDF velocity values for elevation motor: kP = " + nt_az_kP.getDouble(1) + ", kI = " + nt_az_kI.getDouble(0) + ", kD = " + nt_az_kD.getDouble(0) + "kF = " + nt_az_kF.getDouble(0) + " ****");
    }

    @Override
    public boolean isFinished() {
        return dc.bind(DriverControls.Id.Driver, XboxButton.A).get();
    }
}