package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.turret.TurretAxes;
import frc.robot.util.PIDFController;

public class PIDFTuning extends CommandBase {
    final TurretAxes turret;

    final DriverControls dc;
    
    // NT stuff
    final String NT_NAME = "PIDTuning";
    
    final NetworkTableEntry nt_kP;
    final NetworkTableEntry nt_kI;
    final NetworkTableEntry nt_kD;
    final NetworkTableEntry nt_kF;

    /**
     * Creates a new command to oscillate the azimuth motor between 0 and 180 with the PID values in the NT.
     * 
     * @param turret The turret to test PIDs for.
     * @param kIAccumReset Whether to reset the integral gain.
     * @param dc The DriverControls.
     */
    public PIDFTuning(TurretAxes turret, boolean kIAccumReset, DriverControls dc) {
        this.turret = turret;
        
        this.dc = dc;

        String NTPrefix = "/";

        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_NAME);

        nt_kP = table.getEntry(NTPrefix + "/kP");
        nt_kI = table.getEntry(NTPrefix + "/kI");
        nt_kD = table.getEntry(NTPrefix + "/kD");
        nt_kF = table.getEntry(NTPrefix + "/kF");

        turret.azimuth.setPIDF(new PIDFController(
            nt_kP.getDouble(1.0),
            nt_kI.getDouble(0.0),
            nt_kD.getDouble(0.0),
            nt_kF.getDouble(0.0)
        ));

        if (kIAccumReset) {
            turret.azimuth.setIGain(0.0);
        }
            new MoveTurretPosLoop(turret);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("**** Final tested PIDF values for azimuth motor: kP = " + 
                        nt_kP.getDouble(1) + ", kI = " + nt_kI.getDouble(0) + ", kD = " + nt_kD.getDouble(0) + "kF = " + nt_kF.getDouble(0) + " ****");
    }

    @Override
    public boolean isFinished() {
        return dc.bind(DriverControls.Id.Driver, XboxButton.A).get();
    }
}