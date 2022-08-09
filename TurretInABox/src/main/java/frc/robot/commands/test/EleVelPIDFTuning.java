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

public class EleVelPIDFTuning extends CommandBase {
    final Turret turret;
    
    final DriverControls dc;
    
    // NT stuff
    final String NT_NAME = "EleVelPIDTuning";

    final NetworkTableEntry nt_ele_kP;
    final NetworkTableEntry nt_ele_kI;
    final NetworkTableEntry nt_ele_kD;
    final NetworkTableEntry nt_ele_kF;

    /**
     * Creates a new command to move the Azimuth motor at 1 deg/sec with the PID values in the NT.
     * 
     * @param turret The turret to test PIDs for.
     * @param kIAccumReset Whether to reset the integral gain.
     * @param dc The DriverControls.
     */
    public EleVelPIDFTuning(Turret turret, boolean kIAccumReset, DriverControls dc) {
        this.turret = turret;

        this.dc = dc;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_NAME);

        nt_ele_kP = table.getEntry("/AzEle_kP");
        nt_ele_kI = table.getEntry("/AzEle_kI");
        nt_ele_kD = table.getEntry("/AzEle_kD");
        nt_ele_kF = table.getEntry("/AzEle_kF");

        turret.setEleVelPID(new PIDFController(
            nt_ele_kP.getDouble(1),
            nt_ele_kI.getDouble(0),
            nt_ele_kD.getDouble(0),
            nt_ele_kF.getDouble(0)
        ));

        if (kIAccumReset) {
            turret.resetEleVelIGain();
        }

        new MoveTurretFor(turret, 0.0, 1.0, Double.MAX_VALUE);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("**** Final tested PIDF velocity values for elevation motor: kP = " + nt_ele_kP.getDouble(1) + ", kI = " + nt_ele_kI.getDouble(0) + ", kD = " + nt_ele_kD.getDouble(0) + "kF = " + nt_ele_kF.getDouble(0) + " ****");
    }

    @Override
    public boolean isFinished() {
        return dc.bind(DriverControls.Id.Driver, XboxButton.A).get();
    }
}