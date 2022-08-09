package frc.robot.commands.test;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.MoveTurretTo;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.PIDFController;

public class ElePosPIDFTuning extends SequentialCommandGroup {
    final Turret turret;

    final DriverControls dc;
    
    // NT stuff
    final String NT_NAME = "ElePosPIDTuning";
    
    final NetworkTableEntry nt_kP;
    final NetworkTableEntry nt_kI;
    final NetworkTableEntry nt_kD;
    final NetworkTableEntry nt_kF;

    /**
     * Creates a new command to oscillate the elevation motor between 0 and 180 with the PID values in the NT.
     * 
     * @param turret The turret to test PIDs for.
     * @param kIAccumReset Whether to reset the integral gain.
     * @param dc The DriverControls.
     */
    public ElePosPIDFTuning(Turret turret, boolean kIAccumReset, DriverControls dc) {
        this.turret = turret;

        this.dc = dc;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_NAME);

        nt_kP = table.getEntry("/kP");
        nt_kI = table.getEntry("/kI");
        nt_kD = table.getEntry("/kD");
        nt_kF = table.getEntry("/kF");

        turret.setElePosPID(new PIDFController(
            nt_ele_kP.getDouble(1.0),
            nt_ele_kI.getDouble(0.0),
            nt_ele_kD.getDouble(0.0),
            nt_ele_kF.getDouble(0.0)
        ));

        if (kIAccumReset) {
            turret.resetElePosIGain();
        }
        
        // I'm sure this is enough
        this.addCommands(
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 0.0, 180.0),
        new MoveTurretTo(turret, 0.0, 0.0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("**** Final tested PIDF velocious values for elevation motor: kP = " + nt_kP.getDouble(1) + ", kI = " + nt_kI.getDouble(0) + ", kD = " + nt_kD.getDouble(0) + ", kF = " + nt_kF.getDouble(0) + " ****");
    }

    @Override
    public boolean isFinished() {
        return dc.bind(DriverControls.Id.Driver, XboxButton.A).get();
    }
}