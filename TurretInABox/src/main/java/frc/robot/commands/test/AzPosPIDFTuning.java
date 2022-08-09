package frc.robot.commands.test;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.turret.MoveTurretTo;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.DesiredMotor;
import frc.robot.util.PIDFController;

public class AzPosPIDFTuning extends SequentialCommandGroup {
    final Turret turret;

    final DriverControls dc;
    
    // NT stuff
    final String NT_NAME = "AzPosPIDTuning";
    
    final NetworkTableEntry nt_az_kP;
    final NetworkTableEntry nt_az_kI;
    final NetworkTableEntry nt_az_kD;
    final NetworkTableEntry nt_az_kF;

    /**
     * Creates a new command to oscillate the azimuth motor between 0 and 180 with the PID values in the NT.
     * 
     * @param turret The turret to test PIDs for.
     * @param kIAccumReset Whether to reset the integral gain.
     * @param dc The DriverControls.
     */
    public AzPosPIDFTuning(Turret turret, boolean kIAccumReset, DriverControls dc) {
        this.turret = turret;

        this.dc = dc;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_NAME);

        nt_az_kP = table.getEntry("/kP");
        nt_az_kI = table.getEntry("/kI");
        nt_az_kD = table.getEntry("/kD");
        nt_az_kF = table.getEntry("/kF");

        turret.setPIDF(DesiredMotor.Azimuth, ControlMode.Position, new PIDFController(
            nt_az_kP.getDouble(1.0),
            nt_az_kI.getDouble(0.0),
            nt_az_kD.getDouble(0.0),
            nt_az_kF.getDouble(0.0)
        ));

        if (kIAccumReset) {
            turret.setIGain(DesiredMotor.Azimuth, ControlMode.Position, 0.0);
        }
        
        // I'm sure this is enough
        this.addCommands(
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0),
        new MoveTurretTo(turret, 180.0, 0.0),
        new MoveTurretTo(turret, 0.0, 0.0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("**** Final tested PIDF position values for azimuth motor: kP = " + nt_az_kP.getDouble(1) + ", kI = " + nt_az_kI.getDouble(0) + ", kD = " + nt_az_kD.getDouble(0) + "kF = " + nt_az_kF.getDouble(0) + " ****");
    }

    @Override
    public boolean isFinished() {
        return dc.bind(DriverControls.Id.Driver, XboxButton.A).get();
    }
}