package frc.robot.commands.test;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hid.XboxButton;
import frc.robot.subsystems.ifx.DriverControls;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.DesiredMotor;
import frc.robot.util.PIDFController;

public class PIDFTuning extends CommandBase {
    final Turret turret;

    final DriverControls dc;
    
    // NT stuff
    final String NT_NAME = "PIDTuning";
    
    final NetworkTableEntry nt_kP;
    final NetworkTableEntry nt_kI;
    final NetworkTableEntry nt_kD;
    final NetworkTableEntry nt_kF;

    final ControlMode controlMode;
    final MotorType motorType;

    /**
     * Creates a new command to oscillate the azimuth motor between 0 and 180 with the PID values in the NT.
     * 
     * @param turret The turret to test PIDs for.
     * @param kIAccumReset Whether to reset the integral gain.
     * @param dc The DriverControls.
     */
    public PIDFTuning(Turret turret, boolean kIAccumReset, DriverControls dc, ControlMode controlMode, MotorType motorType) {
        this.turret = turret;
        
        this.dc = dc;

        this.controlMode = controlMode;
        this.motorType = motorType;

        String NTPrefix = "/" + motorType.toString() + "/" + controlMode.toString();

        NetworkTable table = NetworkTableInstance.getDefault().getTable(NT_NAME);

        nt_kP = table.getEntry(NTPrefix + "/kP");
        nt_kI = table.getEntry(NTPrefix + "/kI");
        nt_kD = table.getEntry(NTPrefix + "/kD");
        nt_kF = table.getEntry(NTPrefix + "/kF");

        turret.setPIDF(DesiredMotor.Azimuth, ControlMode.Position, new PIDFController(
            nt_kP.getDouble(1.0),
            nt_kI.getDouble(0.0),
            nt_kD.getDouble(0.0),
            nt_kF.getDouble(0.0)
        ));

        if (kIAccumReset) {
            turret.setIGain(DesiredMotor.Azimuth, ControlMode.Position, 0.0);
        }
        
        if (controlMode == ControlMode.Velocity) {
            new MoveTurretForLoop(turret);
        } else {
            new MoveTurretToLoop(turret);
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("**** Final tested PIDF " + controlMode.toString() + " values for " + motorType.toString() + " motor: kP = " + 
                        nt_kP.getDouble(1) + ", kI = " + nt_kI.getDouble(0) + ", kD = " + nt_kD.getDouble(0) + "kF = " + nt_kF.getDouble(0) + " ****");
    }

    @Override
    public boolean isFinished() {
        return dc.bind(DriverControls.Id.Driver, XboxButton.A).get();
    }
}