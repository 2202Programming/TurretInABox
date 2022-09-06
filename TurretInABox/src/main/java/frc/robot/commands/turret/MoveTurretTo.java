package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.TurretAxes;

public class MoveTurretTo extends CommandBase {
    
    final TurretAxes turret;
    // Positions to go to
    final double az;
    final double ele;

    /**
     * Creates a new command to move the turret to the given positions (in degrees).
     * 
     * @param turret The turret.
     * @param az_pos The azimuth position to move to.
     * @param ele_pos The elvation position to move to.
     */
    public MoveTurretTo(TurretAxes turret, double az_pos, double ele_pos) {
        this.turret = turret;
        this.az = az_pos;
        this.ele = ele_pos;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.azimuth.setDesPos(az);
        System.out.println("**** Moving az to " + az + "deg, ele to " + ele + "deg ****");
    }

    @Override
    public void end (boolean interrupted) {
        if (!interrupted) System.out.println("**** Move turret pos command completed ****");
        else System.out.println("**** Move turret pos command interrupted ****");
    }

    @Override
    public boolean isFinished() {
        return turret.checkIsFinished();
    }
}
