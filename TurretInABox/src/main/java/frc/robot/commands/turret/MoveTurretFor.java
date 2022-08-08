package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class MoveTurretFor extends CommandBase {
    
    final Turret turret;
    // Velocities to move at
    final double az;
    final double ele;
    final double sec;
    final double SCHEDULER_MULTIPLIER = 50;
    int count = 0;

    /**
     * Creates a new command to move the turret at the given velocities (in deg/sec) for the given amount of seconds.
     * 
     * @param turret The turret.
     * @param az_vel The velocity to move the azimuth motor at.
     * @param ele_vel The velocity to move the elvation motor at.
     * @param sec The time for the command to last for.
     */
    public MoveTurretFor(Turret turret, double az_vel, double ele_vel, double sec) {
        this.turret = turret;
        this.az = az_vel;
        this.ele = ele_vel;
        this.sec = sec;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setAzDesPos(az);
        turret.setEleDesPos(ele);
        System.out.println("**** Moving az at " + az + "deg/sec, ele at " + ele + " deg/sec for " + sec + "sec ****");
    }

    @Override
    public void end (boolean interrupted) {
        if (!interrupted) System.out.println("**** Move turret vel command completed ****");
        else System.out.println("**** Move turret vel command interrupted ****");
    }

    @Override
    public boolean isFinished() {
        count++;
        return ((count / SCHEDULER_MULTIPLIER) >= sec);
    }
}
