// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import frc.robot.commands.turret.MoveTurretTo;
import frc.robot.subsystems.turret.TurretAxes;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MoveTurretPosLoop extends CommandBase {
  private final TurretAxes turret;
  Command moveLeft;
  Command moveRight;
  Command currentScheduled;

  /**
   * Creates a new ExampleCommand.
   *
   * @param turret The turret used by this command.
   */
  public MoveTurretPosLoop(TurretAxes turret) {
    this.turret = turret;

    addRequirements(this.turret);

    moveLeft = new MoveTurretTo(this.turret, 180, 0);
    moveRight = new MoveTurretTo(this.turret, 0, 0);
  }

  @Override
  public void initialize() {
    currentScheduled = moveLeft;
    currentScheduled.schedule();
  }

  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentScheduled.cancel();
    System.out.println("***** Current command (cancelled): " + currentScheduled.toString() + "****");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (currentScheduled.isFinished()) {
      currentScheduled = (currentScheduled == moveLeft) ? moveRight : moveLeft;
      currentScheduled.schedule();
    }

    return false;
  }
}
