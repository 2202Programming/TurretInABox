// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.test;

import frc.robot.commands.turret.MoveTurretTo;
import frc.robot.subsystems.turret.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class MoveTurretToLoop extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Turret turret;
  Command moveCmd;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveTurretToLoop(Turret turret) {
    this.turret = turret;

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    moveCmd = new SequentialCommandGroup(new MoveTurretTo(turret, 180, 0), new MoveTurretTo(turret, 0, 0));
    this.addCommands(moveCmd);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (moveCmd.isFinished()) { moveCmd = new SequentialCommandGroup(new MoveTurretTo(turret, 180, 0), new MoveTurretTo(turret, 0, 0)); this.addCommands(moveCmd);}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
