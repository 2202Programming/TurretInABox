// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.CAN2;
import frc.robot.Constants.TurretConst;
import frc.robot.util.PIDFController;

import static frc.robot.Constants.TurretConst.AZIMUTH_GEAR_RATIO;
import static frc.robot.Constants.TurretConst.ELEVATION_GEAR_RATIO;
import static frc.robot.Constants.TurretConst.BUSTIMEOUT;
import static frc.robot.Constants.TurretConst.NT_NAME;

public class Turret extends SubsystemBase {
  // Talon constanst
  final double COUNTS_PER_REV = 4096;
  final double DEGREES = 360;
  final double VEL_MEASUREMENT_PERIOD = 0.1;

  // commanded pos values
  double az_pos_cmd; // [deg]
  double ele_pos_cmd; // [deg]

  // measured pos values
  double az_pos; // [deg]
  double ele_pos; // [deg]

  // commanded vel values
  double az_vel_cmd; // [deg/s]
  double ele_vel_cmd; // [deg/s]

  // measured vel values
  double az_vel; // [deg/s]
  double ele_vel; // [deg/s]

  // falcons
  WPI_TalonFX azimuthMotor;
  WPI_TalonFX elevationMotor;

  // sensor collections
  TalonFXSensorCollection az_sensors;
  TalonFXSensorCollection ele_sensors;

  // PID stuff
  PIDFController az_PID = new PIDFController(1.0, 0.0, 0.0, 0.0);
  PIDFController ele_PID = new PIDFController(1.0, 0.0, 0.0, 0.0);
  int kSlot = 0;

  public Turret() {
    azimuthMotor = new WPI_TalonFX(CAN2.AZIMUTH, CAN2.BUSNAME);
    elevationMotor = new WPI_TalonFX(CAN2.ELEVATION, CAN2.BUSNAME);

    az_sensors = azimuthMotor.getSensorCollection();
    ele_sensors = elevationMotor.getSensorCollection();

    azimuthMotor.configFactoryDefault();
    elevationMotor.configFactoryDefault();

    azimuthMotor.setNeutralMode(NeutralMode.Brake);
    elevationMotor.setNeutralMode(NeutralMode.Brake);

    az_sensors.setIntegratedSensorPositionToAbsolute(BUSTIMEOUT);
    ele_sensors.setIntegratedSensorPositionToAbsolute(BUSTIMEOUT);

    az_PID.copyTo(azimuthMotor, kSlot);
    ele_PID.copyTo(elevationMotor, kSlot);

    // NT and NTEs
    NTConfig();
  }

  @Override
  public void periodic() {
    az_pos = getAzPos();
    az_vel = getAzVel();
    ele_pos = getElePos();
    ele_vel = getEleVel();
    NTUpdate();
  }

  public boolean checkIsFinishedPos() {
    return ((Math.abs(this.getAzPos() - az_pos_cmd) <= TurretConst.TOLERANCE_AZ)
            && (Math.abs(this.getElePos() - ele_pos_cmd) <= TurretConst.TOLERANCE_ELE));
  }

  /*
   * Note on the nomenclature for the get/set methods below:
   * 
   * get: Get a value
   * set: Set a value
   * 
   * Az: Azimuth motor
   * Ele: Elevation motor
   * 
   * Des: Desired (i.e. command the motor)
   * N/A: Directly set encoder value
   * 
   * Pos: Position
   * Vel: Velocity
   */

  /**
   * Sets the internal positional encoder of the azimuth motor to the specified
   * degree.
   * 
   * @param angle The angle (in deg) to set the encoder to.
   */
  public void setAzPos(double angle) {
    double adjustedPos = angle;

    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, divide by AZIMUTH_GEAR_RATIO because... it's the gear ratio
    double rawCount = adjustedPos / DEGREES * COUNTS_PER_REV / AZIMUTH_GEAR_RATIO;

    azimuthMotor.setSelectedSensorPosition(rawCount);
  }

  /**
   * Sets the desired position of the azimuth motor to the specified degree.
   * 
   * @param angle The angle (in deg) to set the desired position to.
   */
  public void setAzDesPos(double angle) {
    double adjustedPos = angle;

    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, divide by AZIMUTH_GEAR_RATIO because... it's the gear ratio
    double rawCount = adjustedPos / DEGREES * COUNTS_PER_REV / AZIMUTH_GEAR_RATIO;

    azimuthMotor.set(ControlMode.Position, rawCount);

    az_pos_cmd = adjustedPos;
    az_vel_cmd = Double.NaN; // TODO: is this right?
  }

  /**
   * Gets the position of the internal encoder of the azimuth motor in degrees.
   * 
   * @return The position of the encoder (in degrees).
   */
  public double getAzPos() {
    double rawCount = azimuthMotor.getSelectedSensorPosition();

    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees, multiply by AZIMUTH_GEAR_RATIO because... it's the gear ratio
    double adjustedPos = rawCount / COUNTS_PER_REV * DEGREES * AZIMUTH_GEAR_RATIO;

    return adjustedPos;
  }

  /**
   * Sets the internal positional encoder of the elevation motor to the specified degree.
   * 
   * @param angle The angle (in deg) to set the encoder to.
   */
  public void setElePos(double angle) {
    double adjustedPos = angle;

    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to
    // encoder counts, divide by ELEVATION_GEAR_RATIO because... it's the gear ratio
    double rawCount = adjustedPos / DEGREES * COUNTS_PER_REV / ELEVATION_GEAR_RATIO;

    elevationMotor.setSelectedSensorPosition(rawCount);
  }

  /**
   * Sets the desired position of the elevation motor to the specified degree.
   * 
   * @param angle The angle (in deg) to set the desired position to.
   */
  public void setEleDesPos(double angle) {
    double adjustedPos = angle;

    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, divide by ELEVATION_GEAR_RATIO because... it's the gear ratio
    double rawCount = adjustedPos / DEGREES * COUNTS_PER_REV / ELEVATION_GEAR_RATIO;

    elevationMotor.set(ControlMode.Position, rawCount);

    ele_pos_cmd = adjustedPos;
    ele_vel_cmd = Double.NaN; // TODO: is this right?
  }

  /**
   * Gets the position of the internal encoder of the elevation motor in degrees.
   * 
   * @return The position of the encoder (in degrees).
   */
  public double getElePos() {
    double rawCount = elevationMotor.getSelectedSensorPosition();

    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees, multiply by ELEVATION_GEAR_RATIO because... it's the gear ratio
    double adjustedPos = rawCount / COUNTS_PER_REV * DEGREES * ELEVATION_GEAR_RATIO;

    return adjustedPos;
  }

  /**
   * Sets the desired velocity of the azimuth motor to the specified deg/sec.
   * 
   * @param velocity The velocity (in deg/sec) to set the desired velocity to.
   */
  public void setAzDesVel(double velocity) {
    double adjustedVel = velocity;

    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts,
    // multiply by VEL_MEASUREMENT_PERIOD to go from /sec to /0.1sec, divide by AZIMUTH_GEAR_RATIO because... it's the gear ratio
    double rawCount = adjustedVel / DEGREES * COUNTS_PER_REV * VEL_MEASUREMENT_PERIOD / AZIMUTH_GEAR_RATIO;

    azimuthMotor.set(rawCount);

    az_vel_cmd = adjustedVel;
    az_pos_cmd = Double.NaN; // TODO: is this right?
  }

  /**
   * Gets the velocity of the internal encoder of the azimuth motor in deg/sec.
   * 
   * @return The velocity of the encoder (in deg/sec).
   */
  public double getAzVel() {
    double rawCount = azimuthMotor.getSelectedSensorPosition();

    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees,
    // divide by VEL_MEASUREMENT_PERIOD to go from /0.1sec to sec, multiply by AZIMUTH_GEAR_RATIO because... it's the gear ratio
    double adjustedVel = rawCount / COUNTS_PER_REV * DEGREES / VEL_MEASUREMENT_PERIOD * AZIMUTH_GEAR_RATIO;

    return adjustedVel;
  }

  /**
   * Sets the desired velocity of the elevation motor to the specified deg/sec.
   * 
   * @param velocity The velocity (in deg/sec) to set the desired velocity to.
   */
  public void setEleDesVel(double velocity) {
    double adjustedVel = velocity;

    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts,
    // multiply by VEL_MEASUREMENT_PERIOD to go from /sec to /0.1sec, divide by ELEVATION_GEAR_RATIO because... it's the gear ratio
    double rawCount = adjustedVel / DEGREES * COUNTS_PER_REV * VEL_MEASUREMENT_PERIOD / ELEVATION_GEAR_RATIO;

    elevationMotor.set(ControlMode.Velocity, rawCount);

    ele_vel_cmd = adjustedVel;
    ele_pos_cmd = Double.NaN; // TODO: is this right?
  }

  /**
   * Gets the velocity of the internal encoder of the elevation motor in deg/sec.
   * 
   * @return The velocity of the encoder (in deg/sec).
   */
  public double getEleVel() {
    double rawCount = elevationMotor.getSelectedSensorPosition();

    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees,
    // divide by VEL_MEASUREMENT_PERIOD to go from /0.1sec to sec, multiply by ELEVATION_GEAR_RATIO because... it's the gear ratio
    double adjustedVel = rawCount / COUNTS_PER_REV * DEGREES / VEL_MEASUREMENT_PERIOD * ELEVATION_GEAR_RATIO;

    return adjustedVel;
  }

  // NT stuff
  private NetworkTable table;
  private NetworkTableEntry nt_az_pos;
  private NetworkTableEntry nt_az_vel;
  private NetworkTableEntry nt_ele_pos;
  private NetworkTableEntry nt_ele_vel;

  void NTConfig() {
    table = NetworkTableInstance.getDefault().getTable(NT_NAME);
    nt_az_pos = table.getEntry("/azimuth_position");
    nt_az_vel = table.getEntry("/azimuth_velocity");
    nt_ele_pos = table.getEntry("/elevation_position");
    nt_ele_vel = table.getEntry("/elevation_velocity");
  }

  void NTUpdate() {
    nt_az_pos.setDouble(az_pos);
    nt_az_vel.setDouble(az_vel);
    nt_ele_pos.setDouble(ele_pos);
    nt_ele_vel.setDouble(ele_vel);
  }
}
