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

  public enum DesiredMotor {
    Azimuth,
    Elevation,
  }

  ControlMode az_control_mode;
  ControlMode ele_control_mode;

  final int ENUM_ADJ_FACTOR = -1;
  
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
  PIDFController az_pos_PID = new PIDFController(1.0, 0.0, 0.0, 0.0);
  PIDFController ele_pos_PID = new PIDFController(1.0, 0.0, 0.0, 0.0);
  int kSlotPos = 0;

  PIDFController az_vel_PID = new PIDFController(1.0, 0.0, 0.0, 0.0);
  PIDFController ele_vel_PID = new PIDFController(1.0, 0.0, 0.0, 0.0);
  int kSlotVel = 1;

  public Turret(ControlMode defaultMode) {
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

    az_pos_PID.copyTo(azimuthMotor, kSlotPos);
    ele_pos_PID.copyTo(elevationMotor, kSlotPos);
    az_vel_PID.copyTo(azimuthMotor, kSlotVel);
    ele_vel_PID.copyTo(elevationMotor, kSlotVel);

    az_control_mode = defaultMode;
    ele_control_mode = defaultMode;

    // NT and NTEs
    NTConfig();
  }

  @Override
  public void periodic() {
    az_pos = getVal(DesiredMotor.Azimuth, ControlMode.Position);
    az_vel = getVal(DesiredMotor.Azimuth, ControlMode.Velocity);
    ele_pos = getVal(DesiredMotor.Elevation, ControlMode.Position);
    ele_vel = getVal(DesiredMotor.Elevation, ControlMode.Velocity);
    NTUpdate();
  }

  /**
   * Sets the control mode of the indicated motors to the indicated control mode.
   * 
   * @param setAz Whether or not to set the azimuth motor control mode to the given mode.
   * @param motor Which motor to set the mode for.
   */
  public void setMode(DesiredMotor motor, ControlMode mode) {
    switch (motor) {
      case Azimuth: az_control_mode = mode;
      case Elevation: ele_control_mode = mode;
    }
  }

  /**
   * Checks if the actual position of the azimuth and elevation motors are close enough to the commanded positions.
   * 
   * @param checkAz Whether to check if azimuth position is within tolerance of commanded position.
   * @param checkEle Whether to check if elevation position is within tolerance of commanded position.
   * @return True if delta between commanded and actual positions is within tolerances, false otherwise.
   */
  public boolean checkIsFinishedPos(boolean checkAz, boolean checkEle) {
    return (checkAz ? (Math.abs(getVal(DesiredMotor.Azimuth, ControlMode.Position) - az_pos_cmd) <= TurretConst.TOLERANCE_AZ) : true)
            && (checkEle ? (Math.abs(getVal(DesiredMotor.Elevation, ControlMode.Position) - ele_pos_cmd) <= TurretConst.TOLERANCE_ELE) : true);
  }



  /*
   * Note on the nomenclature for the get/set methods below:
   * 
   * get: Get a value
   * set: Set a value
   * reset: Reset a value to 0
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
   * Sets the PIDF of the given mode for the given motor to the given PIDF controller.
   * 
   * @param motor The motor to set the PIDF value of.
   * @param mode The mode to set the PIDF value of.
   * @param controller The values to set the PIDF to.
   */
  public void setPIDF(DesiredMotor motor, ControlMode mode, PIDFController controller) {
    switch (motor) {
      case Azimuth: controller.copyTo(azimuthMotor, mode.value - ENUM_ADJ_FACTOR);
      case Elevation: controller.copyTo(elevationMotor, mode.value - ENUM_ADJ_FACTOR);
    }
  }

  /**
   * Sets the integral gain of the PIDF loop for the given mode of the given motor to the given value.
   * 
   * @param motor The motor to set the integral gain of.
   * @param mode The mode to set the integral gain of.
   * @param IGain The value to set the integral gain to.
   */
  public void setIGain(DesiredMotor motor, ControlMode mode, double IGain) {
    switch (motor) {
      case Azimuth: 
        if (mode == ControlMode.Position) az_pos_PID.setIzone(IGain); else if (mode == ControlMode.Velocity) az_vel_PID.setIzone(IGain);
      case Elevation:
        if (mode == ControlMode.Position) ele_pos_PID.setIzone(IGain); else if (mode == ControlMode.Velocity) ele_vel_PID.setIzone(IGain);
    }
  }

  /**
   * Sets the position of the internal encoder of the given motor to the given angle (in degrees).
   * 
   * @param motor The motor to set the position of.
   * @param angle The angle to set the position to.
   */
  public void setPos(DesiredMotor motor, double angle) {
    double adjustedPos = angle;
    
    switch (motor) {
      // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, divide by AZIMUTH_GEAR_RATIO because... it's the gear ratio
      case Azimuth: azimuthMotor.setSelectedSensorPosition(adjustedPos / DEGREES * COUNTS_PER_REV / AZIMUTH_GEAR_RATIO);
      // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, divide by ELEVATION_GEAR_RATIO because... it's the gear ratio
      case Elevation: elevationMotor.setSelectedSensorPosition(adjustedPos / DEGREES * COUNTS_PER_REV / ELEVATION_GEAR_RATIO);
    }
  }

  /**
   * Sets the commanded value to the given value for the given motor and the given control mode.
   * 
   * @param motor The motor to command.
   * @param mode The mode to command (Position = deg, Velocity = deg/sec).
   * @param value The value to command to.
   */
  public void setDesVal(DesiredMotor motor, ControlMode mode, double value) {
    double adjustedVal = value;
    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts
    double adjustmentFactor = COUNTS_PER_REV / DEGREES;
    // multiply by VEL_MEASUREMENT_PERIOD to go from /sec to /0.1sec
    if (mode == ControlMode.Velocity) adjustmentFactor *= VEL_MEASUREMENT_PERIOD;

    switch (motor) {
      // divide by AZIMUTH_GEAR_RATIO because... it's the gear ratio
      case Azimuth: azimuthMotor.set(mode, adjustedVal * adjustmentFactor / AZIMUTH_GEAR_RATIO); az_control_mode = mode;
      // divide by ELEVATION_GEAR_RATIO because... it's the gear ratio
      case Elevation: elevationMotor.set(mode, adjustedVal * adjustmentFactor / ELEVATION_GEAR_RATIO); ele_control_mode = mode;
    }
  }

  /**
   * Gets the value of the specified motor and mode.
   * 
   * @param motor The motor to get the value of.
   * @param mode The mode to get the value of (Position = deg, Velocity = deg/sec).
   */
  public double getVal(DesiredMotor motor, ControlMode mode) {
    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees
    double adjustmentFactor = DEGREES / COUNTS_PER_REV;
    // divide by VEL_MEASUREMENT_PERIOD to go from /0.1sec to sec
    if (mode == ControlMode.Velocity) adjustmentFactor /= VEL_MEASUREMENT_PERIOD;

    switch (motor) {
      // multiply by AZIMUTH_GEAR_RATIO because... it's the gear ratio
      case Azimuth: 
        if (mode == ControlMode.Position) return (azimuthMotor.getSelectedSensorPosition() * adjustmentFactor * AZIMUTH_GEAR_RATIO); 
        else if (mode == ControlMode.Velocity) return (azimuthMotor.getSelectedSensorVelocity() * adjustmentFactor * AZIMUTH_GEAR_RATIO);
      // multiply by ELEVATION_GEAR_RATIO because... it's the gear ratio
      case Elevation:
      if (mode == ControlMode.Position) return (elevationMotor.getSelectedSensorPosition() * adjustmentFactor * ELEVATION_GEAR_RATIO); 
      else if (mode == ControlMode.Velocity) return (elevationMotor.getSelectedSensorVelocity() * adjustmentFactor * ELEVATION_GEAR_RATIO);
    }

    return Double.NaN;
  }

  // NT stuff
  private NetworkTable table;
  private NetworkTableEntry nt_az_pos;
  private NetworkTableEntry nt_az_vel;
  private NetworkTableEntry nt_ele_pos;
  private NetworkTableEntry nt_ele_vel;
  private NetworkTableEntry nt_az_control_mode;
  private NetworkTableEntry nt_ele_control_mode;

  void NTConfig() {
    table = NetworkTableInstance.getDefault().getTable(NT_NAME);
    nt_az_pos = table.getEntry("/azimuth_position");
    nt_az_vel = table.getEntry("/azimuth_velocity");
    nt_az_control_mode = table.getEntry("/azimuth_control_mode");

    nt_ele_pos = table.getEntry("/elevation_position");
    nt_ele_vel = table.getEntry("/elevation_velocity");
    nt_ele_control_mode = table.getEntry("/elevation_control_mode");
  }

  void NTUpdate() {
    nt_az_pos.setDouble(az_pos);
    nt_az_vel.setDouble(az_vel);
    nt_az_control_mode.setString(az_control_mode.toString());

    nt_ele_pos.setDouble(ele_pos);
    nt_ele_vel.setDouble(ele_vel);
    nt_ele_control_mode.setString(ele_control_mode.toString());
  }
}
