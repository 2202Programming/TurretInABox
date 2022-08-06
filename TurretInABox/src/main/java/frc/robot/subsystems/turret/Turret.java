// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.CAN2;
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
  
  //commanded values
  double az_cmd;     //[deg]
  double ele_cmd;    //[deg]
  
  // measured 
  double az_pos;    //[deg]
  double ele_pos;   //[deg]
  
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
    NetworkTable nt = NetworkTableInstance.getDefault().getTable(NT_NAME);
  }


  public void  setAzimuthPos(double angle) {
    double adjustedPos = angle;
    
    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, divide by AZIMUTH_GEAR_RATIO because... it's the gear ratio
    double rawCount = adjustedPos / DEGREES * COUNTS_PER_REV / AZIMUTH_GEAR_RATIO;

    azimuthMotor.setSelectedSensorPosition(rawCount);
  }

  public double getAzimuthPos() 
  {
    double rawCount = azimuthMotor.getSelectedSensorPosition();

    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees, multiply by AZIMUTH_GEAR_RATIO because... it's the gear ratio
    double adjustedPos = rawCount / COUNTS_PER_REV * DEGREES * AZIMUTH_GEAR_RATIO;
    
    return adjustedPos;
  }

  public void setElevationPos(double angle) {
    double adjustedPos = angle;
    
    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, divide by ELEVATION_GEAR_RATIO because... it's the gear ratio
    double rawCount = adjustedPos / DEGREES * COUNTS_PER_REV / ELEVATION_GEAR_RATIO;

    elevationMotor.setSelectedSensorPosition(rawCount);
  }
  
  public double getElevationPos() {
    double rawCount = elevationMotor.getSelectedSensorPosition();

    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees, multiply by ELEVATION_GEAR_RATIO because... it's the gear ratio
    double adjustedPos = rawCount / COUNTS_PER_REV * DEGREES * ELEVATION_GEAR_RATIO;
    
    return adjustedPos;
  }

  public void  setAzimuthVel(double velocity) {
    double adjustedVel = velocity;
    
    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, 
    // multiply by VEL_MEASUREMENT_PERIOD to go from /sec to /0.1sec, divide by AZIMUTH_GEAR_RATIO because... it's the gear ratio
    double rawCount = adjustedVel / DEGREES * COUNTS_PER_REV * VEL_MEASUREMENT_PERIOD / AZIMUTH_GEAR_RATIO;

    azimuthMotor.set(rawCount);
  }

  public double getAzimuthVel() 
  {
    double rawCount = azimuthMotor.getSelectedSensorPosition();

    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees, 
    // divide by VEL_MEASUREMENT_PERIOD to go from /0.1sec to sec, multiply by AZIMUTH_GEAR_RATIO because... it's the gear ratio
    double adjustedVel = rawCount / COUNTS_PER_REV * DEGREES / VEL_MEASUREMENT_PERIOD * AZIMUTH_GEAR_RATIO;
    
    return adjustedVel;
  }

  public void setElevationVel(double velocity) {
    double adjustedVel = velocity;
    
    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, 
    // multiply by VEL_MEASUREMENT_PERIOD to go from /sec to /0.1sec, divide by ELEVATION_GEAR_RATIO because... it's the gear ratio
    double rawCount = adjustedVel / DEGREES * COUNTS_PER_REV * VEL_MEASUREMENT_PERIOD / ELEVATION_GEAR_RATIO;

    elevationMotor.set(rawCount);
  }
  
  public double getElevationVel() {
    double rawCount = elevationMotor.getSelectedSensorPosition();

    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees, 
    // divide by VEL_MEASUREMENT_PERIOD to go from /0.1sec to sec, multiply by ELEVATION_GEAR_RATIO because... it's the gear ratio
    double adjustedVel = rawCount / COUNTS_PER_REV * DEGREES / VEL_MEASUREMENT_PERIOD * ELEVATION_GEAR_RATIO;
    
    return adjustedVel;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
