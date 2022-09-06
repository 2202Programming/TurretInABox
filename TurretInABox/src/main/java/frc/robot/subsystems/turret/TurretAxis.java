// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.CAN2;
import frc.robot.Constants.TurretConst;
import frc.robot.util.PIDFController;

import static frc.robot.Constants.TurretConst.BUSTIMEOUT;
import static frc.robot.Constants.TurretConst.NT_NAME;

public class TurretAxis extends SubsystemBase {

  // Talon constansts
  final double COUNTS_PER_REV = 4096;
  final double DEGREES = 360;
  final double VEL_MEASUREMENT_PERIOD = 0.1; // [s] or 100ms
  final double GEAR_RATIO;

  // NT prefix / what axis
  final String NTPrefix;

  // Relevant position and velocity values
  double pos_cmd; // [deg] Commanded pos value
  double pos; // [deg] Measured pos value
  double vel; // [deg/s] Measured vel value

  // The motor
  WPI_TalonFX motor;

  // sensor collections
  TalonFXSensorCollection sensors; // sensor collection

  // PID stuff
  PIDFController pidf = new PIDFController(1.0, 0.0, 0.0, 0.0);

  public TurretAxis(String NTPrefix, double gearRatio) {
    this.NTPrefix = NTPrefix;

    this.GEAR_RATIO = gearRatio;

    motor = new WPI_TalonFX(CAN2.AZIMUTH, CAN2.BUSNAME);

    sensors = motor.getSensorCollection();

    motor.configFactoryDefault();

    motor.setNeutralMode(NeutralMode.Brake);

    sensors.setIntegratedSensorPositionToAbsolute(BUSTIMEOUT);

    pidf.copyTo(motor, TurretConst.SLOT_INDEX);


    // Configure Sensor Source for Pirmary PID
		motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TurretConst.LOOP_INDEX, Constants.kTimeoutMs);

    /* set deadband to super small 0.001 (0.1 %).
    The default deadband is 0.04 (4 %) */
    motor.configNeutralDeadband(0.001, Constants.kTimeoutMs);

    /**
    * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
    * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
    * sensor to have positive increment when driving Talon Forward (Green LED)
    */
    motor.setSensorPhase(false);
    motor.setInverted(false);
    /*
    * Talon FX does not need sensor phase set for its integrated sensor
    * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
    * and the user calls getSelectedSensor* to get the sensor's position/velocity.
    * 
    * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
    */
        // _talon.setSensorPhase(true);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, TurretConst.COMM_RATE, Constants.kTimeoutMs);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, TurretConst.COMM_RATE, Constants.kTimeoutMs);

    /* Set the peak and nominal outputs */
    // Max range until proven otherwise
    motor.configNominalOutputForward(0, Constants.kTimeoutMs);
    motor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    motor.configPeakOutputForward(1, Constants.kTimeoutMs);
    motor.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
		motor.configMotionCruiseVelocity(TurretConst.MAX_VEL, Constants.kTimeoutMs);
		motor.configMotionAcceleration(TurretConst.MAX_ACCEL, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		motor.setSelectedSensorPosition(0, TurretConst.LOOP_INDEX, Constants.kTimeoutMs);

    motor.selectProfileSlot(TurretConst.SLOT_INDEX, TurretConst.LOOP_INDEX);

    // NT and NTEs
    NTConfig();
  }

  @Override
  public void periodic() {
    pos = getPos();
    vel = getPos();
    NTUpdate();
  }

  /**
   * Sets the PIDF of the given mode for the given motor to the given PIDF
   * controller.
   * 
   * @param controller The values to set the PIDF to.
   */
  public void setPIDF(PIDFController controller) {
    controller.copyTo(motor, TurretConst.SLOT_INDEX);
    System.out.println("***** PIDF values of " + NTPrefix + " changed *****");
  }

  /**
   * Sets the integral gain of the PIDF loop for the given mode of the given motor
   * to the given value.
   * 
   * @param IGain The value to set the integral gain to.
   */
  public void setIGain(double IGain) {
    pidf.setIzone(IGain);
    System.out.println("***** IGain of " + NTPrefix + " set to " + IGain + "*****");
  }

  /**
   * Sets the position of the internal encoder of the given motor to the given
   * angle (in degrees).
   * 
   * @param angle The angle to set the position to.
   */
  public void setEncPos(double angle) {
    double adjustedPos = angle;
    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, 
    // divide by GEAR_RATIO because... it's the gear ratio
    motor.setSelectedSensorPosition(adjustedPos / DEGREES * COUNTS_PER_REV / GEAR_RATIO);
    System.out.println("***** Encoder position of " + NTPrefix + " set to " + angle + "*****");
  }

  /**
   * Sets the commanded value to the given value for the given motor and the given
   * control mode.
   * 
   * @param value The value to command to.
   */
  public void setDesPos(double value) {
    double adjustedVal = value;
    // divide by DEGREES to go to revolutions, multiply by COUNTS_PER_REV to go to encoder counts, 
    // divide by GEAR_RATIO because... it's the gear ratio
    motor.set(ControlMode.MotionMagic, adjustedVal * COUNTS_PER_REV / DEGREES / GEAR_RATIO);
    pos_cmd = value;
    System.out.println("***** Desired position of " + NTPrefix + " set to " + value + " deg *****");
  }

  /**
   * Gets the position of the motor.
   */
  public double getPos() {
    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees,
    // divide by GEAR_RATIO because... it's the gear ratio
    return (motor.getSelectedSensorPosition() * DEGREES / COUNTS_PER_REV * GEAR_RATIO);
  }

  /**
   * Gets the velocity of the motor.
   */
  public double getVel() {
    // divide by COUNTS_PER_REV to go to revolutions, multiply by DEGREES to go to degrees,
    // divide by GEAR_RATIO because... it's the gear ratio
    return (motor.getSelectedSensorPosition() * DEGREES / COUNTS_PER_REV / VEL_MEASUREMENT_PERIOD * GEAR_RATIO);
  }

  // NT stuff
  private NetworkTable table;
  private NetworkTableEntry nt_pos;
  private NetworkTableEntry nt_vel;
  private NetworkTableEntry nt_pos_cmd;

  void NTConfig() {
    table = NetworkTableInstance.getDefault().getTable(NT_NAME + "/" + NTPrefix);
    nt_pos = table.getEntry("/position");
    nt_vel = table.getEntry("/velocity");
    nt_pos_cmd = table.getEntry("/desiredPosition");
  }

  void NTUpdate() {
    nt_pos.setDouble(pos);
    nt_vel.setDouble(vel);
    nt_pos_cmd.setDouble(pos_cmd);
  }
}
