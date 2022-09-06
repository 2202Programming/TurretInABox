// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.turret.FlyWheel.FlyWheelConfig;
import frc.robot.subsystems.turret.Shooter_Subsystem.ShooterSettings;
import frc.robot.util.PIDFController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double DT = 0.02;  // 50Hz 20ms frametime

    public static final int kTimeoutMs = 30; // for sake of testing

    //Normal CAN bus
    public static class CAN {
        public static final int PDP = 1;
    
        // Shooter CAN devices
        public static final int SHOOTER_UPPER_TALON = 10;
        public static final int SHOOTER_LOWER_TALON = 11;
    }
    // CANivour bus
    public static class CAN2 {
        public static final String BUSNAME = "bob";
        public static final int AZIMUTH = 1;
        public static final int ELEVATION = 2;
        public static final int SHOOTER = 3;
        
    }

    public static class TurretConst { 
        public static final int BUSTIMEOUT = 20;  // [mS] for init config calls
        public static final int COMM_RATE = 10; // [mS] Nyquist rate -- cut periodic by half

        public static String NT_NAME = "/turret";
        
        //[p1][p2][final gear] motor turns to turret turn
        public static final double AZIMUTH_GEAR_RATIO = 9.0*5.0*14.0; 
        public static final double ELEVATION_GEAR_RATIO = 9999.99999;

        // tolerances
        public static final double TOLERANCE_AZ = 0.0; // in deg
        public static final double TOLERANCE_ELE = 0.0; // in deg

        public static final double MAX_VEL = 15000; // counts / 100ms
        public static final double MAX_ACCEL = 6000; // counts / 100ms / sec

        public static final int SLOT_INDEX = 0;
        public static final int LOOP_INDEX = 0;
    }

    public static class NetworkTables {
        
    }

    public static final class Shooter {
        public static final double DefaultRPMTolerance = .05; // percent of RPM
        public static final ShooterSettings DefaultSettings = new ShooterSettings(20.0, -20.0); // ft/s, rot/s
    
        // Power Cell info
        // public static final double PowerCellMass = 3.0 / 16.0; // lbs
        public static final double PCNominalRadius = 10 / 2.0 / 12.0; // feet - power cell
        public static final double PCEffectiveRadius = 8 / 2.0 / 12.0; // feet - compressed radius
    
        public static final double shortVelocity = 40;
        public static final double shortMediumVelocity = 44;
        public static final double mediumVelocity = 50;
        public static final double longVelocity = 60;
        public static final double autoVelocity = 46;
    
        // limelight distance constants
        // how many degrees back is your limelight rotated from perfectly vertical?
        public static final double LL_MOUNT_ANGLE_DEG = 28.0;
        // distance from the center of the Limelight lens to the floor
        public static final double LL_LENS_HEIGHT_INCHES = 27.0;
        // distance from the target to the floor
        public static final double GOAL_HEIGHT_TO_FLOOR_INCHES = 104.0;
        // adjustment from edge to centre of target
        public static final double EDGE_TO_CENTER_INCHES = 24.0;
        // adjustment factor
        public static final double METERS_TO_INCHES = 39.37;
    
        public static final double limelight_default_p = 7; // was 4 // [deg/s / deg-err]
        public static final double limelight_default_i = 0.1;
        public static final double limelight_default_d = 0.1;
    
        // constraints
        public static final double kMaxFPS = 80; // max FPS
        public static final double maxLongRage = 8; // maximum range in long distance shooting mode
        public static final double minLongRange = 1.8; // minimum range in long distance shooting mode
        public static final double maxShortRange = 2; // maximum range in short distance shooting mode
        public static final double degPerPixel = 59.6 / 320; // limelight conversion
        public static final double angleErrorTolerance = 2.0; // [deg] allowed angle error to shoot in guided shooting modes
        public static final double angleVelErrorTolerance = 1.0; // [deg/s] allowed angle error to shoot in guided shooting
                                                                 // modes
    
        // !!
        // !!
        // stuff to touchy touchy
        // !!
        // !!
    
        // for velocity calculations
        // cut over distance between two distance/speed linear relationships
        public static final double FARDISTANCE = 20.0;
    
        // close slope/intercept. Slope will change multiplier between distance and RPM.
        // Intercept will add RPMs to all distances equally.
        public static final double SLOPE = 4.872;
        public static final double INTERCEPT = 26.8 * 1.0; // 10% chance 10k lakes practice adjustment shooting short
    
        // change slope multiplier to increase FPS at far distances.
        public static final double FARSLOPE = SLOPE * 1.35 / 1.35; // division 7-22-2022
        public static final double FARINTERCEPT = (FARDISTANCE - 10) * SLOPE + INTERCEPT; // -10 7-22-2022
    
        // !!
        // !!
        // end stuff to touchy touch
        // !!
        // !!
    
        // Flywheel info
        // Flywheel maxOpenLoopRPM and gear ratio are used to calculate kFF in shooter
        public static FlyWheelConfig upperFWConfig = new FlyWheelConfig();
        static {
          upperFWConfig.maxOpenLoopRPM = 4000; // estimated from 2000 RPM test
          upperFWConfig.gearRatio = 1.0; // upper encoder:fw is 1:1
          upperFWConfig.sensorPhase = true;
          upperFWConfig.inverted = false;
          upperFWConfig.flywheelRadius = 2 / 12.0; // feet
          upperFWConfig.pid = new PIDFController(0.12, 0.0, 4.0, 0.034); // kP kI kD kFF
          upperFWConfig.pid.setIzone(1800);
        }
    
        public static FlyWheelConfig lowerFWConfig = new FlyWheelConfig();
        static {
          lowerFWConfig.maxOpenLoopRPM = 4000;
          lowerFWConfig.gearRatio = 1.0; // lower encoder:fw is 1:1
          lowerFWConfig.sensorPhase = false;
          lowerFWConfig.inverted = false;
          lowerFWConfig.flywheelRadius = 2 / 12.0; // feet
          lowerFWConfig.pid = new PIDFController(0.12, 0.0, 4.0, 0.034); // kP kI kD kFF
          lowerFWConfig.pid.setIzone(1800);
        }
    
      }
}
