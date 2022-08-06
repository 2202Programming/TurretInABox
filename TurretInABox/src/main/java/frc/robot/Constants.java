// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    //Normal CAN bus
    public static class CAN {
        public static final int PDP = 1;
    
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
        public static String NT_NAME = "/turret";
        
        //[p1][p2][final gear] motor turns to turret turn
        public static final double AZIMUTH_GEAR_RATIO = 9.0*5.0*14.0; 
        public static final double ELEVATION_GEAR_RATIO = 9999.99999;

    }

    public static class NetworkTables {
        
    }
}
