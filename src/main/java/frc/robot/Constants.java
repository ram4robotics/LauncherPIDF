/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class LauncherConstants {
        public static final int kLauncherMotorLeft_id = 31;
        public static final int kLauncherMotorRight_id = 39;
        public static final double kClosedLoopRampRate = 1.0;  // 1 second from 0 to full throttle
        public static final IdleMode kIdleMode = IdleMode.kCoast;
        public static final int kSparkMaxBuiltinCPR = 42;
        public static final int kNeoEncoderPulsesPerRev = kSparkMaxBuiltinCPR * 4;
        // public static final double kLauncherPower = 0.8;

        // Values from FRC Characterization Tool (Spark Max controller; units: Rotations per second)
        public static final double kS = 0.0609;
        public static final double kV = 0.126;
        public static final double kA = 0.0177;
        public static final double char_kP_RPS = 0.14; // kP in units of Revolutions per second; output is voltage
        public static final double char_kD = 0.0;
        public static final double kMaxVoltage = 12.0;
        public static final double kP_RPM = char_kP_RPS / (kMaxVoltage * 60);

        // PID coefficients, tryied to manually tune them without characterization tool
        // public static final double  kP = 6e-5; 
        public static final double  kI = 0;
        public static final double  kD = 0; 
        public static final double  kIz = 0; 
        public static final double  kFF = 0.00015; // zero-zero-zero-one-five
        public static final double  kMaxOutput = 1; 
        public static final double  kMinOutput = -1;
        public static final double  kMaxRPM = 5700;
    }
}
