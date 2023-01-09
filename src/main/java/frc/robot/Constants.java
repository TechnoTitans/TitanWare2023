// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
@SuppressWarnings("unused")
public interface Constants {
    double kS = 0.71432; //volts
    double kV = 1.6858; //volt-seconds/meter
    double kA = 0.52711; // volt-seconds squared/meter
    double MAX_VOLTAGE = 0; // volts
    double MAX_VELOCITY = 17 * 12 * 2.54 / 100; //m/s
    double MAX_ACCELERATION = MAX_VELOCITY; //m/s^2
    double RAMSETE_B = 2;
    double RAMSETE_ZETA = .7;
    double DRIVE_WIDTH = 1.1; //meters horizontal distance between the wheels??
    //    double kP_DRIVE_VELOCITY = 1.25; //0.482
    double kP_DRIVE_VELOCITY = 0.0017163;
    double GEARING = 7.39;
    double WHEEL_DIAMETER = 6;
    double ENCODER_EDGES_PER_REV = 2048;

    //    double kP_DRIVE_VELOCITY = 0;
    DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(DRIVE_WIDTH); //meters
}
