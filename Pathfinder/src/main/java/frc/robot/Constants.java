// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final double k1 = 10;
  public static final double k2 = 1;
  public static final double k3 = 1;
  private static final LinearVelocity MAX_VELOCITY_MPS = MetersPerSecond.of(4.5);
  private static final LinearAcceleration MAX_ACCELERATION_MPS_SQUARED = MetersPerSecondPerSecond.of(3.0);
  private static final AngularVelocity MAX_ANGULAR_VELOCITY_RAD_PER_SEC = RadiansPerSecond.of(Math.PI*2);
  private static final AngularAcceleration MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED = RadiansPerSecondPerSecond.of(4*Math.PI);
  public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
      MAX_VELOCITY_MPS,
      MAX_ACCELERATION_MPS_SQUARED,
      MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
      MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQUARED);
public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
    Mass.ofBaseUnits(54, Kilograms),        // robot mass (54kg is typical)
    MomentOfInertia.ofBaseUnits(6, KilogramSquareMeters), // MOI (5-8 typical)
new ModuleConfig(
        0.0508,  // wheel radius meters
        4.5,     // max drive velocity m/s
        1.0,     // wheel COF
        DCMotor.getKrakenX60(1),
        40.0,    // current limit amps
        1        // motors per module
    ),
    Meters.of(0.55)
);
  public static double time = 0.0;
public static Pose2d robotPose = new Pose2d();
}
