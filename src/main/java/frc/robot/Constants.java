// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //#region Shooting
    public static final int LED_COUNT = 38;
    public static final int LED_PORT = 2;
    public static final int SHOOTER_PORT_MAIN = 5;
    public static final int SHOOTER_PORT_SECONDARY = 6;
    public static final int TURNER_PORT = 11;
    public static final int TURNER_GYRO_PORT = 13;
    public static final int INPUT_WHEEL_PORT = 9;
    public static final int UPPER_LIMIT_SWITCH_PORT = 1;
    public static final int LOWER_LIMIT_SWITCH_PORT = 0;
    public static final double PULSE_TO_ANGLE = 360./4000;
    public static final double INPUT_WHEEL_POWER = -0.65;
    public static final double SHOOTER_PULSE_TO_METER = 0.1 * Math.PI / 2048;
    public static final double SHOOTER_KS = 0.02;
    public static final double SHOOTER_KV = 0.03;
    public static final double SHOOTER_KP = 0;//0.05 / (1 / (SHOOTER_PULSE_TO_METER * 10));
    public static final double SHOOTER_KI = 0;//SHOOTER_KP / 10;
    public static final double TURNER_DEFAULT_POWER = -0.4;
    public static final double MAX_SHOOT_ANGLE_ERROR = 1;
    public static final double MAX_SHOOT_VELOCITY_ERROR = 1.5;
    public static final double SHOOTING_DEFAULT_VELOCITY = 6;
    public static final double SHOOTING_DEFAULT_ANGLE = 0;
    public static final double SHOOTING_AUTO_VELOCITY = 6;
    public static final double SHOOTING_AUTO_ANGLE = 0;
    public static final double CAMERA_TOWER_DIFF = 1.5;
    public static final double MAX_ANGLE_ERROR_CHASSIS = 3;
    public static final double DRIVE_POWER_FACTOR = 0.75;
    public static final double CAMERA_ANGLE = 0;
    public static final double SPIN_PERCENTAGE = 0;

    public static final double SHOOTING_VELOCITIES_DIFF = -1;
    public static final double MIN_SHOOTING_DISTANCE = 1.3;
    //#endregion

    //#region Chassis
    public static final Pose2d RED_LEFT_POSE = new Pose2d(9.348, 3.519, Rotation2d.fromDegrees(-30));
    public static final Pose2d RED_RIGHT_POSE = new Pose2d(8.918, 5.125, Rotation2d.fromDegrees(60));
    public static final Pose2d BLUE_LEFT_POSE = new Pose2d(7.312, 4.694, Rotation2d.fromDegrees(150));
    public static final Pose2d BLUE_RIGHT_POSE = new Pose2d(7.742, 3.089, Rotation2d.fromDegrees(-120));
    public static final Translation2d HUB_POSITION = new Translation2d(8.33, 4.107);
    public static final double ROBOT_LENGTH = 0.79;
    public static final double DISTANCE_FRON_HUB_TO_FENDER = 0.78;

    public static final double MAX_VELOCITY = 1;//3;
    public static final double MAX_ANGULAR_VELOCITY = Math.PI;
    public static final double PULSES_PER_METER = 12.0*2048/(6.0*25.4*Math.PI/1000.0);

    public static final int LEFT_FRONT_PORT = 3; 
    public static final int LEFT_BACK_PORT = 4; 
    public static final int RIGHT_FRONT_PORT = 1; 
    public static final int RIGHT_BACK_PORT = 2; 

    public static final int GYRO_PORT = 14;

    public static final double TURN_VELOCITY = 1.5;
    public static final int ANGLE_KI = -1;
    public static final int ANGLE_KD = -1;

    public static final int STOP_ANGLE = 5;
    public static final double SCALE_VELOCITY_ON_PICKUP = 0.5;

    public static final double KS = 0.59316 / 12;
    public static final double KV = 2.6475 / 12;
    public static final double KP = 1. / 10000;
    public static final double TURN_SCALE_SAGI = 0.9;
    public static final double TURN_SCALE_GUY = 1.2;
    public static final double ZERO_TURN_SAGI = 0.7;
    public static final double LOW_TURN_SAGI = 0.20;
    public static final double KA = 0;
    public static final double TRACK_WIDTH = 0.56;
    public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

    public static final double MAX_VELOCITY_AUTO = 2;
    public static final double MAX_ACCELERATION_AUTO = 1.5;

    public static final double MOVE_POWER = 0.3;
    public static final double TURN_POWER = 0.3;

    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
    //#endregion

    //#region Climb
    //motors
    public static final int TELESCOPIC_MOTOR = 12;
    public static final int SHACKLE_OPENNER = 8;
    
    //power
    public static final double SHACKLE_OPENNING_MAX_POWER = -0.5;

    //controller
    public static final double JOYSTICK_DEADBAND = 0.1;

    //elivator
    public static final double CLIMB_PULSES_PER_METER = -1;

    //distane to move
    public static final double DISTANCE_STEP_1 = -1;
    public static final double DISTANCE_STEP_2 = -1;

    //PID for climbing
    public static final double CLIMB_KP = -1;
    public static final double CLIMB_KI = -1;
    public static final double CLIMB_KD = -1;

    //FeedForward for climbing
    public static final double CLIMB_KS = -1;
    public static final double CLIMB_KV = -1;
    public static final double CONTROLLER_DEADBAND = 0.1;
    //#endregion
    
    //#region Intake
    //Ports
    public static final int INTAKE_PORT = 7;
    public static final int ARM_PORT = 10;
    
    //Power
    public static final double PICKUP_POWER = 1;

    //Velocity
    public static final double ARM_UP_POWER = -0.8;
    public static final double ARM_DOWN_POWER = 0.6;
    //#endregion
    public static final double ANGLE_KP = 0.1;
}
