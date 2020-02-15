/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //region I2C
    public static final int FRONT_LEFT_LIDAR = 21;
    public static final int FRONT_RIGHT_LIDAR = 22;
    //endregion

    //region Timing
    public static final double LIDAR_SAMPLE_TIME = 100; //in ms
    public static final double COLOR_SAMPLE_TIME = 100; //in ms
    public static final int LOG_REFRESH_RATE = 100; //in ms
    //endregion

    //region PWM
    public static final int E_ROT_SPARKMAX_PWM = 1; //Extension Arm Rotation Motor
    //endregion

    //region CAN
    public static final int E_SPARKMAX_CANID = 14; //Extension Motor
    public static final int WN_SPARKMAX_CANID = 16; //Winch Motor
    public static final int FLOOR_SENSOR = 17; //Time of Flight sensor, measures distance from floor
    //endregion
}
