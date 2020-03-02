/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

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

    public final static I2C.Port i2cPort = I2C.Port.kOnboard;

    public class DriveTrain {
        public final static int kFL = 1;
        public final static int kFR = 1;
        public final static int kRL = 1;
        public final static int kRR = 1;
    }

    public class Controller {
        public final static int kStickMainPort = 0;
        public final static int kStickRotPort = 1;
        public final static int kXboxPort = 2;
    }

    public class Turret {
        public final static int kMaster = 0;
        public final static int kSlave = 0;

        public final static int kYaw = 0;
        public final static int kPitch = 0;

        public final static int kTurretRight = 0;
        public final static int kTurretLeft = 0;

        public final static int kFeeder = 0;

    }

    public class Elevator {
        public final static int kElevator = 0;
        public final static int kElevatorHighLS = 0;
        public final static int kElevatorLowLS = 0;
    }

    public class Intake {
        public final static int kIntake = 0;
    }

    public class Spinner {
        public final static int kSpinner = 0;
    }
}
