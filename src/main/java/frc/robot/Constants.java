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
        // Motor controllers
        // LR - SRX - Lead Right 
        // LL - SRX - Lead Left 
        // FR - SPX - Follow Right 
        // FL - SPX - Follow Left 
        public final static int kLR = 10;
        public final static int kLL = 11;
        public final static int kFL = 12;
        public final static int kFR = 13;
    }

    public class Controller {
        //USB ports
        public final static int kStickMainPort = 0;
        public final static int kStickRotPort = 1;
        public final static int kXboxPort = 2;
    }

    public class Turret {
        //--------------- Motor controllers --------------------
       
        // LS - SRX - Lead Shooter (Power Cells)
        // FS - SPX - Follow Shooter (Power Cells)
        public final static int kLS = 20;
        public final static int kFS = 21;

        // YS - SRX - Yaw Shooter (Power Cells)
        // PS - SRX - pitch Shooter (Power Cells)
        public final static int kYS = 22;
        public final static int kPS = 23;

        // FE - SPX - Feeder (Power Cells)
        public final static int kFeeder = 24;

        //------------------limit switch ------------------------
		public final static int kPitchLeftLM = 0;
        public final static int kPitchRightLM = 0;;
        
        public final static int kYawRightLM = 0;
        public final static int kYawLeftLM = 0;

    }
    public class Intake {
        // IN - SPX - Intake (Power Cells)
        public final static int kIntake = 25;

        //--------------------Transporter--------------
        // LT - SPX - Lead Transporter (Power Cells)
        public final static int kLT = 26;
        // FT - SPX - Follow Transporter (Power Cells)
        public final static int kFT = 27;

        
    }

    public class Elevator {
        //----------------- Motor Controller
        // EC - SRX - Elevator Climb (Climber)
        public final static int kEC = 31;

        // EH - SRX - Elevator Hook (Climber)
        public final static int kEH = 30;

        //----------------- Limit switch
        public final static int kElevatorHighLS = 0;
        public final static int kElevatorLowLS = 0;
    }

  

    public class Spinner {
        // SP - SRX - Spinner (Control Panel)
        public final static int kSpinner = 40;
        public final static double kSpinnerTickstoRotations = 1024;
    }

}
