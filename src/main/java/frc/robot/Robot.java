/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public enum Cores {
    /**
     * Acessar o valor de cada elemento com o metodo ordinal() Ex.: int blue =
     * Cores.B.ordinal(); blue vai ter valor 0, pois B está na posicao 0.
     */
    B, G, R, Y;
  }

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(Constants.i2cPort);

  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  // ------------ Drive train ----------------
  private WPI_TalonSRX FL = new WPI_TalonSRX(Constants.DriveTrain.kFL);
  private WPI_VictorSPX FR = new WPI_VictorSPX(Constants.DriveTrain.kFR);
  private WPI_TalonSRX RL = new WPI_TalonSRX(Constants.DriveTrain.kRL);
  private WPI_VictorSPX RR = new WPI_VictorSPX(Constants.DriveTrain.kRR);

  private DifferentialDrive driveTrain = new DifferentialDrive(FL, FR);

  // ---------- Controllers---------------
  private Joystick stickMain = new Joystick(Constants.Controller.kStickMainPort);
  private Joystick stickRot = new Joystick(Constants.Controller.kStickRotPort);
  private XboxController xbox = new XboxController(Constants.Controller.kXboxPort);

  // --------------Spinner-------------
  private WPI_TalonSRX spinner = new WPI_TalonSRX(Constants.Spinner.kSpinner);

  // -------------- Intake --------------------
  private WPI_TalonSRX intake = new WPI_TalonSRX(Constants.Intake.kIntake);

  // -------------- Turret ----------------
  private WPI_TalonSRX master = new WPI_TalonSRX(Constants.Turret.kMaster);
  private WPI_VictorSPX slave = new WPI_VictorSPX(Constants.Turret.kSlave);

  private WPI_TalonSRX yaw = new WPI_TalonSRX(Constants.Turret.kYaw);
  private WPI_TalonSRX pitch = new WPI_TalonSRX(Constants.Turret.kPitch);

  DigitalInput turretRight = new DigitalInput(Constants.Turret.kTurretRight);
  DigitalInput turretLeft = new DigitalInput(Constants.Turret.kTurretLeft);

  // -------------------Elevator------------------
  private WPI_TalonSRX elevator = new WPI_TalonSRX(Constants.Elevator.kElevator);
  DigitalInput elevatorHigh = new DigitalInput(Constants.Elevator.kElevatorHighLS);
  DigitalInput elevatorLow = new DigitalInput(Constants.Elevator.kElevatorLowLS);

  // ---------- Util -------------------
  Timer timer = new Timer();
  String gameData;
  String colorString;
  String colorLetras;
  int colorNum;
  int gameDataNum;
  boolean start;

  public void configureMotorControllers() {
    // --- drive train
    RL.follow(FL);
    RR.follow(FR);

    // ----- turret
    slave.follow(master);
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    configureMotorControllers();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    gameData = DriverStation.getInstance().getGameSpecificMessage();

    if (gameData.equals("B")) {
      gameDataNum = Cores.B.ordinal();
    } else if (gameData.equals("G")) {
      gameDataNum = Cores.G.ordinal();
    } else if (gameData.equals("R")) {
      gameDataNum = Cores.R.ordinal();
    } else if (gameData.equals("Y")) {
      gameDataNum = Cores.Y.ordinal();
    }

    if (match.color == kBlueTarget) {
      colorString = "Blue";
      colorLetras = "B";
      colorNum = Cores.B.ordinal();
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      colorLetras = "R";
      colorNum = Cores.R.ordinal();

    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      colorLetras = "G";
      colorNum = Cores.G.ordinal();

    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      colorLetras = "Y";
      colorNum = Cores.Y.ordinal();

    } else {
      colorString = "Unknown";
      colorLetras = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
    case kCustomAuto:
      // Put custom auto code here
      break;
    case kDefaultAuto:
    default:
      // Put default auto code here
      break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // --------- manual drive ----------------
    driveTrain.arcadeDrive(stickMain.getY(), stickRot.getX());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
