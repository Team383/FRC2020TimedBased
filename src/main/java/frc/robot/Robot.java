/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

  // private final ColorSensorV3 m_colorSensor = new
  // ColorSensorV3(Constants.i2cPort);

  // private final ColorMatch m_colorMatcher = new ColorMatch();

  // private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  // private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  // private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  // private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524,
  // 0.113);

  // ------------ Drive train ----------------
  private WPI_TalonSRX LL = new WPI_TalonSRX(Constants.DriveTrain.kLL);
  private WPI_VictorSPX LR = new WPI_VictorSPX(Constants.DriveTrain.kLR);
  private WPI_TalonSRX FR = new WPI_TalonSRX(Constants.DriveTrain.kFL);
  private WPI_VictorSPX FL = new WPI_VictorSPX(Constants.DriveTrain.kFR);

  private DifferentialDrive driveTrain = new DifferentialDrive(LL, LR);

  // ---------- Controllers---------------
  private Joystick stickMain = new Joystick(Constants.Controller.kStickMainPort);
  private Joystick stickRot = new Joystick(Constants.Controller.kStickRotPort);
  Joystick xbox = new Joystick(Constants.Controller.kXboxPort);
  // private XboxController xbox = new
  // XboxController(Constants.Controller.kXboxPort);

  // --------------Spinner-------------
  private WPI_TalonSRX spinner = new WPI_TalonSRX(Constants.Spinner.kSpinner);

  // // -------------- Intake --------------------
  private WPI_VictorSPX intake = new WPI_VictorSPX(Constants.Intake.kIntake);

  //------------------ Transporter ------------------

  private WPI_VictorSPX leadTransporter = new WPI_VictorSPX(Constants.Intake.kLT);
  private WPI_VictorSPX followTransporter = new WPI_VictorSPX(Constants.Intake.kFT);

  // // -------------- Turret ----------------
  private WPI_TalonSRX leadShooter = new WPI_TalonSRX(Constants.Turret.kLS);
  private WPI_VictorSPX followShooter = new WPI_VictorSPX(Constants.Turret.kFS);

  private WPI_TalonSRX yawShooter = new WPI_TalonSRX(Constants.Turret.kYS);
  private WPI_TalonSRX pitchShooter = new WPI_TalonSRX(Constants.Turret.kPS);

  // DigitalInput pitchRight = new DigitalInput(Constants.Turret.kPitchRightLM);
  // DigitalInput pitchLeft = new DigitalInput(Constants.Turret.kPitchLeftLM);

  // DigitalInput yawRight = new DigitalInput(Constants.Turret.kYawRightLM);
  // DigitalInput yawLeft = new DigitalInput(Constants.Turret.kYawLeftLM);

  // -------------------Elevator------------------
  private WPI_TalonSRX elevatorClimber = new WPI_TalonSRX(Constants.Elevator.kEC);

  private WPI_TalonSRX elevatorHook = new WPI_TalonSRX(Constants.Elevator.kEH);

  // DigitalInput elevatorHigh = new
  // DigitalInput(Constants.Elevator.kElevatorHighLS);
  // DigitalInput elevatorLow = new
  // DigitalInput(Constants.Elevator.kElevatorLowLS);

  // ------------------- Pneumatic subsystem -----------------
  // Compressor compressor = new Compressor();
  // DoubleSolenoid grabberSolenoid = new DoubleSolenoid(2,3);
  // DoubleSolenoid spinnerSolenoid = new DoubleSolenoid(0,1);

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
    // RL.follow(FL);
    // RR.follow(FR);

    // ----- turret
    // slave.follow(master);

    // --------------- Elevator --------------
    // elevator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    // spinner.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

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

    // m_colorMatcher.addColorMatch(kBlueTarget);
    // m_colorMatcher.addColorMatch(kGreenTarget);
    // m_colorMatcher.addColorMatch(kRedTarget);
    // m_colorMatcher.addColorMatch(kYellowTarget);

    // compressor.start();

    configureMotorControllers();

    // spinner.setSelectedSensorPosition(0,0,10);
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
    // Color detectedColor = m_colorSensor.getColor();

    // /**
    // * Run the color match algorithm on our detected color
    // */

    // ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    // gameData = DriverStation.getInstance().getGameSpecificMessage();

    // if (gameData.equals("B")) {
    // gameDataNum = Cores.B.ordinal();
    // } else if (gameData.equals("G")) {
    // gameDataNum = Cores.G.ordinal();
    // } else if (gameData.equals("R")) {
    // gameDataNum = Cores.R.ordinal();
    // } else if (gameData.equals("Y")) {
    // gameDataNum = Cores.Y.ordinal();
    // }

    // if (match.color == kBlueTarget) {
    // colorString = "Blue";
    // colorLetras = "B";
    // colorNum = Cores.B.ordinal();
    // } else if (match.color == kRedTarget) {
    // colorString = "Red";
    // colorLetras = "R";
    // colorNum = Cores.R.ordinal();

    // } else if (match.color == kGreenTarget) {
    // colorString = "Green";
    // colorLetras = "G";
    // colorNum = Cores.G.ordinal();

    // } else if (match.color == kYellowTarget) {
    // colorString = "Yellow";
    // colorLetras = "Y";
    // colorNum = Cores.Y.ordinal();

    // } else {
    // colorString = "Unknown";
    // colorLetras = "Unknown";
    // }

    // /**
    // * Open Smart Dashboard or Shuffleboard to see the color detected by the
    // sensor.
    // */
    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("Confidence", match.confidence);
    // SmartDashboard.putString("Detected Color", colorString);
    // SmartDashboard.putNumber("Elevator sensor position",
    // elevator.getSelectedSensorPosition());
    // SmartDashboard.putNumber("spinner sensor position",
    // spinner.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Voltas rodinha",
    // (double)(spinner.getSelectedSensorPosition())/(Constants.Spinner.kSpinnerTickstoRotations));

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
    // driveTrain.arcadeDrive(stickMain.getY(), stickRot.getX());

    // if(xbox.getXButtonPressed()){
    // grabberSolenoid.set(Value.kForward);
    // } else if(xbox.getYButtonPressed()) {
    // grabberSolenoid.set(Value.kReverse);
    // } else{
    // grabberSolenoid.set(Value.kOff);
    // }

    // liga todos motores do sistema de pegar bolas
    if (xbox.getRawButton(1)) { // A
      intake.set(1.0);
      spinner.set(1.0);
    } else {
      intake.set(.0);
      spinner.set(.0);
    }

    // intake
    if (xbox.getRawButton(2)) { // B
      intake.set(1.0);
    } else {
      intake.set(.0);
    }

    // spinner
    if (xbox.getRawButton(3)) { // B
      spinner.set(1.0);
    } else {
      spinner.set(.0);
    }

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
