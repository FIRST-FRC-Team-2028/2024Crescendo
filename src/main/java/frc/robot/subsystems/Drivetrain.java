// Copyright (c) Triple Helix Robotics, FRC 2363. All rights reserved.

package frc.robot.subsystems;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// auto imports
//import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
//import com.pathplanner.lib.util.PIDConstants;
//import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;


/** Constructs a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  boolean iShouldStop;  // logical mechanism to stop moving when other reasons are not met
  static double kMaxSpeed = Constants.DriveConstants.kMaxTranslationalVelocity;
  static double kMaxAngularSpeed = Constants.DriveConstants.kMaxRotationalVelocity;
  private PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");
   AprilTagFieldLayout aprilTagFieldLayout;
   AprilCamera aprilSubsystem;

  private final SwerveDriveKinematics m_kinematics = DriveConstants.kDriveKinematics;


  private final SwerveModule m_frontLeft =
      new SwerveModule(
          "FL",
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftAbsoluteEncoderPort);
  private final SwerveModule m_frontRight =
      new SwerveModule(
          "FR",
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightAbsoluteEncoderPort);
  private final SwerveModule m_backLeft =
      new SwerveModule(
          "BL",
          DriveConstants.kBackLeftDriveMotorPort,
          DriveConstants.kBackLeftTurningMotorPort,
          DriveConstants.kBackLeftAbsoluteEncoderPort);
  private final SwerveModule m_backRight =
      new SwerveModule(
          "BR",
          DriveConstants.kBackRightDriveMotorPort,
          DriveConstants.kBackRightTurningMotorPort,
          DriveConstants.kBackRightAbsoluteEncoderPort);

  private SwerveModule[] modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};

  private final Pigeon2 m_gyro = new Pigeon2(0);
  
  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics1,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

/** Constructs a swerve drive style drivetrain. */
  public Drivetrain() {
    resetGyro();

    for (SwerveModule module : modules) {
      module.resetDriveEncoder();
      module.initializeAbsoluteTurningEncoder();
      module.initializeRelativeTurningEncoder();
    }
  }

  public void resetGyro() {
    m_gyro.reset();
  }
  
  private final SwerveDrivePoseEstimator m_poseEstimator =
    new SwerveDrivePoseEstimator(
        m_kinematics,
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),                                                                 
          m_backRight.getPosition()
         },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public void update() {
    m_poseEstimator.update(m_gyro.getRotation2d(),
                          new SwerveModulePosition[] {                                  
                          m_frontLeft.getPosition(),
                          m_frontRight.getPosition(),
                          m_backLeft.getPosition(),
                          m_backRight.getPosition()
    });
    if(Constants.APRIL_AVAILABLE){
      var res = camera.getLatestResult();
      if (res.hasTargets()) {
        var imageCaptureTime = res.getTimestampSeconds();
        //var camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
        //var camPose = aprilTagFieldLayout.getTagPose(4).transformBy(camToTargetTrans.inverse());
        m_poseEstimator.addVisionMeasurement(
                  aprilSubsystem.camPose().getReferencePose().toPose2d(), imageCaptureTime);
        SmartDashboard.putNumber("Robot X Pos", m_poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Robot Y Pos", m_poseEstimator.getEstimatedPosition().getY());
          }
    }
  }
  


  /** If false it wouldn't spew the drive encoder values onto smartdashboard */
  static final boolean SPEWDRIVE = false;
  @Override
  public void periodic() {
    update();
    if(SPEWDRIVE) {
    for (SwerveModule module : modules) {
      SmartDashboard.putNumber(
          module.getName() + "RTurn",
          module.getRelativeTurningPosition().getDegrees());

      SmartDashboard.putNumber(
          module.getName() + "ATurn",
          module.getAbsTurningPosition(0.0).getDegrees());

      SmartDashboard.putNumber(
          module.getName() + "RDrive", module.getRelativeDrivePosition());

      SmartDashboard.putNumber(
          module.getName() + "AMagOff",
          module.getAbsTurningEncoderOffset().getDegrees());
        
    }
  }

    SmartDashboard.putNumber("Heading", m_gyro.getRotation2d().getDegrees());
  }



  /**
   * @param chassisSpeeds x, y, and theta speeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.kPeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    setModuleStates(swerveModuleStates);
    /*m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);*/
  }

  // TODO: see also this.update() method; do we need both?
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  /** Reconfigures all swerve module steering angles using external alignment device */
  /*  public void zeroAbsTurningEncoderOffsets() {
    for (SwerveModule module : modules) {
      module.zeroAbsTurningEncoderOffset();
    }
  } */



  /**
   * @return The heading of the robot
   */
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

    /** return a Rotation2d representing the heading of the robot
     * described in radians clockwise from forward
     */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading().getDegrees());
  }

    /**
     * @return rotate 90 degrees
     */
  public Rotation2d desiredRotation() {
    return Rotation2d.fromDegrees(270);
  }

  public void BreakMode() {
    m_frontLeft.BreakMode();
    m_frontRight.BreakMode();
    m_backLeft.BreakMode();
    m_backRight.BreakMode();
  }

  public void CoastMode() {
    m_frontLeft.CoastMode();
    m_frontRight.CoastMode();
    m_backLeft.CoastMode();
    m_backRight.CoastMode();
  }

  public SwerveModuleState[] chassis2ModuleStates(ChassisSpeeds speeds){
    return DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
  }
  public Pose2d getPose() {
    //updateOdometry();
    System.out.println("robot Pose = "+m_odometry.getPoseMeters().toString());
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    resetGyro();
    m_odometry.resetPosition(getHeading(), getSwerveModulePositions(), pose);
  }
  
  public SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] bill = {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
    return bill;
  }

  public SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    for (int i = 0; i <= 3; i++) {
      states[i++] = modules[i].getState();
    }

    return states;
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState());
  }

  public void stopModules() {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_backLeft.setDesiredState(desiredStates[2]);
    m_backRight.setDesiredState(desiredStates[3]);
  }

    /**Drives in robot or field coordinate system.
     * @xhowfast and 
     * @yhowfast are the components of the vector
     * in meters/sec,
     * @turnSpeed is the rotation rate in radians/sec counterclockwise
     */
    public void driveit(double xhowfast, double yhowfast, double turnSpeed, boolean fieldoriented) {
      // Remember that xspeed is backwards on robot
      ChassisSpeeds chassisSpeeds;
      if (fieldoriented){
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          -xhowfast, yhowfast, turnSpeed, getRotation2d());
      } 
      else{
          chassisSpeeds = new ChassisSpeeds(-xhowfast, yhowfast, turnSpeed);
      }

      drive(chassisSpeeds);
      
      /*SwerveModuleState[] moduleStates = chassis2ModuleStates(chassisSpeeds);
      setModuleStates(moduleStates);*/
  }

  /** test for premature stop criterion */
  public boolean shouldistop() {
    return iShouldStop;
  }

  /** set premature stop criterion to false
   *  ie, go as directed
   */
  public void makemefalse() {
    iShouldStop = false;
  } 

  /** set premature stop criterion to true
   *  ie, stop right now
   */
  public void makeMeStop() {
    iShouldStop = true;
  }

  /* public void configurePathPlanner() {
    AutoBuilder.configureHolonomic(
        () -> this.getPose(), // Robot pose supplier
        this::resetOdometry,
        this::getCurrentRobotChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds

        // update these to fit our robot
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(
                Constants.ModuleConstants.kDriveP,
                Constants.ModuleConstants.kDriveI,
                Constants.ModuleConstants.kDriveD), // Translation PID constants
            new PIDConstants(
                Constants.ModuleConstants.kTurningP,
                Constants.ModuleConstants.kDriveI,
                Constants.ModuleConstants.kDriveD), // Rotation PID constants
            Constants.DriveConstants.kMaxTranslationalVelocity, // Max module speed, in m/s
            Constants.DriveConstants
                .kRadius, // Drive base radius in meters. Distance from robot center to furthest
            // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
  } */

  /*SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
    );
  public String getName() {
    return "MrGumbert";
  }
  public void voltageDrive(Measure<Voltage> sysSpeed) {
    m_frontLeft.voltageDrive(sysSpeed.magnitude());
    m_frontRight.voltageDrive(sysSpeed.magnitude());
    m_backLeft.voltageDrive(sysSpeed.magnitude());
    m_backRight.voltageDrive(sysSpeed.magnitude());
  }
  public double[] logMotors(SysIdRoutineLog kidGumbert) {
    double[] MrsGumbert =
    {m_frontLeft.getRelativeDrivePosition(), m_frontLeft.getVelocity(), m_frontLeft.getVoltage()};
    return MrsGumbert;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }*/

}
