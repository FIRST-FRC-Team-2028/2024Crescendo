package frc.robot;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;

/* Table of Contents:
 *    Availablility of Subsystems
 *    ModuleConstants
 *    DriveConstants
 *    ArmConstants
 *    HandlerConstants
 *    ColorConstants
 *    CANIDs
 *    AutoConstants
 *    OIConstants
 *    CamConstant
 *    RobotConstants
 *    FieldConstants
 */

public final class Constants {
  public static final boolean PHOTONVISION_AVAILABLE = true;
  public static final boolean DRIVE_AVAILABLE = true;
  public static final boolean ARM_AVAILABLE = false;
  public static final boolean INTAKE_AVAILABLE = false;
  public static final boolean CLIMB_AVAILABLE = false;
  public static final boolean APRIL_AVAILABLE = false;


  public static final class ModuleConstants {
      public static final int kDriveMotorCurrentLimit = 80;
      public static final int kTurningMotorCurrentLimit = 80;
      public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
      public static final double kDriveMotorGearRatio = 1 / 6.75;
      public static final double kTurningMotorGearRatio = 1 / 12.8;
      public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
     // Theoraticlly maybe; in practice from measurements
      //public static final double kDriveEncoderRot2Meter = Units.inchesToMeters(1)/41.2;
      public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
      public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 42;
      public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 42;
      public static final double kPTurning_Comp = 0.5;
      public static final double kDriveP = 0.1; // 2023 Competition Robot
    public static final double kDriveI = 0.0; // 2023 Competition Robot
    public static final double kDriveD = 0.0; // 2023 Competition Robot
    public static final double kDriveFF = 0.255; // 2023 Competition Robot

    public static final double kPTurning = 0.5;
    public static final double kTurningP = 0.75; // 2023 Competition Robot
    public static final double kTurningI = 0.0; // 2023 Competition Robot
    public static final double kTurningD = 0.0; // 2023 Competition Robot

    public static final double kTurnGearRatio = 12.8; // 2023 Competion Robot
    public static final double kTurnPositionConversionFactor = 1.0 / kTurnGearRatio;
  
      // By default, the drive encoder in position mode measures rotations at the drive motor
    // Convert to meters at the wheel
    public static final double kDriveGearRatio = 6.75; // 2023 Competion Robot
    public static final double kDrivePositionConversionFactor =
        (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

    // By default, the drive encoder in velocity mode measures RPM at the drive motor
    // Convert to meters per second at the wheel
    public static final double kDriveVelocityConversionFactor =
        kDrivePositionConversionFactor / 60.0;
  }

    public static final class DriveConstants {
        //Defines the conventional order of the modules when in arrays
        public static final int Front_Left = 0;
        public static final int Front_Right = 1;
        public static final int Back_Left = 2;
        public static final int Back_Right = 3;

        // Distance between right and left wheels
        // Distance between front and back wheels
        public static final double kTrackWidth = Units.inchesToMeters(16.5);
        //
        public static final double kWheelBase = Units.inchesToMeters(22.5);
        // 
        public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final SwerveDriveKinematics kDriveKinematics1 = new SwerveDriveKinematics(
//                new Translation2d( kWheelBase / 2, -kTrackWidth / 2),
  //              new Translation2d( kWheelBase / 2,  kTrackWidth / 2),
    //            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
      //          new Translation2d(-kWheelBase / 2,  kTrackWidth / 2));

                new Translation2d( kWheelBase / 2,  kTrackWidth / 2), // front left
                new Translation2d( kWheelBase / 2, -kTrackWidth / 2), // front right
                new Translation2d(-kWheelBase / 2,  kTrackWidth / 2), // back left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // back right



                  

//    COMP BOT  FRONT                PRAC BOT    FRONT
//     +----------------------+        +----------------------+
//     | D11 S21      D12 S22 |        | D15 S25      D16 S26 |
//     | E31          E32     |        | E35          E36     |
//     |                      |        |                      |
//     |                      |        |                      |
//     |                      |        |                      |
//     |                      |        |                      |
//     | D13 S23      D14 S24 |        | D17 S27      D18 S28 |
//     | E33          E34     |        | E37          E38     |
//     +----------------------+        +----------------------+
//     
        //Front Left
        public static final int kFrontLeftDriveMotorPort        = 10;    // Module 1
        public static final int kFrontLeftTurningMotorPort      = 11;
        public static final int kFrontLeftAbsoluteEncoderPort   = 12;
        
        //Front Right
        public static final int kFrontRightDriveMotorPort       = 20;    // Module 2
        public static final int kFrontRightTurningMotorPort     = 21;
        public static final int kFrontRightAbsoluteEncoderPort  = 22;

        //Back Right
        public static final int kBackRightDriveMotorPort        = 30;    // Module 3
        public static final int kBackRightTurningMotorPort      = 31;
        public static final int kBackRightAbsoluteEncoderPort   = 32;

        //Back Left
        public static final int kBackLeftDriveMotorPort         = 40;    // Module 4
        public static final int kBackLeftTurningMotorPort       = 41;
        public static final int kBackLeftAbsoluteEncoderPort    = 42;

        //Encoder Inversions
        public static final boolean kFrontLeftTurningEncoderReversed  = true;
        public static final boolean kBackLeftTurningEncoderReversed   = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed  = true;

        public static final boolean kFrontLeftDriveEncoderReversed  = false;
        public static final boolean kBackLeftDriveEncoderReversed   = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed  = false;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed  = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed   = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed  = true;
        
        // Speed Limits
        public static final double kPhysicalMaxSpeedMetersPerSecond = 1.5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond /1.5;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                                           kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.8;
        public static final double kptwist = .5;

        public static final double kFineControlSpeed = .5;
  
    //    CAN_Number = new int[10];
    //    CAN_Name   = new string[30];

     //   CAN_Number[1] = 11;
     //   CAN_Name[1]   = "Front Left Drive Motor";
     //   CAN_Number[2] = 21;
     //   CAN_Name[2]   = "Front Left Turn Motor";
     //   CAN_Number[3] = 31;
     //   CAN_Name[3]   = "Front Left Absolute Encoder";

      //  CAN_Number[4] = 12;
      //  CAN_Name[4]   = "Front Right Drive Motor";
      //  CAN_Number[5] = 22;
      //  CAN_Name[5]   = "Front Right Turn Motor";
     //   CAN_Number[6] = 32;
      //  CAN_Name[6]   = "Front Right Absolute Encoder";

     //   CAN_Number[7] = 13;
     //   CAN_Name[7]   = "Back Right Drive Motor";
     //   CAN_Number[8] = 23;
     //   CAN_Name[8]   = "Back Right Turn Motor";
     //   CAN_Number[9] = 33;
     //   CAN_Name[9]   = "Back Right Absolute Encoder";

      //  CAN_Number[10] = 14;
     //   CAN_Name[10]   = "Back Left Drive Motor";
     //   CAN_Number[11] = 24;
     //   CAN_Name[11]   = "Back Left Turn Motor";
     //   CAN_Number[12] = 34;
    //    CAN_Name[12]   = "Back Left Absolute Encoder";

      //  CAN_Number[13] = 99;
      public static final String kAbsEncoderMagnetOffsetKey = "kAbsEncoderMagnetOffsetKey";
      public static final double kDefaultAbsEncoderOffset = 0.0;

    // Units are meters per second
    public static final double kMaxTranslationalVelocity = 4.0; // 2023 Competion Robot // max 4.5

    // Units are radians per second
    public static final double kMaxRotationalVelocity = 5.0; // 2023 Competion Robot // max 5.0


    }

    public static final class ArmConstants {

      public static final double kRetractPosition = 0;
      
      //Elbow PID
        public static final double kElbowP = 0;
        public static final double kElbowI = 0;
        public static final double kElbowD = 0;
      //Wrist PID
        public static final double kWristP = 0;
        public static final double kWristI = 0;
        public static final double kWristD = 0;
      //Elbow Positions
        public static final double kElbowGround = 0;
        public static final double kElbowSource = 0;
        public static final double kElbowSpeaker = 0;
        public static final double kElbowAmp = 0;
      //Wrist Positions
        public static final double kWristSource = 0;
        public static final double kWristGround = 0;
        public static final double kWristSpeaker = 0;
        public static final double kWristAmp = 0;

      //Tolerances
      public static final double elbowTolerance = 0;
      public static final double wristTolerance = 0;
      //Soft limits
        public static final float kElbowForwardLimit = 8;
       public static final float kElbowReverseLimit = -70;
        public static final float kWristForwardLimit = 0;
        public static final float kWristReverseLimit = 0;
      //Encoders
        public static final int ABSENCODERPORT = 0;
        public static final int kAbsoluteEncoder = 0;
        public static final int RelMin = 0;
        public static final int Ratio = 0;
        public static final double AbsMin = 0;
    
      }

    public static final class HandlerConstants {
      //public static final double kLowP = 0;
      //public static final double kLowI = 0;
      //public static final double kLowD = 0;
      //public static final double kHighP = 0;
      //public static final double kHighI = 0;
      //public static final double kHighD = 0;
      public static final double kLowInSpeed = 0;
      public static final double kHighOutSpeed = 0;
      public static final double kLowOutSpeed = 0;
      
    }

    public static final class ColorConstants {
      public static final I2C.Port sensorPort = I2C.Port.kOnboard;
    public static final float NoteHue = 0;
    }

    public static final class CANIDs {

      // Arm
        public static final int elbow = 50; // left
        public static final int elbow_follower = 51; // right
        public static final int wrist = 52;
      
        //Handler
      public static final int low_side = 53;
      public static final int high_side = 54;

      // Climber
      public static final int climb_right = 56;
      public static final int climb_left  = 55;
      
    }
 
    public static final class AutoConstants {    /////////// *************************************
        public static final double kMaxSpeedMetersPerSecond = 3.0;   // DriveConstants.kPhysicalMaxSpeedMetersPerSecond/4 ;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                             //   DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;  //   / 4;
        public static final double kPXController = .1;   // 1.5;
        public static final double kPYController = .1;    // 1.5;
        public static final double kPThetaController = 1;  //.06; // 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
        public static final double kPBalancer = .6;  // sin -> Vbus
        public static final double Balancemultiplier = .55;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort  = 0;
        //public static final int kButtonBoxPort_0       = 1; // buttonBox Port 0
        //public static final int kButtonBoxPort_1       = 2; // buttonBox Port 1
        //public static final int kDriverControllerPort4 = 3; // ti launchpad

        public static final int kDriverYAxis                  = 0;
        public static final int kDriverXAxis                  = 1;
        public static final int kDriverRotAxis                = 4;
        public static final int fineControlAxis               = 2;
        public static final int kDriverResetGyroButtonIdx     = 1; // driverJoystick button A
        public static final int kDriverResetOdometryButtonIdx = 3; // driverJoystick button X
        public static final int BALANCE_AUGMENTER             = 4; //  driver stick Y button
        public static final int DriveGenericTester            = 4; //  driver stick Y button
        public static final int kDriverFieldOrientedButtonIdx = 5; // driverJoystick button left-bumper
        public static final int kElbowUpButton                = 3; // driverJoystick button right-bumper
        public static final int kElbowDownButton              = 2; // driverJoystick back button
        public static final int kStopElbowButton              = 1; // driverJoystick start button
        
        //need to start putting stuff on the button board TODO
        
        public static final double kDeadband = 0.05;
    }

    public static final class CamConstant {
        // april tag USB camera connected to RaspBerry Pi runnind PhotonVision
        public static final double PitchAngle =           0.; // Pitch angle of camera in degrees
        public static final double CameraLocationX =      0.; // camera location relative to robots center
        public static final double CameraLocationY =      0.;
        public static final double CameraLocationZ =      0.;
        public static final double CameraLocationX_COMP = Units.inchesToMeters(5.); // camera location relative to robots center
        public static final double CameraLocationY_COMP = 0.;
        public static final double CameraLocationZ_COMP = Units.inchesToMeters(45.);
        public static final double PitchAngle_Comp =      -30.; // Pitch angle of camera in degrees

    }

    public static final class RobotConstants {
      public static final double xcg = Units.inchesToMeters(0);
      public static final double kNominalVoltage = 12.0;
      public static final double kPeriod = TimedRobot.kDefaultPeriod;
    }

    public static final class FieldConstants {
      public static final double chargingstationwidth = Units.inchesToMeters(48.+13.11*2); // Width of the charging station.
      public static final double leaveCommunityDist = Units.inchesToMeters(140.+5);  // distance from Grid to farthest edge of Community
      public static final double Halflength = Units.feetToMeters(11);
      public static final double Gridtostation = Units.feetToMeters(6.);

    }

    public static int COMPBOT;

}



/* +---------------------------------------------------------------------------------+
   |  Absolute Encoder Offsets (Degrees) these values are set in the CANcoders and   |
   |                                  are recorded here for backup purposes          |
   |    Module 0  Front Left    -20.83                                               |
   |           1  Back Left    -188.17                                               |
   |           2  Front Right  -287.40                                               |
   |           3  Back Right   -281.33                                               |
   +---------------------------------------------------------------------------------+  */