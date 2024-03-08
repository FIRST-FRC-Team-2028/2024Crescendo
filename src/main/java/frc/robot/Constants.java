package frc.robot;

//import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;

/* Table of Contents:
 *    Availablility of Subsystems
 *    ModuleConstants
 *    DriveConstants
 *    ArmConstants
 *    HandlerConstants
 *    ClimberConstants
 *    ColorConstants
 *    CANIDs
 *    AutoConstants
 *    OIConstants
 *    CamConstant
 *    RobotConstants
 *    FieldConstants
 */

public final class Constants {
  public static final boolean PHOTONVISION_AVAILABLE = false;
  public static final boolean DRIVE_AVAILABLE = true;
  public static final boolean ARM_AVAILABLE = true;
  public static final boolean HANDLER_AVAILABLE = true;
  public static final boolean CLIMB_AVAILABLE = true;
  public static final boolean APRIL_AVAILABLE = false;
  public static final boolean COLOR_AVALIBLE = true;
  public static final boolean PID_CLIMB = false;
  public static final boolean Shoot_Pickup_Shoot = true;


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
    public static final double kRampRate = 1;
    public static final double kRampRateT = 0.75;
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
        public static final double kPhysicalMaxSpeedMetersPerSecond = 3;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond /1.5;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                                           kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.8;
        public static final double kptwist = .5;

        public static final double kFineControlSpeed = .5;
        public static final double kFasterSpeed = .2;
  
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
    public static final double kRotateToZero = 2;
   


    }

    public static final class ArmConstants {
      //Elbow PID
      public static final double kElbowP = 0.025;
      public static final double kElbowI = 0;
      public static final double kElbowD = 0;
      //Wrist PID
      public static final double kWristP = 0.01;
      public static final double kWristI = 0;
      public static final double kWristD = 0;

      //Elbow Position  (in degrees up from parallel to the floor)
      public static final double kElbowGround = 10;
      public static final double kElbowHighSpeaker = 80;
      public static final double kElbowSource = 52;
      public static final double kElbowSpeaker = 51;
      public static final double kElbowAmp = 58; //80;
      public static final double kElbowPreFloow = 20;
      public static final double kElbowFloor = -9;  //-5;
      //Wrist Positions (in degrees from perpendicular to arm)
      public static final double kWristSource = 93;
      public static final double kWristHighSpeaker = -41;
      public static final double kWristGround = 0;
      public static final double kWristSpeaker = 3 ;  //22;
      public static final double kWristAmp = 19;  //41;
      public static final double kWristPreFloor = 30;
      public static final double kWristFloor = 56;  //68;
      public static final double kRetract = 85;
      // sweet spot where, when disabled, the arm and handler to not sag
      public static final double elbowSweetSpot = 77.;  
      public static final double wristSweetSpot = 6.4;
      // arm and wrist position to drive under stage
      public static final double elbowDuck = 10.;  
      public static final double wristDuck = 80;
      // arm and wrist position to drive under stage
      public static final double elbowTravel = 15;  //57.;  
      public static final double wristTravel = 50;  //0;
      // arm and wrist position when getting hit
      public static final double elbowHitPosition = 68;
      public static final double wristHitPosition = -1;

      //Tolerances
      public static final double elbowTolerance = .1;
      public static final double wristTolerance = .1;
      //Soft limits 
      public static final float kElbowForwardLimit = 90.f;
      public static final float kElbowReverseLimit = -5f;
      public static final float kWristForwardLimit = 85.f;
      public static final float kWristReverseLimit = -66.f;
      //Encoders
      public static final int ABSENCODERPORT = 0;
      public static final int kAbsoluteEncoder = 0;
      public static final double elbowEncoderFactor = 90./(80.); 
      public static final int RelMin = 6;  //  upright
      public static final int AbsMin = 584;  // parallel to floor
      public static final int RelMax = 45;  // upright
      public static final int AbsMax = 860; // upright
      public static final double Ratio = 39./(860.-584.); //(RelMax-RelMin)/(AbsMax-AbsMin);
      public static final int kAbsoluteEncoderW = 1;
      public static final double wristEncoderFactor = 64.7/16.5;  
      public static final int RelMinW = -32;     //SUBJECT TO CHEANGE
      public static final int AbsMinW = 1345;   //SUBJECT TO CHEANGE
      public static final int RelMaxW = 32;     //SUBJECT TO CHEANGE
      public static final int AbsMaxW = 950;    //SUBJECT TO CHEANGE
      public static final double RatioW = 64./(950-1345);
      //public static final double RatioW = (RelMaxW-RelMinW)./(AbsMaxW-AbsMinW);

      public static final double ElbowCurrentLimit = 25.;
      public static final double kElbowRampRate = 2.;  // seconds
      public static final double kWristRampRate = 2.;  // seconds

        //33.2 = 38.7
        //49.7 = -28
        // 16.5 = -64.7
      public static final double elbowNudgeAmount = 2.;
      public static final double elbowWristAmount = 2.;
      public static final double elbowNudgeAmountFine = 1.;
      public static final double elbowWristAmountFine = 1.;


    }

    public static final class HandlerConstants {
      //public static final double kLowP = 0;
      //public static final double kLowI = 0;
      //public static final double kLowD = 0;
      //public static final double kHighP = 0;
      //public static final double kHighI = 0;
      //public static final double kHighD = 0;
      public static final double kLowPickUpSpeed = .5;
      public static final double kLowToHighSpeed = 1;
      public static final double kHighOutSpeed = 1;
      public static final double kLowOutSpeed = -.7;
      public static final double HighSpinTime = 1.75;
      public static final double TotalShootTime = 3.;
      public static final double kHighAmpSpeed = 0.5;
      public static final double kLowAmpSpeed = -.5;
      public static final double kSpitBackWaitTime = .75;
      
    }

    public static final class ClimberConstants {
        public static final double ExtendSpeed = 0.5;
        public static final double ExtendPosition = 9.2;
        public static final double RetractPosition = 0;
        public static final float extendLimit = 10.1f;
        public static final float retractLimit = 0.f;
        public static final double encoderConversionFactor = 4.0625/(522-398);
    }

    public static final class ColorConstants {
      //public static final I2C.Port sensorPort = I2C.Port.kOnboard;
      //public static final double NoteHue = 0.2;
      public static final int sensordPort = 8;
    }

    public static final class CANIDs {

      // Arm
      public static final int elbow = 50; // left
      public static final int elbow_follower = 51; // right
      public static final int wrist = 52;

      //Handler
      public static final int low_side = 53;
      public static final int high_side = 54;
      public static final int high_side_follower = 55;
      
      // Climber
      public static final int climb_right = 57;
      public static final int climb_left  = 56;
      
    }
 
    public static final class AutoConstants {    /////////// *************************************
        public static final double kMaxSpeedMetersPerSecond = 3.0;   // DriveConstants.kPhysicalMaxSpeedMetersPerSecond/4 ;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                             //   DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;  //   / 4;
        public static final double kPXController = .5;   // 1.5;
        public static final double kPYController = .5;    // 1.5;
        public static final double kPThetaController = 1;  //.06; // 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
        public static final double kToNoteTurnP = 0.1;
        public static final double kToShootTurnP = 1.4;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort  = 0;
        public static final int kMechControllerPort = 1;  // left region of button box (A)
        public static final int kMechControllerPort2 =2;  // right region of button box (B)
        //public static final int kDriverControllerPort4 = 3; // ti launchpad

        public static final int kDriverYAxis                  = 0;
        public static final int kDriverXAxis                  = 1;
        public static final int kDriverRotAxis                = 4;
        public static final int fineControlAxis               = 2;
        public static final int fasterSpeedAxis               = 3;
        public static final int kDriverResetGyroButtonIdx     = 1; // driverJoystick button A
        public static final int kArmDuck                      = 2; // driverJoystick B button
        public static final int kDriverResetOdometryButtonIdx = 3; // driverJoystick button X
        public static final int kDriverRobotOrientedButtonIdx = 5; // driverJoystick button left-bumper

        //Mech contoller buttons left region
        public static final int kArmTravel                    = 1;
        public static final int kArmFloor                     = 2; 
        public static final int kArmSubwoofer                 = 3;
        public static final int kStage                        = 4;
        public static final int kArmAmp                       = 5;
        public static final int kDuck                         = 6;
        public static final int kArmSource                    = 7; 
        public static final int kSwitch                       = 8;
        public static final int kNudgeElbowUp                 = 9;
        public static final int kNudgeElbowDown               = 10;
        public static final int kElbowRearmButton             = 12; 

        //Mech controller buttons right region
        public static final int kNudgeWristUp                 = 1;
        public static final int kNudgeWristDown               = 2;
        public static final int kTestLeftExtend               = 3;
        public static final int kClimberExtend                = 4;
        public static final int shootButton                   = 5;
        public static final int kIntake                       = 6;
        public static final int kTestRightExtend              = 7;
        public static final int kClimberRetract               = 8;
        public static final int kShootSequenceButton          = 10;
        
        public static final double kDeadband = 0.075;
        //public static final int kRetract = 1;


    }

    public static final class CamConstant {
      public static final double camera_Height_Meters = Units.inchesToMeters(7.);
      public static final double target_Height_Meters = Units.inchesToMeters(78.);

    }

    public static final class RobotConstants {
      public static final double xcg = Units.inchesToMeters(0);  // from the geometric centroid
      public static final double kNominalVoltage = 12.0;
      public static final double kPeriod = TimedRobot.kDefaultPeriod;
      public static final double robotLength = Units.inchesToMeters(34.5); //inches
      public static final double robotWidth = Units.inchesToMeters(29.25) ; //inches
      public static final double handlerThickness = Units.inchesToMeters(6.); //inches
    }

    public static final class FieldConstants {
      public static final double leaveWing = Units.inchesToMeters(231.2);    // distance to farthest edge of Wing
      public static final double Halflength = Units.inchesToMeters(250.5+76.1);     // half length of field
      public static final double SpeakerfaceX = Units.inchesToMeters(36.37); // radius of the faces of the speaker
      public static final double noteRadius = Units.inchesToMeters(7.);
      public static final double StageX = Units.inchesToMeters(121.);        // location of the corner of the stage
      public static final double StageY = 0.;                                       // nearest the speaker
      public static final double noteDistance = Units.inchesToMeters(57.);   // distance between notes in wing
      public static final double Speaker2StageY = Units.inchesToMeters(57.); // distance from center face of speaker to nearest Stage
      public static final double StageWidth = Units.inchesToMeters(122.62);
    }

    public static enum Stations {  // center of faces of the speaker base (subwoofer?)
      Right (FieldConstants.SpeakerfaceX*0.5, -FieldConstants.SpeakerfaceX*0.866, -60.), 
      Center(FieldConstants.SpeakerfaceX    , 0.,  0.), 
      Left  (FieldConstants.SpeakerfaceX*0.5, FieldConstants.SpeakerfaceX*0.866, 60.);

      public final double x;
      public final double y;
      public final double heading;
      Stations(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
      }
    };

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