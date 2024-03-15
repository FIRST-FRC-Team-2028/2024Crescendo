package frc.robot;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
//import com.revrobotics.SparkMaxLimitSwitch.Direction;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Stations;
import frc.robot.commands.Amp;
import frc.robot.commands.ArmRun;
import frc.robot.commands.AutoDriveOut;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.AutoShootAndMove;
import frc.robot.commands.Climb;
import frc.robot.commands.ClimbWithoutPID;
import frc.robot.commands.DriveGeneric;
//import frc.robot.commands.DriveGenericHeadTest;
import frc.robot.commands.ElbowDown;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElbowUp;
import frc.robot.commands.ExtendClimber;
import frc.robot.commands.InHandler;
import frc.robot.commands.ShootMovePickup;
import frc.robot.commands.ShootPickup;
import frc.robot.commands.Speaker;
import frc.robot.commands.Spit_Back;
//import frc.robot.commands.DriveGeneric;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TravelPosition;
import frc.robot.commands.Wait;
//import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AprilCamera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Handler;
import frc.robot.commands.WristUp;

public class RobotContainer {

    private AprilCamera april;
    private Arm armSubsystem;
    private Handler handlerSubsystem;
    private Drivetrain swerveSubsystem;
    private Climber climber;
    
    //private DriveGeneric driveGeneric;
    private Pigeon2 gyro;
    private SysIdRoutine routine;
    SendableChooser <Command> m_chooser;
    
  
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick mechJoytick1 = new Joystick(OIConstants.kMechControllerPort);
    private final Joystick mechJoytick2 = new Joystick(OIConstants.kMechControllerPort2);
  
    
        public RobotContainer() {
        /*  swapped out to put drive function in teleopPeriodic
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
        */
                if (Constants.DRIVE_AVAILABLE){
                        swerveSubsystem = new Drivetrain();
                        SmartDashboard.putData(swerveSubsystem);
                }
                else swerveSubsystem = null;

               

                if (Constants.ARM_AVAILABLE){
                       armSubsystem = new Arm();
                }
                else armSubsystem = null;

                if (Constants.HANDLER_AVAILABLE){
                        handlerSubsystem = new Handler();
                        
                }

                if (Constants.APRIL_AVAILABLE){
                        april = new AprilCamera();
                }

                gyro = new Pigeon2(0);

                if (Constants.CLIMB_AVAILABLE){ 
                        climber = new Climber(gyro);
                }
                
                //configureButtonBindings();   // postponed until telopInit    

                m_chooser = new SendableChooser<Command>();
                m_chooser.addOption("DoNothing", new Wait(1));
                //m_chooser.addOption("Travel out", new AutoDriveOut(swerveSubsystem));
                m_chooser.setDefaultOption("Travel out", new AutoDriveOut(swerveSubsystem));
                if (Constants.ARM_AVAILABLE) {
                        m_chooser.addOption("2 Note Center", new ShootMovePickup(handlerSubsystem, swerveSubsystem, armSubsystem, Stations.Center));
                        m_chooser.setDefaultOption("Auto Shoot", new AutoShoot(armSubsystem, handlerSubsystem));
                        m_chooser.addOption("2 Note Right", new InstantCommand(() -> gyro.setYaw(-60))
                                .andThen(new ShootMovePickup(handlerSubsystem, swerveSubsystem, armSubsystem, Stations.Right)));
                        m_chooser.addOption("2 Note Left", new InstantCommand(() -> gyro.setYaw(60))
                                .andThen(new ShootMovePickup(handlerSubsystem, swerveSubsystem, armSubsystem, Stations.Left)));
                        m_chooser.addOption("Shoot 1 Pickup 1 Right", new InstantCommand(() -> gyro.setYaw(-60))
                                .andThen(new ShootPickup(handlerSubsystem, swerveSubsystem, armSubsystem, Stations.Right)));
                        m_chooser.addOption("Shoot 1 Pickup 1 Left", new InstantCommand(() -> gyro.setYaw(60))
                                .andThen(new ShootPickup(handlerSubsystem, swerveSubsystem, armSubsystem, Stations.Left)));
                }
        
                //m_chooser.addOption("Shoot, Move, pickup", );
                /*m_chooser.addOption("HeadingTest", new InstantCommand(()-> gyro.setYaw(60))
                        .andThen(new DriveGenericHeadTest(swerveSubsystem, armSubsystem, handlerSubsystem)));*/
                SmartDashboard.putData(m_chooser);
        }
    
        public final Arm getArm() {
                return armSubsystem;
        }
    
        public final Handler getHandler() {
                return handlerSubsystem;
        }

        public final Climber getClimber(){
                return climber;
        }

        public final SendableChooser getAutoChooser() {
                return m_chooser;
        }
        public final AprilCamera getAprilCamera() {
                return april;
        }




     /** map operations to triggers:
      *       driver operations
      *       arm/wrist combinations
      *       handler operations
      */
        public void configureButtonBindings() { 
        // driverJoytick Buttons
            if (swerveSubsystem!=null){
                new JoystickButton(driverJoytick, OIConstants.kDriverResetGyroButtonIdx).
                     onTrue(new InstantCommand(() -> swerveSubsystem.resetGyro())); 
                new JoystickButton(driverJoytick, OIConstants.kDriverResetOdometryButtonIdx).
                     onTrue(new InstantCommand(() -> 
                        swerveSubsystem.resetOdometry(new Pose2d(0., 0., new Rotation2d(0.0)))));
                //new JoystickButton(driverJoytick, OIConstants.kDriverTurnToTarget).
                //        onTrue();
            }
         /*  new JoystickButton(driverJoytick, 5).
          whileTrue(routine.quasistatic(SysIdRoutine.Direction.kForward));
        // whenPressed(() -> swerveSubsystem.resetOdometry(new Pose2d(0., 0., new Rotation2d(0.0))));
        
                // mechJoytick Buttons
         
        if (Constants.PHOTONVISION_AVAILABLE) {
                //new JoystickButton(buttonBox1, OIConstants.kgetRobotPositionButton).
                        //onTrue(new GetRobotPosition(camera));
                //new JoystickButton(buttonBox1, OIConstants.kgetAprilTagButton).
                        //onTrue(new GetAprilTag(camera));
        }*/
        /*new JoystickButton(driverJoytick, OIConstants.kElbowRearmButton).
                onTrue(Commands.runOnce( armSubsystem::rearmArm, armSubsystem));*/
        if(Constants.ARM_AVAILABLE){
        new JoystickButton(driverJoytick, Constants.OIConstants.kArmDuck).
                onTrue(new ArmRun(armSubsystem, ArmConstants.elbowDuck, ArmConstants.wristDuck, 0.25).
                andThen(new InstantCommand(() -> armSubsystem.IamDucked(true))));
        
        
        
        new JoystickButton(mechJoytick1, OIConstants.kDuck).
                toggleOnTrue(new InstantCommand(() -> armSubsystem.Toggleducked()));
        /*new JoystickButton(mechJoytick1, 3).
                whileTrue(new WristUp(armSubsystem,.2));*/
        new JoystickButton(mechJoytick1, OIConstants.kArmTravel).
                onTrue(new TravelPosition(armSubsystem));
                //.andThen(new InstantCommand(()-> armSubsystem.IamDucked(false))));
        new JoystickButton(mechJoytick1, OIConstants.kArmAmp).
                onTrue(new ArmRun(armSubsystem, Constants.ArmConstants.kElbowAmp, ArmConstants.kWristAmp, .25)
                );
        new JoystickButton(mechJoytick1, OIConstants.kArmSubwoofer).
                onTrue(new ArmRun(armSubsystem, Constants.ArmConstants.kElbowSpeaker, ArmConstants.kWristSpeaker, .25)
                //.andThen(new InstantCommand(() -> this.rumble()))
                );
        }
        if (Constants.HANDLER_AVAILABLE){
        if (Constants.COLOR_AVALIBLE){
                new JoystickButton(mechJoytick2, OIConstants.kIntake)
                        .onTrue(new InHandler(handlerSubsystem)
                        .andThen(new Spit_Back(handlerSubsystem))
                        .andThen(new TravelPosition(armSubsystem).onlyIf(handlerSubsystem::doIHaveIt))
                        );
                new JoystickButton(mechJoytick2, OIConstants.kIgnoreSensor)
                        .onTrue(new InstantCommand(() -> handlerSubsystem.useSensorFalse()) );
                new JoystickButton(mechJoytick2, OIConstants.kIgnoreSensor)
                        .onFalse(new InstantCommand(() -> handlerSubsystem.useSensorTrue()) );
        } else {
                new JoystickButton(mechJoytick2, OIConstants.kIntake)
                .whileTrue(new InHandler(handlerSubsystem));
                new JoystickButton(mechJoytick2, OIConstants.kIntake)
                .onFalse(new Spit_Back(handlerSubsystem)
                .andThen(new TravelPosition(armSubsystem)));
        }
        }
        if (Constants.ARM_AVAILABLE){
        new JoystickButton(mechJoytick1, OIConstants.kElbowRearmButton).
                onTrue(Commands.runOnce( armSubsystem::rearmArm, armSubsystem));
        new JoystickButton(mechJoytick1, OIConstants.kArmFloor).
                onTrue(new ArmRun(armSubsystem, ArmConstants.kElbowPreFloow, ArmConstants.kWristPreFloor, 0.5)
                .andThen(new ArmRun(armSubsystem, Constants.ArmConstants.kElbowFloor, Constants.ArmConstants.kWristFloor, .25)));
        /*new JoystickButton(mechJoytick, 3).
                 onTrue(new ArmRun(armSubsystem, 90, 0));*/
        new JoystickButton(mechJoytick2, Constants.OIConstants.kShootSequenceButton ).
                onTrue(new Speaker(handlerSubsystem, april)
                // and return arm/wrist to travelling position 
                .andThen(new TravelPosition(armSubsystem))
                );
        new JoystickButton(mechJoytick2, Constants.OIConstants.shootButton).
                whileTrue(new Amp(handlerSubsystem)
                // back up a bit
                // return arm to travel position
                );
        new JoystickButton(mechJoytick1, Constants.OIConstants.kArmSource).
                onTrue(new ArmRun(armSubsystem, ArmConstants.kElbowSource, ArmConstants.kWristSource, 0.25));
        
        /**/
        new JoystickButton(mechJoytick1, OIConstants.kSwitch).
                onTrue(new InstantCommand(() -> armSubsystem.switchtoFineNudge()));
        new JoystickButton(mechJoytick1, OIConstants.kSwitch).
                onFalse(new InstantCommand(() -> armSubsystem.switchtoStandardNudge()));
        // and switch to the retargetElbow(int) and retargetWrist(int) methods
        /**/
        new JoystickButton(mechJoytick1, OIConstants.kNudgeElbowUp).
                //onTrue(new InstantCommand(() -> armSubsystem.retargetElbow(ArmConstants.elbowNudgeAmount)));
                onTrue(new InstantCommand(() -> armSubsystem.retargetElbow(ArmConstants.nudgeUp)));
        new JoystickButton(mechJoytick1, OIConstants.kNudgeElbowDown).
                //onTrue(new InstantCommand(() -> armSubsystem.retargetElbow(-ArmConstants.elbowNudgeAmount)));
                onTrue(new InstantCommand(() -> armSubsystem.retargetElbow(ArmConstants.nudgeDown)));
        new JoystickButton(mechJoytick2, OIConstants.kNudgeWristUp).
                //onTrue(new InstantCommand(() -> armSubsystem.retargetWrist(ArmConstants.elbowWristAmount)));
                onTrue(new InstantCommand(() -> armSubsystem.retargetWrist(ArmConstants.nudgeUp)));
        new JoystickButton(mechJoytick2, OIConstants.kNudgeWristDown).
                //onTrue(new InstantCommand(() -> armSubsystem.retargetWrist(-ArmConstants.elbowWristAmount)));
                onTrue(new InstantCommand(() -> armSubsystem.retargetWrist(ArmConstants.nudgeDown)));
        }

        if (Constants.CLIMB_AVAILABLE){
                new JoystickButton(mechJoytick2, OIConstants.kClimberExtend).
                        onTrue(new ExtendClimber(climber));
                if(Constants.PID_CLIMB){
                        new JoystickButton(mechJoytick2, OIConstants.kClimberRetract).
                                onTrue(new Climb(climber, gyro, armSubsystem));
                } else {
                        new JoystickButton(mechJoytick2, OIConstants.kClimberRetract).
                                whileTrue(new ClimbWithoutPID(climber));
                }
        }
     }


     void rumble(){
        mechJoytick1.setRumble(GenericHID.RumbleType.kBothRumble,1);
     }
 
     /**
     * @return
     */
        public Command getAutonomousCommand() {
         // 1. Create trajectory settings
                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                          .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
                double scale = -.4;



                Trajectory exampleTrajectory =
                TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
                   new Pose2d(0, 0, new Rotation2d()),
            // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(3, 0, new Rotation2d()),
                        trajectoryConfig);

                Trajectory TwoMeterDrive =
                TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
                   new Pose2d(0, 0, new Rotation2d()),
            // Pass through these two interior waypoints, making an 's' curve path
                        List.of(//new Translation2d(1, 0),
                                new Translation2d(1, 0)
                                //,new Translation2d(2, 0)
            //          ,  new Translation2d(1, 0)
                         ),
            // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(2., 0, new Rotation2d()), //90
                                trajectoryConfig);

                Trajectory LeftHalfMeter =
                TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(0, -0.25)
                        ),
            // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, -0.5, Rotation2d.fromDegrees(0)),
                        trajectoryConfig);


                Trajectory trajectory_test1 = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(
                                new Translation2d(0, 1),
                                new Translation2d(1, 1),
                                new Translation2d(1, 0)//,
                        //new Translation2d(0, 0),
                        //new Translation2d(0.5, 0.5)
                        ),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(90)),
                        trajectoryConfig);



                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        List.of(

                                new Translation2d(  -1 * scale,   0 * scale),
                                new Translation2d(  -1 * scale,   1 * scale),
                                new Translation2d(  -2 * scale ,  1 * scale),
                                new Translation2d(  -2 * scale ,  0 * scale),
                                new Translation2d(  -1 * scale,   0 * scale),
                                new Translation2d(  -1 * scale,   1 * scale),
                                new Translation2d(   0 * scale,   1 * scale),
                                new Translation2d(   0 * scale,   0 * scale),

                                new Translation2d(  -1 * scale,   0 * scale),
                                new Translation2d(  -1 * scale,   1 * scale),
                                new Translation2d(  -2 * scale ,  1 * scale),
                                new Translation2d(  -2 * scale ,  0 * scale),
                                new Translation2d(  -1 * scale,   0 * scale),
                                new Translation2d(  -1 * scale,   1 * scale),
                                new Translation2d(   0 * scale,   1 * scale)),
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),  //360)),
                        trajectoryConfig);



        // 3. Define PID controllers for tracking trajectory
                PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
                PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
                ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
       // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //        trajectory,
        //        swerveSubsystem::getPose,
        //        DriveConstants.kDriveKinematics,
        //        xController,
         //       yController,
          //      thetaController,
          //      swerveSubsystem::setModuleStates,
           //     swerveSubsystem);

                        final SwerveControllerCommand swerveControllerCommand =
                        new SwerveControllerCommand( TwoMeterDrive,
                                swerveSubsystem::getPose,
                                Constants.DriveConstants.kDriveKinematics,
                                xController,
                                yController,
                                thetaController,
                  //      swerveSubsystem::desiredRotation,
                                swerveSubsystem::setModuleStates,
                                swerveSubsystem);




        // 5. Add some init and wrap-up, and return everything
                return Commands.sequence(
                        new InstantCommand(() -> swerveSubsystem.resetOdometry(TwoMeterDrive.getInitialPose())),
                        swerveControllerCommand,
                        new InstantCommand(() -> swerveSubsystem.stopModules()));
        }
    
 
        public Drivetrain getSwerveSS() {
                return swerveSubsystem;
        }

  /*   public Pigeon2 getGyro() {
        return gyro;
    }
*/
     

}