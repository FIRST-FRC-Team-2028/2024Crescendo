package frc.robot;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.SparkMaxLimitSwitch.Direction;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ElbowDown;
//import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ElbowUp;
//import frc.robot.commands.DriveGeneric;
import frc.robot.commands.GetAprilTag;
import frc.robot.commands.GetRobotPosition;
import frc.robot.commands.SwerveJoystickCmd;
//import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AprilTagCamera;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Handler;

public class RobotContainer {

    private Arm armSubsystem;
    private Handler handlerSubsystem;
    private Drivetrain swerveSubsystem;
    private AprilTagCamera camera;
    //private DriveGeneric driveGeneric;
    private Pigeon2 gyro;
    private SysIdRoutine routine;
    SendableChooser <Command> m_chooser;
    
  
        private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  
    
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

                if (Constants.PHOTONVISION_AVAILABLE){
                        camera = new AprilTagCamera();
               }

                if (Constants.ARM_AVAILABLE){
                       armSubsystem = new Arm();
                }
                else armSubsystem = null;

                if (Constants.INTAKE_AVAILABLE){
                        handlerSubsystem = new Handler();
                }
                else handlerSubsystem = null;

                gyro = new Pigeon2(0);
                configureButtonBindings();       

                m_chooser = new SendableChooser<>();
        
        }
    
        public final Arm getArm() {
                return armSubsystem;
        }

     //MEE 
        private void configureButtonBindings() { 
        // driverJoytick Buttons
                if (swerveSubsystem!=null){
                new JoystickButton(driverJoytick, OIConstants.kDriverResetGyroButtonIdx).
                  onTrue(new InstantCommand(() -> swerveSubsystem.resetGyro())); 
         new JoystickButton(driverJoytick, OIConstants.kDriverResetOdometryButtonIdx).
                  onTrue(new InstantCommand(() -> 
                  swerveSubsystem.resetOdometry(new Pose2d(0., 0., new Rotation2d(0.0)))));
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
        new JoystickButton(driverJoytick, OIConstants.kElbowUpButton).
                whileTrue(new ElbowUp(armSubsystem));
        new JoystickButton(driverJoytick, OIConstants.kElbowDownButton).
                whileTrue(new ElbowDown(armSubsystem));
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
                   new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            // Pass through these two interior waypoints, making an 's' curve path
                    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                        trajectoryConfig);

                Trajectory TwoMeterDrive =
                TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
                   new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            // Pass through these two interior waypoints, making an 's' curve path
                        List.of(//new Translation2d(1, 0),
                                new Translation2d(1, 0),
                                new Translation2d(2, 0)
            //          ,  new Translation2d(1, 0)
                         ),
            // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(0, 0, Rotation2d.fromDegrees(0)), //90
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
        public AprilTagCamera getPhotonVisionSS() {
                if (Constants.PHOTONVISION_AVAILABLE) {
                        return camera;
                } else return null;
        }

}