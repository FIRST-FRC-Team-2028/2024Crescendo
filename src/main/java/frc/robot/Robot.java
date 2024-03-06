package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

//import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.DataLogManager;   //MEE
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmRun;
import frc.robot.commands.TravelPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.I2C;           // Maddox: Color sensor
import edu.wpi.first.wpilibj.util.Color;    // Maddox: Color sensor

import com.revrobotics.ColorSensorV3;       // Maddox: Color sensor

import frc.robot.subsystems.Handler;

//import com.ctre.phoenix.sensors.Pigeon2_Faults;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick mechJoytick1 = new Joystick(OIConstants.kMechControllerPort);
    private final Joystick mechJoytick2 = new Joystick(OIConstants.kMechControllerPort2);
    private Drivetrain swerveSubsystem;
    Arm arm;
    Climber climber;
   // private Pigeon2 pigeon;

    private PowerDistribution PDH;
    //private AnalogInput pixyCam;
    //private PWM lights;
    // private Timer timer = new Timer();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {

        // Instantiate our RobotContainer.
        //      o build subsystems based on what is available,
        //      o perform all our button bindings,
        //      o put our autonomous chooser on the dashboard.

        PDH = new PowerDistribution(1, ModuleType.kRev);
       
        m_robotContainer = new RobotContainer();
        //pigeon = m_robotContainer.getGyro();
        //PortForwarder.add(1182, "photonvision.local",5800 );

        //DataLogManager.start();   //MEE
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Maddox: Color sensor
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        //lights.setSpeed(0.93);
        SendableChooser<Command> slector = m_robotContainer.getAutoChooser();
        m_autonomousCommand = slector.getSelected();
        if (arm == null) arm = m_robotContainer.getArm();
        arm.setBrakeMode();
        (new TravelPosition(arm)).schedule();
        //pigeon.setYaw(180);  Do we need to initialize the gyro?
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    boolean buttonbinding = false;

    @Override
    public void teleopInit() {
        m_robotContainer.configureButtonBindings();
        buttonbinding = true;
        //UsbCamera usbCamera = new UsbCamera("Front Cam", 0);
        //MjpegServer mjpegServer1 = new MjpegServer("Front Server", 1181);
        //mjpegServer1.setSource(usbCamera);
        //CameraServer.startAutomaticCapture();

         // Creates the CvSink and connects it to the UsbCamera
        //CvSink cvSink = new CvSink("opencv_USB Camera 0");
        //cvSink.setSource(usbCamera);

        // Creates the CvSource and MjpegServer [2] and connects them
        //CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
        //MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
        //mjpegServer2.setSource(outputStream);

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        if (climber == null) climber = m_robotContainer.getClimber();
        climber.zeroSoftLimit();
        if (arm == null) arm = m_robotContainer.getArm();
        arm.setBrakeMode();
        // start up arm PID controller; move to Travel position
        (new TravelPosition(arm)).schedule();
        
        // to drive in teleopPeriodic rather than defaultCommand {
        this.swerveSubsystem = m_robotContainer.getSwerveSS();
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
      //  this.swerveSubsystem.setCoastMode();

    }

    double smoothedXSpeed = 0.;
    double smoothedYSpeed = 0.;
    double smoothedTurningSpeed = 0.;
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {

        /* if (driverJoytick.getRawButton(3)) {
                System.out.println("Button Pressed");
        } */

        //SmartDashboard.putNumber("BotA", pigeon.getAngle());
        SmartDashboard.putNumber("BatV", PDH.getVoltage());

        //   SmartDashboard.putNumber("Pitch", pigeon.getPitch().getValue().doubleValue());
        //  SmartDashboard.putNumber("Yaw", pigeon.getYaw().getValue().doubleValue());
        //  SmartDashboard.putNumber("Angle", pigeon.getAngle());

        //SmartDashboard.putNumber()

        // CRG use SwerveSubsystem drive methods similar to the SwerveJoysickCmd  {
        if (Constants.DRIVE_AVAILABLE){
            // 1. Get real-time joystick inputs
            double xSpeed = -driverJoytick.getRawAxis(OIConstants.kDriverXAxis); // Negative values go forward
            double ySpeed = -driverJoytick.getRawAxis(OIConstants.kDriverYAxis);
            double turningSpeed = -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis);

            // 2. Apply deadband
            xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
            ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
            //System.out.println("Deadband Applied");
            //System.out.println("X: " + String.format("%.3f", xSpeed)
            //                + " Y: " + String.format("%.3f", ySpeed)
            //                + " R: " + String.format("%.3f", turningSpeed));
            xSpeed*=1.-DriveConstants.kFineControlSpeed*driverJoytick.getRawAxis(OIConstants.fineControlAxis);
            ySpeed*=1.-DriveConstants.kFineControlSpeed*driverJoytick.getRawAxis(OIConstants.fineControlAxis);
            turningSpeed*=1.-DriveConstants.kFineControlSpeed*driverJoytick.getRawAxis(OIConstants.fineControlAxis);
            //    Smooth driver inputs
            smoothedXSpeed = smoothedXSpeed + (xSpeed - smoothedXSpeed) * .08;
            smoothedYSpeed = smoothedYSpeed + (ySpeed - smoothedYSpeed) * .08;
            smoothedTurningSpeed = smoothedTurningSpeed + (turningSpeed - smoothedTurningSpeed) * .08;
            //    System.out.println("Raw Joystick Values");
            //    System.out.println("X: " + String.format("%.3f", xSpeed) 
            //                    + " Y: " + String.format("%.3f", ySpeed)
            //                    + " R: " + String.format("%.3f", turningSpeed));

            // if (driverJoytick.getRawButton(OIConstants.BALANCE_AUGMENTER)) {
            //     double augment = Math.sin(Math.toRadians(pigeon.getPitch()-1));
            //     //System.out.println(augment);
            //     smoothedXSpeed+=augment*.036;
            // }
            xSpeed = smoothedXSpeed;
            ySpeed = smoothedYSpeed;
            turningSpeed = smoothedTurningSpeed;

            // 3. Make the driving smoother
            xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
            turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
            //System.out.println("Smoothing Applied");
            //System.out.println("X: " + String.format("%.3f", xSpeed)
            //                + " Y: " + String.format("%.3f", ySpeed)
            //                + " R: " + String.format("%.3f", turningSpeed));

            //System.out.println("=====================");

            // 4. Construct desired chassis speeds
            ChassisSpeeds chassisSpeeds;
            if ( !driverJoytick.getRawButton(OIConstants.kDriverRobotOrientedButtonIdx)) {
                // Relative to field
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
            } else {
                // Relative to robot
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            }
            //System.out.println("Chassis Speeds");
            //System.out.println("X: " + String.format("%.3f", xSpeed)
            //                + " Y: " + String.format("%.3f", ySpeed)
            //                + " R: " + String.format("%.3f", swerveSubsystem.getRotation2d()));

            //System.out.println("Encoder: " + frontleftsteerencoder.getPosition());

            swerveSubsystem.drive(chassisSpeeds);
            // MrG recommends looking at swerveSubsystem.driveIt  TODO

            // 5. Convert chassis speeds to individual module states
            //SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            /* this should be done in the SwerveSubsystem */
            // SwerveModuleState[] moduleStates = swerveSubsystem.chassis2ModuleStates(chassisSpeeds);

            // 6. Output each module states to wheels
            //swerveSubsystem.setModuleStates(moduleStates);
        
            // steps 4-6 should be accomplished by the swerve subsystem via a method such as
            // swerveSubsystem.driveit(xSpeed, ySpeed, turningSpeed, fieldoriented);
            //}
    
            //swerveSubsystem.reportStatesToSmartDashbd(moduleStates);
        }
    }

    Handler handler;
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().getActiveButtonLoop().clear();
        // Gets rid of all button bindings
        
        //if (arm == null) arm = m_robotContainer.getArm();
        //arm.setBrakeMode();
        // immediately move the arm to the floor and end the PID control
        /*(new ArmRun(arm, ArmConstants.kElbowPreFloow, ArmConstants.kWristPreFloor, 2)
            .andThen(new ArmRun(arm, Constants.ArmConstants.kElbowFloor, Constants.ArmConstants.kWristFloor, .25))
            .andThen(new InstantCommand(()-> arm.stopIt()))).schedule();*/
    

        handler = m_robotContainer.getHandler();
        if (Constants.CLIMB_AVAILABLE){
            climber = m_robotContainer.getClimber();
        }
        arm = m_robotContainer.getArm();
    }

    /** This function is called periodically during test mode. */
    @Override

    public void testPeriodic() {

        if (new JoystickButton(mechJoytick2, OIConstants.kNudgeElbowUp).getAsBoolean()) {
            arm.elbowUpSlow();  
        } 
        else if (new JoystickButton(mechJoytick2, OIConstants.kNudgeElbowDown).getAsBoolean() ) {
            arm.elbowDownSlow();  
        }
        else {
            arm.stopElbow();
            //System.out.println("Stopped Elbow");
        }

        if (new JoystickButton(mechJoytick2,OIConstants.kNudgeWristDown).getAsBoolean()) {
            arm.moveWrist(-.1);  
        }else if (new JoystickButton(mechJoytick2,OIConstants.kNudgeWristUp).getAsBoolean()) {
            arm.moveWrist(.1);  
        } else {
            arm.moveWrist(0.);
            //System.out.println("Stopped Wrist");
        }
      
        if (Constants.CLIMB_AVAILABLE){

            if (new JoystickButton(mechJoytick1, OIConstants.kSwitch).getAsBoolean()) {
                climber.disableLimit();
            }else {
                climber.enableSoftLimit();
            }
                //climber.getRoll();
                if (new JoystickButton(mechJoytick1, OIConstants.kTestLeftExtend).getAsBoolean()) {
                    climber.extend_left(.5);
                    //System.out.println(String.format("Climber Position = %4.f" , m_robotContainer.getClimber().getPositionDriver()));
                    System.out.println(climber.getPositionDriver());

                }
                else if (new JoystickButton(mechJoytick1, OIConstants.kClimberExtend).getAsBoolean() ) {
                    climber.extend_left(-.5);
                    //System.out.println(String.format("Climber Position = %.4f" , m_robotContainer.getClimber().getPositionDriver()));
                    System.out.println(climber.getPositionDriver());

                }
                else{
                    climber.stop_left();
                }
            // reuse buttons in test mode for other than they were designated in teleop
             if (new JoystickButton(mechJoytick1, OIConstants.kTestRightExtend).getAsBoolean()) {
                    climber.extend_right(.5);
                    //System.out.println(String.format("Climber Position = %4.f" , m_robotContainer.getClimber().getPositionDriver()));
                    System.out.println(climber.getPositionLeveler());

                }
                else if (new JoystickButton(mechJoytick2, OIConstants.kElbowRearmButton).getAsBoolean() ) {
                    climber.extend_right(-.5);
                    //System.out.println(String.format("Climber Position = %.4f" , m_robotContainer.getClimber().getPositionDriver()));
                    System.out.println(climber.getPositionLeveler());

                }
                else{
                    climber.stop_right();
                }
            
            if (new JoystickButton(mechJoytick2, OIConstants.kArmSource).getAsBoolean()) {
                climber.zeroSoftLimit();
            }
            
        }
        // handler 
        if (new JoystickButton(mechJoytick2, OIConstants.shootButton).getAsBoolean()) {
            handler.low_out();  
        } else if (new JoystickButton(mechJoytick2, OIConstants.kIntake).getAsBoolean() && !handler.useSensor() ) {
            handler.low_PickUp();  
            System.out.println(handler.useSensor());
        } else handler.stop();

    }
}
