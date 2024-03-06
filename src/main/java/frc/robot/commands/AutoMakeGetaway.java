package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.Stations;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;

import frc.robot.subsystems.Drivetrain;

    public class AutoMakeGetaway {
      /** If we are allied with a robot that wants to do it all, 
       *    we can just shoot our preloaded note and then move out of the way.
       *  A good place to move to prepare for Teleop would be the field centerline.
       *  If the robot is initially positioned at the side of the Speaker near the Amp, 
       *     there is an unobstructed path to midfield.
       *  Otherwise a slightly more complicated trajectory is necessary.
       *  This command presumes we have shot and waited some appropriate time; it just drives.
       */
      public Command getAwayCommand(Stations station, Drivetrain drive) {
	  Pose2d initial, destination;
	  Translation2d waypoint;
	  Trajectory getOutOfDodge;
	    Optional<Alliance> ally = DriverStation.getAlliance();
         // 1. Create trajectory settings
                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                          .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory relative to the speaker
	    initial = new Pose2d(station.x, station.y, Rotation2d.fromDegrees(station.heading));

	    if ((station == Stations.Right && ally.get() == Alliance.Red) ||
		(station == Stations.Left && ally.get() == Alliance.Blue)) {
		destination = new Pose2d(FieldConstants.Halflength-RobotConstants.robotLength*1.5,
			                 0., new Rotation2d());

                getOutOfDodge =
                TrajectoryGenerator.generateTrajectory(initial,null,destination, //90
                                trajectoryConfig);
	    } else if (station == Stations.Center) {
	      if (ally.get() == Alliance.Red) {
		  destination = new Pose2d(FieldConstants.Halflength-RobotConstants.robotLength*1.5
			          , FieldConstants.Speaker2StageY+FieldConstants.StageWidth*.5+RobotConstants.robotLength*1.5
				  , new Rotation2d());
	      }else{
		  destination = new Pose2d(FieldConstants.Halflength-RobotConstants.robotLength*1.5
			          , -FieldConstants.Speaker2StageY-FieldConstants.StageWidth*.5-RobotConstants.robotLength*1.5
				  , new Rotation2d());
	      }
		waypoint = new Translation2d(Units.inchesToMeters(6.), destination.getY() - initial.getY());
                getOutOfDodge = TrajectoryGenerator.generateTrajectory(initial, List.of(waypoint), destination, 
                                trajectoryConfig);
	    } else {
	      if (ally.get() == Alliance.Red) {
		  destination = new Pose2d(FieldConstants.Halflength-RobotConstants.robotLength*1.5
			          , FieldConstants.Speaker2StageY+FieldConstants.StageWidth*.5+RobotConstants.robotLength*1.5
				  , new Rotation2d());
	      }else{
		  destination = new Pose2d(FieldConstants.Halflength-RobotConstants.robotLength*1.5
			          , -FieldConstants.Speaker2StageY-FieldConstants.StageWidth*.5-RobotConstants.robotLength*1.5
				  , new Rotation2d());
	      }
		waypoint = new Translation2d(Units.inchesToMeters(6.), destination.getY() - initial.getY());
                getOutOfDodge = TrajectoryGenerator.generateTrajectory(initial, List.of(waypoint), destination, 
                                trajectoryConfig);
	    }


        // 3. Define PID controllers for tracking trajectory
                PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
                PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
                ProfiledPIDController thetaController = new ProfiledPIDController(
                        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
       // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        //        trajectory,
        //        drive::getPose,
        //        DriveConstants.kDriveKinematics,
        //        xController,
         //       yController,
          //      thetaController,
          //      drive::setModuleStates,
           //     drive);

                        final SwerveControllerCommand swerveControllerCommand =
                        new SwerveControllerCommand( getOutOfDodge,
                                drive::getPose,
                                DriveConstants.kDriveKinematics,
                                xController,
                                yController,
                                thetaController,
                  //      drive::desiredRotation,
                                drive::setModuleStates,
                                drive);

        // 5. Add some init and wrap-up, and return everything
                return Commands.sequence(
                        new InstantCommand(() -> drive.resetOdometry(getOutOfDodge.getInitialPose())),
                        swerveControllerCommand,
                        new InstantCommand(() -> drive.stopModules()));
        }
    }
