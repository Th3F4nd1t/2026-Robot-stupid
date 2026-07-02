// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOCompetition;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.limelight.LimelightCamera;
import frc.robot.subsystems.shooter.ShooterArray;
import frc.robot.subsystems.shooter.feeder.Feeder;
import frc.robot.subsystems.shooter.feeder.FeederIO;
import frc.robot.subsystems.shooter.feeder.FeederIOCompetition;
import frc.robot.subsystems.shooter.feeder.FeederIOSim;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOCompetition;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.util.GenericNTButton;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  public static Drivetrain drivetrain;
  public static ShooterArray shooterArray = new ShooterArray();
  public static Indexer indexer;
  public static Intake intake;
  public static LimelightCamera rightHopperLimelight;
  public static LimelightCamera leftHopperLimelight;

  public static Flywheel leftFlywheel;
  public static Flywheel rightFlywheel;
  public static Feeder leftFeeder;
  public static Feeder rightFeeder;

  public static GenericNTButton hubStateButton =
      new GenericNTButton("Hub State", NetworkTableInstance.getDefault().getTable("Hub State"), true);

  public static final XboxController driverController =
      new XboxController(Constants.DriverController.PORT);

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    switch (Constants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.start();

    switch (Constants.currentMode) {
      case REAL:
        leftFlywheel =
            new Flywheel(
                "leftFlywheel",
                new FlywheelIOCompetition(Constants.Shooter.Flywheel.leftFlywheelMotorCANID));
        rightFlywheel =
            new Flywheel(
                "rightFlywheel",
                new FlywheelIOCompetition(Constants.Shooter.Flywheel.rightFlywheelMotorCANID));
        leftFeeder =
            new Feeder(
                "leftFeeder",
                new FeederIOCompetition(Constants.Shooter.Feeder.leftFeederMotorCANID));
        rightFeeder =
            new Feeder(
                "rightFeeder",
                new FeederIOCompetition(Constants.Shooter.Feeder.rightFeederMotorCANID));
        indexer = new Indexer(new IndexerIOCompetition());
        break;

      case SIM:
        leftFlywheel = new Flywheel("leftFlywheel", new FlywheelIOSim());
        rightFlywheel = new Flywheel("rightFlywheel", new FlywheelIOSim());
        leftFeeder = new Feeder("leftFeeder", new FeederIOSim());
        rightFeeder = new Feeder("rightFeeder", new FeederIOSim());
        indexer = new Indexer(new IndexerIOSim());
        break;

      default:
        leftFlywheel = new Flywheel("leftFlywheel", new FlywheelIO() {});
        rightFlywheel = new Flywheel("rightFlywheel", new FlywheelIO() {});
        leftFeeder = new Feeder("leftFeeder", new FeederIO() {});
        rightFeeder = new Feeder("rightFeeder", new FeederIO() {});
        indexer = new Indexer(new IndexerIO() {});
        break;
    }

    robotContainer = new RobotContainer();
    stopAllMotorOutputs();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    stopAllMotorOutputs();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    stopAllMotorOutputs();
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    stopAllMotorOutputs();
  }

  @Override
  public void teleopPeriodic() {
    // Right trigger runs motors forward, left trigger runs motors in reverse.
    double combinedPower =
        MathUtil.applyDeadband(
            driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis(), 0.05);

    leftFlywheel.setPower(combinedPower);
    rightFlywheel.setPower(combinedPower);
    leftFeeder.setFeedSpeed(combinedPower);
    rightFeeder.setFeedSpeed(combinedPower);
    indexer.setIndexSpeed(combinedPower);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    stopAllMotorOutputs();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void stopAllMotorOutputs() {
    if (leftFlywheel != null) {
      leftFlywheel.setPower(0);
    }
    if (rightFlywheel != null) {
      rightFlywheel.setPower(0);
    }
    if (leftFeeder != null) {
      leftFeeder.setFeedSpeed(0);
    }
    if (rightFeeder != null) {
      rightFeeder.setFeedSpeed(0);
    }
    if (indexer != null) {
      indexer.setIndexSpeed(0);
    }
  }

  public static DriverStation.Alliance getAlliance() {
    return DriverStation.isDSAttached()
        ? DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        : DriverStation.Alliance.Blue;
  }
}
