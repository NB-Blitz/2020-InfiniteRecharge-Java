package com.nbblitz.infiniterecharge;

import com.nbblitz.lib.control.XboxController;
import com.nbblitz.lib.drive.Mecanum;
import com.nbblitz.lib.drive.MotorLocation;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages the Robot Timing and Controls
 */
public class Robot extends TimedRobot {

    // Sub-Systems
    private BallManager ballManager;
    private BallLauncher ballLauncher;
    private BallTurret ballTurret;
    private Climber climber;
    private Mecanum drivetrain;
    private Timer timer;

    // Controllers
    private XboxController driveController;
    private XboxController manipController;

    // Constants
    private final double TRIGGER_ACTIVATION_THRESHOLD = .5;

    private final double FRONT_LEFT_FGAIN = 0;
    private final double FRONT_LEFT_PGAIN = 0;
    private final double FRONT_LEFT_IGAIN = 0;
    private final double FRONT_LEFT_DGAIN = 0;
    private final double FRONT_LEFT_DIR = 1;

    private final double BACK_LEFT_FGAIN = 0;
    private final double BACK_LEFT_PGAIN = 0;
    private final double BACK_LEFT_IGAIN = 0;
    private final double BACK_LEFT_DGAIN = 0;
    private final double BACK_LEFT_DIR = 1;

    private final double FRONT_RIGHT_FGAIN = 0;
    private final double FRONT_RIGHT_PGAIN = 0;
    private final double FRONT_RIGHT_IGAIN = 0;
    private final double FRONT_RIGHT_DGAIN = 0;
    private final double FRONT_RIGHT_DIR = -1;

    private final double BACK_RIGHT_FGAIN = 0;
    private final double BACK_RIGHT_PGAIN = 0;
    private final double BACK_RIGHT_IGAIN = 0;
    private final double BACK_RIGHT_DGAIN = 0;
    private final double BACK_RIGHT_DIR = -1;

    private final double TOP_PGAIN = 0.0002;
    private final double TOP_IGAIN = 0;
    private final double TOP_DGAIN = 0.01;
    private final double TOP_FEED_FORWARD = 0.000167;

    private final double BOTTOM_PGAIN = 0.0002;
    private final double BOTTOM_IGAIN = 0;
    private final double BOTTOM_DGAIN = 0.01;
    private final double BOTTOM_FEED_FORWARD = 0.000167;

    private final double LIFT_SPEED = 0.3;

    public Robot() {

        // Initialize Sub-Systems
        ballManager = new BallManager();
        ballLauncher = new BallLauncher();
        ballTurret = new BallTurret();
        climber = new Climber();
        drivetrain = new Mecanum();

        // Initialize Controllers
        driveController = new XboxController(0);
        manipController = new XboxController(1);
    }

    @Override
    public void robotInit() {

        // Controller
        driveController.setUniversalDeadband(0.1);
        driveController.leftXAxis.recenter();
        driveController.leftYAxis.recenter();
        driveController.rightXAxis.recenter();
        driveController.rightYAxis.recenter();

        manipController.setUniversalDeadband(0.15);
        manipController.leftXAxis.recenter();
        manipController.leftYAxis.recenter();
        manipController.rightXAxis.recenter();
        manipController.rightYAxis.recenter();

        // SmartDashboard
        SmartDashboard.putNumber("TOP PGain", TOP_PGAIN);
        SmartDashboard.putNumber("TOP IGain", TOP_IGAIN);
        SmartDashboard.putNumber("TOP DGain", TOP_DGAIN);
        SmartDashboard.putNumber("TOP Feed Forward", TOP_FEED_FORWARD);

        SmartDashboard.putNumber("BOTTOM PGain", BOTTOM_PGAIN);
        SmartDashboard.putNumber("BOTTOM IGain", BOTTOM_IGAIN);
        SmartDashboard.putNumber("BOTTOM DGain", BOTTOM_DGAIN);
        SmartDashboard.putNumber("BOTTOM Feed Forward", BOTTOM_FEED_FORWARD);

        SmartDashboard.putNumber("Shooter RPM", 0);

        // Drivetrain
        drivetrain.setMotorDirection(MotorLocation.BackLeft, BACK_LEFT_DIR);
        drivetrain.setMotorDirection(MotorLocation.BackRight, BACK_RIGHT_DIR);
        drivetrain.setMotorDirection(MotorLocation.FrontLeft, FRONT_LEFT_DIR);
        drivetrain.setMotorDirection(MotorLocation.FrontRight, FRONT_RIGHT_DIR);

        drivetrain.tuneF(MotorLocation.BackLeft, BACK_LEFT_FGAIN);
        drivetrain.tuneP(MotorLocation.BackLeft, BACK_LEFT_PGAIN);
        drivetrain.tuneI(MotorLocation.BackLeft, BACK_LEFT_IGAIN);
        drivetrain.tuneD(MotorLocation.BackLeft, BACK_LEFT_DGAIN);

        drivetrain.tuneF(MotorLocation.BackRight, BACK_RIGHT_FGAIN);
        drivetrain.tuneP(MotorLocation.BackRight, BACK_RIGHT_PGAIN);
        drivetrain.tuneI(MotorLocation.BackRight, BACK_RIGHT_IGAIN);
        drivetrain.tuneD(MotorLocation.BackRight, BACK_RIGHT_DGAIN);

        drivetrain.tuneF(MotorLocation.FrontLeft, FRONT_LEFT_FGAIN);
        drivetrain.tuneP(MotorLocation.FrontLeft, FRONT_LEFT_PGAIN);
        drivetrain.tuneI(MotorLocation.FrontLeft, FRONT_LEFT_IGAIN);
        drivetrain.tuneD(MotorLocation.FrontLeft, FRONT_LEFT_DGAIN);

        drivetrain.tuneF(MotorLocation.FrontRight, FRONT_RIGHT_FGAIN);
        drivetrain.tuneP(MotorLocation.FrontRight, FRONT_RIGHT_PGAIN);
        drivetrain.tuneI(MotorLocation.FrontRight, FRONT_RIGHT_IGAIN);
        drivetrain.tuneD(MotorLocation.FrontRight, FRONT_RIGHT_DGAIN);
    }

    @Override
    public void autonomousInit() {
        timer.reset();
        timer.start();
    }

    @Override
    public void autonomousPeriodic() {
        double time = timer.get();
        if (time < 1)
            drivetrain.drive(0, -.5, 0);
        else
            drivetrain.drive(0, 0, 0);
    }

    @Override
    public void teleopInit() {

        // Controller
        driveController.leftXAxis.recenter();
        driveController.leftYAxis.recenter();
        driveController.rightXAxis.recenter();
        driveController.rightYAxis.recenter();

        manipController.leftXAxis.recenter();
        manipController.leftYAxis.recenter();
        manipController.rightXAxis.recenter();
        manipController.rightYAxis.recenter();
    }

    @Override
    public void teleopPeriodic() {

        // Controller
        driveController.update();
        manipController.update();

        // Manipulator Controls
        boolean primeShooter = manipController.leftTrigger.get() > TRIGGER_ACTIVATION_THRESHOLD;
        boolean shootBalls = manipController.rightTrigger.get() > TRIGGER_ACTIVATION_THRESHOLD;

        boolean autoAim = false; // manipController.aButton.get();
        boolean intakeBalls = manipController.bButton.get();
        boolean pukeBalls = manipController.xButton.get();

        double manualAim = manipController.rightXAxis.get();
        double moveLiftSpeed = manipController.leftYAxis.get();
        double moveWinchSpeed = manipController.rightYAxis.get();

        // Driver Controls
        double xValue = driveController.leftXAxis.get();
        double yValue = driveController.leftYAxis.get();
        double zValue = driveController.rightXAxis.get();

        // Launcher PID
        double topPGain = SmartDashboard.getNumber("TOP PGain", TOP_PGAIN);
        double topIGain = SmartDashboard.getNumber("TOP IGain", TOP_IGAIN);
        double topDGain = SmartDashboard.getNumber("TOP DGain", TOP_DGAIN);
        double topFeedForward = SmartDashboard.getNumber("TOP Feed Forward", TOP_FEED_FORWARD);

        double bottomPGain = SmartDashboard.getNumber("BOTTOM PGain", BOTTOM_PGAIN);
        double bottomIGain = SmartDashboard.getNumber("BOTTOM IGain", BOTTOM_IGAIN);
        double bottomDGain = SmartDashboard.getNumber("BOTTOM DGain", BOTTOM_DGAIN);
        double bottomFeedForward = SmartDashboard.getNumber("BOTTOM Feed Forward", BOTTOM_FEED_FORWARD);

        ballLauncher.tuneTopPID(topPGain, topIGain, topDGain, topFeedForward);
        ballLauncher.tuneBottomPID(bottomPGain, bottomIGain, bottomDGain, bottomFeedForward);

        // Ball Launcher Controls
        int smartDashboardRPM = (int) SmartDashboard.getNumber("Shooter RPM", 0);
        int launcherRPM = 3000;
        if (smartDashboardRPM != 0)
            launcherRPM = smartDashboardRPM;

        boolean readyToShoot = false;
        if (primeShooter) {
            readyToShoot = ballLauncher.primeLauncher(launcherRPM);
        } else {
            readyToShoot = ballLauncher.primeLauncher(0);
        }

        // Ball Storage Controls
        SmartDashboard.putBoolean("Storage Full", ballManager.isFull());
        if (pukeBalls)
            ballManager.puke();
        else if (shootBalls && readyToShoot)
            ballManager.feedShooter();
        else if (intakeBalls)
            ballManager.intakeBalls();
        else
            ballManager.stopIntake();

        // Ball Turret Controls
        if (autoAim)
            ballTurret.setSpeed(0); // TODO: Auto Aim
        else
            ballTurret.setSpeed(manualAim);

        // Climber Controls
        climber.driveLift(moveLiftSpeed * LIFT_SPEED);
        climber.driveWinch(moveWinchSpeed);

        // Drivetrain Controls
        drivetrain.drive(xValue, yValue, zValue);

        // SmartDashboard
        SmartDashboard.putBoolean("Manipulator: Prime Shooter", primeShooter);
        SmartDashboard.putBoolean("Manipulator: Shoot Balls", shootBalls);
        SmartDashboard.putBoolean("Manipulator: Automatic Turret", autoAim);
        SmartDashboard.putBoolean("Manipulator: Intake Balls", intakeBalls);
        SmartDashboard.putBoolean("Manipulator: Puke Balls", pukeBalls);
        SmartDashboard.putNumber("Manipulator: Manual Turret Speed", manualAim);

        SmartDashboard.putNumber("Driver: X-Value", xValue);
        SmartDashboard.putNumber("Driver: Y-Value", yValue);
        SmartDashboard.putNumber("Driver: Z-Value", zValue);

        SmartDashboard.putNumber("Top Motor RPM", ballLauncher.getTopMotorRPM());
        SmartDashboard.putNumber("Bottom Motor RPM", ballLauncher.getBottomMotorRPM());
        SmartDashboard.putNumber("Turret Angle", ballTurret.getAngle());
        SmartDashboard.putNumber("Lidar Distance", ballLauncher.getLidarDistance());
        SmartDashboard.putBoolean("Motors at Speed", readyToShoot);

        SmartDashboard.putNumber("Front Left Velocity", drivetrain.getMotorVelocity(MotorLocation.FrontLeft));
        SmartDashboard.putNumber("Front Right Velocity", drivetrain.getMotorVelocity(MotorLocation.FrontRight));
        SmartDashboard.putNumber("Back Left Velocity", drivetrain.getMotorVelocity(MotorLocation.BackLeft));
        SmartDashboard.putNumber("Back Right Velocity", drivetrain.getMotorVelocity(MotorLocation.BackRight));

        SmartDashboard.putNumber("Front Left Value", drivetrain.getMotorSetValue(MotorLocation.FrontLeft));
        SmartDashboard.putNumber("Front Right Value", drivetrain.getMotorSetValue(MotorLocation.FrontRight));
        SmartDashboard.putNumber("Back Left Value", drivetrain.getMotorSetValue(MotorLocation.BackLeft));
        SmartDashboard.putNumber("Back Right Value", drivetrain.getMotorSetValue(MotorLocation.BackRight));
    }
}
