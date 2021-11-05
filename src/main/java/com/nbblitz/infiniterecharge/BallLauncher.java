package com.nbblitz.infiniterecharge;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Manages the top and bottom ball launcher motors
 */
public class BallLauncher {

    // Motors
    private CANSparkMax topMotor;
    private CANSparkMax bottomMotor;

    // Encoders
    private CANPIDController topMotorPID;
    private CANPIDController bottomMotorPID;

    private CANEncoder topMotorEncoder;
    private CANEncoder bottomMotorEncoder;

    // Sensors
    private DigitalInput lidar;
    private Counter lidarCounter;

    // Constants
    private final double TOP_PGAIN = 0.0002;
    private final double TOP_IGAIN = 0;
    private final double TOP_DGAIN = 0.01;
    private final double TOP_IZONE = 0;
    private final double TOP_FEED_FORWARD = 0.000167;

    private final double BOTTOM_PGAIN = 0.0002;
    private final double BOTTOM_IGAIN = 0;
    private final double BOTTOM_DGAIN = 0.01;
    private final double BOTTOM_IZONE = 0;
    private final double BOTTOM_FEED_FORWARD = 0.000167;

    private final double MIN_OUTPUT = -1;
    private final double MAX_OUTPUT = 1;
    private final double RAMP_RATE = 1;
    private final int RPM_BUFFER = 100;
    private final int BACK_SPIN = 2000;

    public BallLauncher() {
        // Motors
        topMotor = new CANSparkMax(MotorIDs.TOP_BALL_LAUNCHER, CANSparkMax.MotorType.kBrushless);
        topMotorPID = topMotor.getPIDController();
        topMotorEncoder = topMotor.getEncoder();

        bottomMotor = new CANSparkMax(MotorIDs.BOTTOM_BALL_LAUNCHER, CANSparkMax.MotorType.kBrushless);
        bottomMotorPID = bottomMotor.getPIDController();
        bottomMotorEncoder = bottomMotor.getEncoder();

        // Sensors
        lidar = new DigitalInput(3);
        lidarCounter = new Counter(lidar);

        // Configure Sensors
        lidarCounter.setMaxPeriod(1.0);
        lidarCounter.setSemiPeriodMode(true);
        lidarCounter.reset();

        // Configure PID
        topMotorPID.setP(TOP_PGAIN);
        topMotorPID.setI(TOP_IGAIN);
        topMotorPID.setD(TOP_DGAIN);
        topMotorPID.setIZone(TOP_IZONE);
        topMotorPID.setFF(TOP_FEED_FORWARD);
        topMotorPID.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);
        topMotor.setClosedLoopRampRate(RAMP_RATE);

        bottomMotorPID.setP(BOTTOM_PGAIN);
        bottomMotorPID.setI(BOTTOM_IGAIN);
        bottomMotorPID.setD(BOTTOM_DGAIN);
        bottomMotorPID.setIZone(BOTTOM_IZONE);
        bottomMotorPID.setFF(BOTTOM_FEED_FORWARD);
        bottomMotorPID.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);
        bottomMotor.setClosedLoopRampRate(RAMP_RATE);
    }

    /**
     * Primes the launcher to fire a ball
     * 
     * @param rpm - The RPM to set the motors to
     * @return - Whether or not the launcher is primed
     */
    public boolean primeLauncher(int rpm) {
        if (rpm == 0) {
            return setLauncherSpeed(rpm, 0);
        } else {
            return setLauncherSpeed(rpm, BACK_SPIN);
        }
    }

    /**
     * Sets the rpm of the top motor
     * 
     * @param rpm - The rpm to set the top motor to
     */
    public void setTopMotorRPM(int rpm) {
        topMotorPID.setReference(rpm, ControlType.kVelocity);
    }

    /**
     * Sets the rpm of the bottom motor
     * 
     * @param rpm - The rpm to set the bottom motor to
     */
    public void setBottomMotorRPM(int rpm) {
        bottomMotorPID.setReference(rpm, ControlType.kVelocity);
    }

    /**
     * Gets the current RPM of the top motor
     * 
     * @return The current RPM of the top motor
     */
    public double getTopMotorRPM() {
        return topMotorEncoder.getVelocity();
    }

    /**
     * Gets the current RPM of the bottom motor
     * 
     * @return The current RPM of the bottom motor
     */
    public double getBottomMotorRPM() {
        return bottomMotorEncoder.getVelocity();
    }

    /**
     * Sets the speed of the ball launcher
     * 
     * @param rpm      - The rpm to set the ball launcher to
     * @param backSpin - Amount of rpm to spin the ball launcher backwards
     * @return if the ball launcher is currently at the desired rpm
     */
    public boolean setLauncherSpeed(int rpm, int backSpin) {
        int topRPM = -(rpm - (backSpin / 2));
        int bottomRPM = (rpm + (backSpin / 2));

        setTopMotorRPM(topRPM);
        setBottomMotorRPM(bottomRPM);

        boolean isTopMotorAtSpeed = (getTopMotorRPM() > (topRPM - RPM_BUFFER)
                && getTopMotorRPM() < (topRPM + RPM_BUFFER));
        boolean isBottomMotorAtSpeed = (getBottomMotorRPM() > (bottomRPM - RPM_BUFFER)
                && getBottomMotorRPM() < (bottomRPM + RPM_BUFFER));

        return isTopMotorAtSpeed && isBottomMotorAtSpeed;
    }

    /**
     * Calculates the rpm of the ball launcher based on the distance to the target
     * 
     * @param distance - Distance to the target in cm?
     * @return RPM of the ball launcher
     */
    public double calcLaunchVelocity(double distance) {
        double rpm = 0;
        if (distance < 18 && distance >= 10) {
            rpm = (-95 * distance) + 4540;
        }

        return rpm;
    }

    /**
     * Gets the lidar distance
     * 
     * @return The distance to the target in cm
     */
    public double getLidarDistance() {
        if (lidarCounter.get() < 1)
            return 0;
        else
            return (lidarCounter.getPeriod() * 1000000.0 / 10.0);
    }

    /**
     * Tunes the PID constants of the top motor
     * 
     * @param p - The new P constant
     * @param i - The new I constant
     * @param d - The new D constant
     * @param f - The new feed forward constant
     */
    public void tuneTopPID(double p, double i, double d, double f) {
        topMotorPID.setP(p);
        topMotorPID.setI(i);
        topMotorPID.setD(d);
        topMotorPID.setFF(f);
    }

    /**
     * Tunes the PID constants of the bottom motor
     * 
     * @param p - The new P constant
     * @param i - The new I constant
     * @param d - The new D constant
     * @param f - The new feed forward constant
     */
    public void tuneBottomPID(double p, double i, double d, double f) {
        bottomMotorPID.setP(p);
        bottomMotorPID.setI(i);
        bottomMotorPID.setD(d);
        bottomMotorPID.setFF(f);
    }

}
