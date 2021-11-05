package com.nbblitz.infiniterecharge;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

/**
 * Manages the rotation of the ball launcher
 */
public class BallTurret {

    // Motor
    private CANSparkMax turretMotor;
    private CANPIDController turretMotorPID;
    private CANEncoder turretMotorEncoder;

    // Constants
    private final double COUNTS_PER_MOTOR_REVOLUTION = 44.4;
    private final double SMALL_GEAR_TEETH = 12;
    private final double LARGE_GEAR_TEETH = 154;
    private final double COUNTS_PER_TURRET_REVOLUTION = COUNTS_PER_MOTOR_REVOLUTION
            * (LARGE_GEAR_TEETH / SMALL_GEAR_TEETH);
    private final double COUNTS_PER_DEGREE = COUNTS_PER_TURRET_REVOLUTION / 360;

    private final double TURRET_PGAIN = 0;
    private final double TURRET_IGAIN = 0;
    private final double TURRET_DGAIN = 0;
    private final double TURRET_IZONE = 0;
    private final double TURRET_FEED_FORWARD = 0;
    private final double TURRET_ANGLE_THRESHOLD = 5;
    private final double TURRET_POSITION_CONVERSION = 1 / COUNTS_PER_DEGREE;

    // Vars
    private double turrentSetPoint = 0;

    public BallTurret() {
        // Motor
        turretMotor = new CANSparkMax(MotorIDs.TURRET, CANSparkMax.MotorType.kBrushless);
        turretMotorPID = turretMotor.getPIDController();
        turretMotorEncoder = turretMotor.getEncoder();

        // Configure PID
        turretMotorPID.setP(TURRET_PGAIN);
        turretMotorPID.setI(TURRET_IGAIN);
        turretMotorPID.setD(TURRET_DGAIN);
        turretMotorPID.setIZone(TURRET_IZONE);
        turretMotorPID.setFF(TURRET_FEED_FORWARD);
        turretMotorPID.setOutputRange(-1, 1);
        turretMotorEncoder.setPositionConversionFactor(TURRET_POSITION_CONVERSION);
    }

    /**
     * Sets the home for the turret at the current position
     */
    public void setHome() {
        turretMotorEncoder.setPosition(0);
    }

    /**
     * Gets the current angle of the turret
     * 
     * @return the current angle of the turret
     */
    public double getAngle() {
        return turretMotorEncoder.getPosition();
    }

    /**
     * Rotates the turret relative to it's current position
     * 
     * @param angle The angle to rotate the turret
     * @return if the turret was able to reach the desired angle
     */
    public boolean setRotationRelative(double angle) {
        turrentSetPoint = getAngle() + angle;
        return setPosition(turrentSetPoint);
    }

    /**
     * Sets the turret to a specific angle relative to it's home
     * 
     * @param angle - The angle to set the turret to
     * @return if the turret was able to reach the desired angle
     */
    public boolean setRotationAbsolute(double angle) {
        turrentSetPoint = angle;
        return setPosition(turrentSetPoint);
    }

    /**
     * Attempt to rotate the turret towards a specific angle
     * 
     * @param angle - The angle to set the turret to
     * @return if the turret was able to reach the desired angle
     */
    public boolean setPosition(double angle) {
        turretMotorPID.setReference(angle, ControlType.kPosition);
        double currentAngle = turretMotorEncoder.getPosition();
        return (currentAngle <= (angle + TURRET_ANGLE_THRESHOLD)) && (currentAngle >= (angle - TURRET_ANGLE_THRESHOLD));
    }

    /**
     * Rotates the turret at a certain speed
     * 
     * @param speed - The speed to rotate the turret at (-1 to 1)
     */
    public void setSpeed(double speed) {
        turretMotor.set(speed);
    }
}
