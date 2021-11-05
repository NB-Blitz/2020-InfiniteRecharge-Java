package com.nbblitz.infiniterecharge;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Manages the lift and winch
 */
public class Climber {
    private TalonSRX liftMotor;
    private TalonSRX winchMotor;

    public Climber() {
        liftMotor = new TalonSRX(12);
        winchMotor = new TalonSRX(14);
    }

    /**
     * Sets the lift motor's home to the current position
     */
    public void homeLift() {
        liftMotor.setSelectedSensorPosition(0);
    }

    /**
     * Moves the lift at a set speed
     * 
     * @param speed - Speed in percentage (-1 - 1)
     */
    public void driveLift(double speed) {
        liftMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    /**
     * Moves the winch at a set speed
     * 
     * @param speed - Speed in percentage (-1 - 1)
     */
    public void driveWinch(double speed) {
        winchMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }
}
