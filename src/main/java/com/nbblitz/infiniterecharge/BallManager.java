package com.nbblitz.infiniterecharge;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Manages ball intake and storage
 */
public class BallManager {

    // Motors
    private CANSparkMax intakeMotor;
    private CANSparkMax feederMotor1;
    private CANSparkMax feederMotor2;
    private TalonSRX launcherFeedMotor;

    // Switches
    private DigitalInput ballAtIntakeSwitch;
    private DigitalInput firstStageSwitch;
    private DigitalInput storageFullSwitch;

    // Constants
    private final double INTAKE_SPEED = .25;
    private final double STORAGE_INTAKE_SPEED = 1;
    private final double STORAGE_SHOOT_SPEED = 1;
    private final double LAUNCHER_FEED_SPEED = -1;

    // Vars
    private boolean secondBallMoved = false;
    private int unblockedCount = 0;
    private int blockedCount = 0;

    public BallManager() {
        intakeMotor = new CANSparkMax(11, MotorType.kBrushless);
        feederMotor1 = new CANSparkMax(9, MotorType.kBrushless);
        feederMotor2 = new CANSparkMax(10, MotorType.kBrushless);
        launcherFeedMotor = new TalonSRX(8);

        ballAtIntakeSwitch = new DigitalInput(0);
        firstStageSwitch = new DigitalInput(1);
        storageFullSwitch = new DigitalInput(2);
    }

    /**
     * Intakes balls from the floor into the ball storage
     */
    public void intakeBalls() {
        // Get Data
        boolean storageFull = storageFullSwitch.get();
        boolean ballAtIntake = ballAtIntakeSwitch.get();
        boolean firstStage = firstStageSwitch.get();

        // SmartDashboard
        SmartDashboard.putBoolean("Storage Full Line Break", storageFull);
        SmartDashboard.putBoolean("Intake Line Break", ballAtIntake);
        SmartDashboard.putBoolean("First Stage Line Break", firstStage);

        // Intake
        if (storageFull) {
            intakeMotor.set(INTAKE_SPEED);

            if (!ballAtIntake) {
                feederMotor1.set(-STORAGE_INTAKE_SPEED);
                feederMotor2.set(STORAGE_INTAKE_SPEED);
                secondBallMoved = false;
                unblockedCount = 0;
            } else if (secondBallMoved && firstStage) {
                if (blockedCount <= 30) {
                    blockedCount++;
                } else {
                    feederMotor1.set(0);
                    feederMotor2.set(0);
                    blockedCount = 0;
                }
            }

            if (ballAtIntake && unblockedCount == 10) {
                secondBallMoved = true;
            } else if (ballAtIntake) {
                unblockedCount++;
            }
        } else {
            intakeMotor.set(0);
            feederMotor1.set(0);
            feederMotor2.set(0);
        }

        launcherFeedMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Feeds balls from the ball storage to the shooter
     */
    public void feedShooter() {
        intakeMotor.set(INTAKE_SPEED);
        feederMotor1.set(-STORAGE_SHOOT_SPEED);
        feederMotor2.set(STORAGE_SHOOT_SPEED);
        launcherFeedMotor.set(ControlMode.PercentOutput, LAUNCHER_FEED_SPEED);
    }

    /**
     * Expels balls from the shooter & ball storage towards the floor
     */
    public void puke() {
        intakeMotor.set(-INTAKE_SPEED);
        feederMotor1.set(STORAGE_SHOOT_SPEED);
        feederMotor2.set(-STORAGE_SHOOT_SPEED);
        launcherFeedMotor.set(ControlMode.PercentOutput, -LAUNCHER_FEED_SPEED);
    }

    public void stopIntake() {
        intakeMotor.set(0);
        feederMotor1.set(0);
        feederMotor2.set(0);
        launcherFeedMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Checks if the ball storage is full and uploads data to SmartDashboard
     * 
     * @return true if the ball storage is full
     */
    public boolean isFull() {
        // Get Data
        boolean storageFull = storageFullSwitch.get();
        boolean ballAtIntake = ballAtIntakeSwitch.get();
        boolean firstStage = firstStageSwitch.get();

        // SmartDashboard
        SmartDashboard.putBoolean("Storage Full Line Break", storageFull);
        SmartDashboard.putBoolean("Intake Line Break", ballAtIntake);
        SmartDashboard.putBoolean("First Stage Line Break", firstStage);

        // Export
        return !storageFull;
    }
}
