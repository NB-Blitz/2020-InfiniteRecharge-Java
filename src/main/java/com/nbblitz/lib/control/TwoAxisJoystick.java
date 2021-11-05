package com.nbblitz.lib.control;

import edu.wpi.first.wpilibj.Joystick;

public class TwoAxisJoystick {

    // Buttons
    public Button trigger = new Button();
    public Button button2 = new Button();
    public Button button3 = new Button();
    public Button button4 = new Button();
    public Button button5 = new Button();
    public Button button6 = new Button();
    public Button button7 = new Button();
    public Button button8 = new Button();

    private final int TRIGGER_ID = 1;
    private final int BUTTON2_ID = 2;
    private final int BUTTON3_ID = 3;
    private final int BUTTON4_ID = 4;
    private final int BUTTON5_ID = 5;
    private final int BUTTON6_ID = 6;
    private final int BUTTON7_ID = 7;
    private final int BUTTON8_ID = 8;

    // Axis
    public Axis xAxis = new Axis();
    public Axis yAxis = new Axis();
    public Axis dial = new Axis();

    private final int X_AXIS_ID = 0;
    private final int Y_AXIS_ID = 1;
    private final int DIAL_ID = 2;

    // Joystick
    Joystick joystick;

    /**
     * Defines a 2-Axis Joystick
     * 
     * @param port - Port ID the joystick is plugged in to
     */
    public TwoAxisJoystick(int port) {
        this.joystick = new Joystick(port);
    }

    /**
     * Updates all buttons and axis with the latest information. Only run this
     * function once per loop!
     */
    public void update() {

        // Buttons
        trigger.update(joystick.getRawButton(TRIGGER_ID));
        button2.update(joystick.getRawButton(BUTTON2_ID));
        button3.update(joystick.getRawButton(BUTTON3_ID));
        button4.update(joystick.getRawButton(BUTTON4_ID));
        button5.update(joystick.getRawButton(BUTTON5_ID));
        button6.update(joystick.getRawButton(BUTTON6_ID));
        button7.update(joystick.getRawButton(BUTTON7_ID));
        button8.update(joystick.getRawButton(BUTTON8_ID));

        // Axis
        xAxis.update(joystick.getRawAxis(X_AXIS_ID));
        yAxis.update(joystick.getRawAxis(Y_AXIS_ID));
        dial.update(joystick.getRawAxis(DIAL_ID));
    }

    /**
     * Sets the deadband for all controller axis
     * 
     * @param deadband - The deadband value
     */
    public void setUniversalDeadband(double deadband) {
        xAxis.setDeadband(deadband);
        yAxis.setDeadband(deadband);
        dial.setDeadband(deadband);
    }
}