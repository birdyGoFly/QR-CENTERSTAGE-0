package org.firstinspires.ftc.teamcode.Tempest.utility;

public class IntakeController {
    private double rightStoredIntakePosition = 0.435;
    private double leftStoredIntakePosition = 0.605;
    private double rightIntakePosition = 0.15;
    private double leftIntakePosition = 0.87;

    public void setIntakePosition(double percentage)
    {
        //Make sure percentage is within the valid range
        percentage = Math.max(0, Math.min(100,percentage));

        //Set servo positions
        //rightIntakeServo.setPosition(rightStoredIntakePosition + (rightIntakePosition - rightStoredIntakePosition) * (percentage / 100));
        //leftIntakeServo.setPosition(leftStoredIntakePosition + (leftIntakePosition - leftStoredIntakePosition) * (percentage / 100));
    }
    /*
    public static void main (String[] args)
    {
        IntakeController intakecontroller = new IntakeController();

        //intakecontroller.setIntakePosition(50);
    }
     */

}
