package org.firstinspires.ftc.teamcode.Meet2;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
* This file is a rewritten form of MyFirstTeleOpWITHARM, since it was rather spaghetti-coded
* on the fly at Meet 1. I finally decided to make this new version when the arm was proving very
* difficult to debug.

    Important features to add are:

☐ Drone launcher
☐ Autonomous
☑ Second grabber
☐ Hang macro

 */

@TeleOp(name="Meet 2 TeleOp", group="Iterative OpMode")

public class MySecondTeleOp extends OpMode {

    //╔╦╗┌─┐┌┬┐┌─┐┬─┐┌─┐
    //║║║│ │ │ │ │├┬┘└─┐
    //╩ ╩└─┘ ┴ └─┘┴└─└─┘
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor armExtension = null;
    private DcMotor armRotation = null;

    //╔═╗┌─┐┬─┐┬  ┬┌─┐┌─┐
    //╚═╗├┤ ├┬┘└┐┌┘│ │└─┐
    //╚═╝└─┘┴└─ └┘ └─┘└─┘
    private Servo gate = null;
    private Servo drone = null;
    private Servo wrist = null;
    private Servo grab1 = null;
    private Servo grab2 = null;

    //╦  ╦┌─┐┬─┐┬┌─┐┌┐ ┬  ┌─┐┌─┐
    //╚╗╔╝├─┤├┬┘│├─┤├┴┐│  ├┤ └─┐
    // ╚╝ ┴ ┴┴└─┴┴ ┴└─┘┴─┘└─┘└─┘
    boolean intakeMode = false;
    boolean hangRobot = false;
    boolean launchDrone = false;
    private ElapsedTime runtime = new ElapsedTime();

    int ArmPosition = 0;
    boolean isLeftGrabberCurrentlyOpen = true;
    boolean isRightGrabberCurrentlyOpen = true;

    boolean leftGrabberButtonCheck = false;
    boolean rightGrabberButtonCheck = false;
    boolean intakeModeButtonCheck = false;

    int armRotationOffset = 0;
    int armExtensionOffset = 0;

    int intakePosition = 0;
    int carryingPosition = 1;
    int placementPosition1 = 2;
    int drivingPosition = 3;
    int hangPosition = 4;



    //--------------------------
    //ENCODER SETUP:

    double armRotationPower = 0.3; //This value can be from 0-1. I like to keep it low so that it does not break anything whilst I am debugging
    int armRotationTarget = 0;
    double armExtensionPower = 1;
    int armExtensionTarget = 0;

    //--------------------------
    //POSITION SETUP:

    //Drone
    double droneArmedPosition = 0;
    double droneLaunchPosition = 1;
    //Wrist
    double wristIntakePosition = 0.55;
    double wristPlacementPosition = 0.05;
    //Grabber
    double grabberOpenPosition = 0.3;
    double grabberClosedPosition = 0.9;
    int armHangExtensionPosition = 500;

    //------------------------------------------------------------------------------------------------
    @Override
    public void init() {
        telemetry.addData("Current Status", "Robot Has Been Initialized");

        // Motors
        frontLeft = hardwareMap.get(DcMotor.class, "front left");
        frontRight = hardwareMap.get(DcMotor.class, "front right");
        backRight = hardwareMap.get(DcMotor.class, "back right");
        backLeft = hardwareMap.get(DcMotor.class, "back left");
        armExtension = hardwareMap.get(DcMotor.class, "linear actuator");
        armRotation = hardwareMap.get(DcMotor.class, "linear actuator rotation");

        //Servos
        //gate = hardwareMap.get(Servo.class, "gate servo"); //This is a backup for if we revert back to the toggling gate
        drone = hardwareMap.get(Servo.class, "drone servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");//The "wrist" is the servo which controls the rotation of the grabber.
        grab1 = hardwareMap.get(Servo.class, "left grabber servo"); //This is the LEFT grabber.
        grab2 = hardwareMap.get(Servo.class, "right grabber servo"); //This is the RIGHT grabber.

        //SETUP// This sets the directions for the motors for mecanum drive (being able to drive forward, backward, left, right, rotate left, and rotate right).
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //ENCODER SETUP// This sets the mode of the encoders on the arm motors.
        armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void init_loop() {
        //Anything here would run after the PLAY button was pressed
    }

    @Override
    public void loop() {

        //╔╦╗┬─┐┬┬  ┬┬┌┐┌┌─┐
        // ║║├┬┘│└┐┌┘│││││ ┬
        //═╩╝┴└─┴ └┘ ┴┘└┘└─┘

        double y = -gamepad1.right_stick_y;
        double x = gamepad1.right_stick_x * 1.1; // The multiplier is to counteract imperfect strafing.
        double rx = gamepad1.left_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        armRotation.setTargetPosition(armRotationTarget);
        armRotation.setPower(armRotationPower);
        armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armExtension.setTargetPosition(armExtensionTarget);
        armExtension.setPower(armExtensionPower);
        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //-----------------------------------------------------------------




        //Toggle on/off logic for left grabber
        if(gamepad1.a && !leftGrabberButtonCheck)
        {
            if(!isLeftGrabberCurrentlyOpen)
            {
                isLeftGrabberCurrentlyOpen = true;
            }
            else
            {
                isLeftGrabberCurrentlyOpen = false;
            }
            leftGrabberButtonCheck = true;
        }
        else if(!gamepad1.a)
        {
            leftGrabberButtonCheck = false;
        }


        //Toggle on/off logic for right grabber
        if(gamepad1.x && !rightGrabberButtonCheck)
        {
            if(!isRightGrabberCurrentlyOpen)
            {
                isRightGrabberCurrentlyOpen = true;
            }
            else
            {
                isRightGrabberCurrentlyOpen = false;
            }
            rightGrabberButtonCheck = true;
        }
        else if(!gamepad1.x)
        {
            rightGrabberButtonCheck = false;
        }

        //Switch between intake states // ArmPosition 0 means the arm is lowered and ready to pick up a pixel, 1 is the position to place a pixel, and 2 is for the hanging
        if(gamepad1.b && !intakeModeButtonCheck)
        {
            if(ArmPosition == intakePosition)
            {
                ArmPosition = carryingPosition;
            }
            else if(ArmPosition == carryingPosition)
            {
                ArmPosition = placementPosition1;
            }
            else if(ArmPosition == placementPosition1)
            {
                ArmPosition = drivingPosition;
            }
            else if(ArmPosition == drivingPosition)
            {
                ArmPosition = intakePosition;
            }
                intakeModeButtonCheck = true;
            }
            else if(!gamepad1.b)
            {
                intakeModeButtonCheck = false;
            }

            if(ArmPosition == intakePosition)
            {
                armRotationPower = 0.75;
            }
            else if (ArmPosition == drivingPosition)
            {
                armRotationPower = 0.3;
            }
            else if (ArmPosition == 2)
            {
                armRotationPower = 0.3;
            }
            else if (ArmPosition == 3)
            {
                armRotationPower = 0.25;
            }
/*
        if(gamepad1.right_bumper && ArmPosition == 1)
        {
            armExtensionOffset = armExtensionOffset + 5;
        }
        else if(gamepad1.right_trigger > 0.1)
        {
            armExtensionOffset = armExtensionOffset - 5;
        }
        else
        {
            if(ArmPosition == 0)
            {
                armExtensionOffset = 0;
            }
        }
*/
        rotateArm(ArmPosition);

        if (ArmPosition == carryingPosition)
        {
            armExtensionTarget = 0;
        }
        else if(ArmPosition == intakePosition)
        {
            armExtensionTarget = 0;
        }
        else if (ArmPosition == placementPosition1)
        {
            //extendToTargetPosition(armExtensionOffset);
            armExtensionTarget = 3500;
        }
        else if(ArmPosition == drivingPosition)
        {
            armExtensionTarget = 0;
        }
        //-----------------------------------------------------------------


        leftGrabber(isLeftGrabberCurrentlyOpen);
        rightGrabber(isRightGrabberCurrentlyOpen);

        //╔╦╗┌─┐┬  ┌─┐┌┬┐┌─┐┌┬┐┬─┐┬ ┬
        // ║ ├┤ │  ├┤ │││├┤  │ ├┬┘└┬┘
        // ╩ └─┘┴─┘└─┘┴ ┴└─┘ ┴ ┴└─ ┴
        telemetry.addData("debug isLeftGrabberCurrentlyOpen", isLeftGrabberCurrentlyOpen);
        telemetry.addData("debug armRotationTarget", armRotationTarget);
        telemetry.addData("debug intakeMode", ArmPosition);
        telemetry.addData("debug armRotation Encoder Current Position", armRotation.getCurrentPosition());
        telemetry.addData("debug armExtensionTarget", armExtensionTarget);
    }





//╔═╗┬ ┬┌┐┌┌─┐┌┬┐┬┌─┐┌┐┌┌─┐
//╠╣ │ │││││   │ ││ ││││└─┐
//╚  └─┘┘└┘└─┘ ┴ ┴└─┘┘└┘└─┘
    private void launchDrone()
    {
        drone.setPosition(0.5);
    }

    private void leftGrabber(boolean leftGrabberCurrentlyOpen)
    {
        if (leftGrabberCurrentlyOpen)
        {
            grab1.setPosition(grabberClosedPosition);
        }
        else if (!leftGrabberCurrentlyOpen)
        {
            grab1.setPosition(grabberOpenPosition);
        }
    }

    private void rightGrabber(boolean rightGrabberCurrentlyOpen)
    {
        if (rightGrabberCurrentlyOpen)
        {
            grab2.setPosition(grabberClosedPosition);
        }
        else if (!rightGrabberCurrentlyOpen)
        {
            grab2.setPosition(grabberOpenPosition);
        }
    }

    private int rotateArm(int ArmPosition)
    {
        if (ArmPosition == intakePosition)
        {
            wrist.setPosition(wristIntakePosition);
            return armRotationTarget = 15;
        }
        else if (ArmPosition == carryingPosition || ArmPosition == drivingPosition)
        {
            wrist.setPosition(wristIntakePosition);
            return armRotationTarget = -100;
        }
        else if (ArmPosition == placementPosition1)
        {
            wrist.setPosition(wristPlacementPosition);
            return armRotationTarget = -660;
        }
        else //This would be to enter the "hangingPosition."
        {
            hangTime();
            return(armRotationTarget = -300);
        }
    }
/*
    private double extendToTargetPosition(int armExtensionTarget)
    {
        if (armExtension.getCurrentPosition() < armExtensionTarget - 50)
        {
            return(armExtensionPower = 1);
        }
        else if (armExtension.getCurrentPosition() > armExtensionTarget + 50)
        {
            return(armExtensionPower = -1);
        }
        else
        {
            return(armExtensionPower = 0);
            //armExtension.getCurrentPosition();
        }
    }


 */


    private void hangTime() //Macro to hang from the rigging
    {
        ArmPosition = 4;
        armRotationTarget = -300;
        wrist.setPosition(wristPlacementPosition);
    }
}