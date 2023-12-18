
package org.firstinspires.ftc.teamcode.Meet3;

import static org.firstinspires.ftc.teamcode.Meet3.utility.StateENUMs.robotMode.boardPosition;
import static org.firstinspires.ftc.teamcode.Meet3.utility.StateENUMs.robotMode.drivingPosition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Meet3.utility.StateENUMs;


@TeleOp(name="Meet 3 TeleOp", group="Iterative OpMode")
public class Meet3Code extends OpMode
{

    //╔╦╗┌─┐┌┬┐┌─┐┬─┐┌─┐
    //║║║│ │ │ │ │├┬┘└─┐
    //╩ ╩└─┘ ┴ └─┘┴└─└─┘
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor leftSliderExtension = null;
    private DcMotor rightSliderExtension = null;
    private DcMotor intakeMotor = null;

    //╔═╗┌─┐┬─┐┬  ┬┌─┐┌─┐
    //╚═╗├┤ ├┬┘└┐┌┘│ │└─┐
    //╚═╝└─┘┴└─ └┘ └─┘└─┘

    private CRServo transferWheel = null;
    private Servo transferRotation = null;
    private CRServo transferArm = null;
    private Servo transferDoor = null;
    private Servo leftFlipoutIntakeServo = null;
    private Servo rightFlipoutIntakeServo = null;

    //SERVO POSITION VARIABLES
    private boolean armToBoardPosition = false;
    private double transferWheelTurnPower = 1; /* maybe change this*/
    private boolean turnTransferWheel = false;



    private double transferArmBoardTarget = 200; /*measure value*/ //this is the variable that measures how much the arm must turn to reach the board
    private double transferArmRestTarget = 0; /*measure value*/ //this is the variable that measures how much the arm must turn to reach resting position
    private double doorOpenPosition = 1;/*measure the value*/
    private double doorClosedPosition = 0;/*change this*//*assuming that this is the starting position*/
    private double transferArmPower = 1;
    private double transferRotationDepositPosition = 1;/*measure the value*/
    private double transferRotationIntakePosition = 0;/*change this*//*assuming that this is the starting position*/
    private double rightStoredIntakePosition = 0.435; /*this is when the right flipout intake is retracted*/
    private double leftStoredIntakePosition = 0.57; /*this is when the left flipout intake is retracted*/
    private double rightIntakePosition = 0.15; /*this is when the right flipout intake is deployed*/
    private double leftIntakePosition = 0.87; /*this is when the left flipout intake is deployed*/



    //MOTOR POSITION VARIABLES
    private int extentionLength = 1000 /*change this value probably*/;
    private double extentionPower = 1;
    private int extentionChange = 1; // how much a bumper trigger increases or decreases the extention length
    private int sliderRest = 0; /* this is the rest. Maybe change*/
    private int intakeMotorPower = 1; /*maybe change this*/


    //Variables to check how many times "A" or "B" has been pressed
    private int numAPress = 0;
    private boolean hasABeenPressed = false;
    private boolean BHasBeenPressed = false;
    private StateENUMs.robotMode activeRobotMode = drivingPosition;


    @Override
    public void init()
    {
        telemetry.addData("Current Status", "Robot Has Been Initialized");

        // WHEEL Motors
        frontLeft = hardwareMap.get(DcMotor.class, "front left");
        frontRight = hardwareMap.get(DcMotor.class, "front right");
        backRight = hardwareMap.get(DcMotor.class, "back right");
        backLeft = hardwareMap.get(DcMotor.class, "back left");

        // SLIDER Motors
        leftSliderExtension = hardwareMap.get(DcMotor.class, "left slider");
        rightSliderExtension = hardwareMap.get(DcMotor.class, "right slider");

        //INTAKE//
        leftFlipoutIntakeServo = hardwareMap.get(Servo.class, "left intake servo");
        rightFlipoutIntakeServo = hardwareMap.get(Servo.class, "right intake servo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake motor");


        //TRANSFER & ARM Servos
        transferWheel = hardwareMap.get(CRServo.class, "transfer wheel"); //The wheel for taking pixels from the intake and holding them for placement on the backdrop
        transferRotation = hardwareMap.get(Servo.class, "transfer rotation");//This is essentially the "wrist" of the robot and it is controls the orientation of the transfer
        transferArm = hardwareMap.get(CRServo.class, "arm rotation"); //This is the rotation for the arm holding the transfer, essentially the "elbow" or "shoulder" of the robot
        transferDoor = hardwareMap.get(Servo.class, "door"); //The door is for dropping pixels out of the transfer




        //get our analog input from the hardwareMap
        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "transferArm");

        // get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
        double position = analogInput.getVoltage() / 3.3 * 360;

        leftSliderExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSliderExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSliderExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSliderExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        transferDoor.setPosition(doorClosedPosition);
//
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSliderExtension.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSliderExtension.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    @Override
    public void init_loop() {
        //Anything here would run after the PLAY button was pressed
    }

    @Override
    public void loop()
    {

        switch (activeRobotMode) {
            case drivingPosition:
                //DRIVING CODE COPIED FROM MEET 2 -------------------------------------------------------
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
                ///////////////////////////////////////////////////////////////////////////////////////////
                if(gamepad1.x) { //if X is pressed X will spit out pixels
                    intakeMotor.setPower(-intakeMotorPower); //turn the intakes the opposite way
                }else if(gamepad1.y){ //if Y is pressed Y will spit out pixels as well as pixels already stored in the robot
                    intakeMotor.setPower(-intakeMotorPower); //turn the intakes the opposite way
                    transferWheel.setPower(-transferWheelTurnPower); //turns the transfer wheel the other way to spit out pixels
                }else if(gamepad1.a) { //if A is pressed  A will intake pixels
                    leftFlipoutIntakeServo.setPosition(rightIntakePosition); //stretch out the intakes
                    rightFlipoutIntakeServo.setPosition(leftIntakePosition); //stretch out the intakes
                    intakeMotor.setPower(intakeMotorPower); //turn the intakes
                    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    transferWheel.setPower(transferWheelTurnPower); // turns the transfer wheel
                }else{
                    leftFlipoutIntakeServo.setPosition(rightStoredIntakePosition); //store the intakes
                    rightFlipoutIntakeServo.setPosition(leftStoredIntakePosition); //store the intakes
                    intakeMotor.setPower(0); //stop turning the intakes
                    transferWheel.setPower(0); // makes sure the transfer wheel isn't turning when nothing is pressed
                    if (gamepad1.b && !BHasBeenPressed) { //if nothing else was pressed, check if b was pressed to switch modes
                        BHasBeenPressed = true;
                        armToBoardPosition = true; //puts the sliders, arm, and transferRotation to the right position
                        activeRobotMode = boardPosition; //switches mode
                        numAPress = 0; //resets how many times A has been pressed just in case
                    } else if(!gamepad1.b) { //makes sure B cannot be held
                        BHasBeenPressed = false;
                    }
                }
                break;
            case boardPosition:
                if(gamepad1.a){ //if A is pressed
                    hasABeenPressed = true;
                    if(numAPress == 0){
                        //first press opens door
                        transferDoor.setPosition(doorOpenPosition);
                    }else if(numAPress == 1){
                        //second press turns wheel
                        transferWheel.setPower(transferWheelTurnPower);
                    }
                }else if(hasABeenPressed){
                    hasABeenPressed = false;
                    numAPress++;
                    //notes that A has been pressed
                }else if(gamepad1.right_bumper){ //if A is not pressed then checks if the bumpers are pressed
                    extentionLength += extentionChange;
                } else if (gamepad1.left_bumper) {

                }else { //if nothing was pressed then checks if B was pressed
                    if (gamepad1.b && !BHasBeenPressed) {
                        transferWheel.setPower(0); // makes sure the transfer wheel stops turning when switching into driving mode
                        BHasBeenPressed = true;
                        armToBoardPosition = false; //puts the sliders, arm, and transferRotation to the right position
                        activeRobotMode = drivingPosition; //switches mode
                        numAPress = 0; //resets how many times A has been pressed
                    } else if(!gamepad1.b){ //makes sure B cannot be held
                        BHasBeenPressed = false;
                    }
                }
                break;
        }
        //Runs the sliders, arm, and transferRotation to the right position
        if(armToBoardPosition){
            leftSliderExtension.setTargetPosition(extentionLength);
            leftSliderExtension.setPower(extentionPower); //maybe swap the signs
            leftSliderExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSliderExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightSliderExtension.setTargetPosition(extentionLength);
            rightSliderExtension.setPower(-extentionPower); //maybe swap the signs
            rightSliderExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSliderExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if(/*position*/ 0 < transferArmBoardTarget) {/*CRSERVO STUFF TO FIX*/
                transferArm.setPower(transferArmPower); //maybe swap the sign
            }
            transferRotation.setPosition(transferRotationDepositPosition);
        }else{
            leftSliderExtension.setTargetPosition(sliderRest); // should be resting position
            leftSliderExtension.setPower(-extentionPower); //maybe swap the signs
            leftSliderExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSliderExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightSliderExtension.setTargetPosition(sliderRest); // should be resting position
            rightSliderExtension.setPower(extentionPower); //maybe swap the signs
            rightSliderExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSliderExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if(/*position > transferArmRestTarget*/0<transferArmRestTarget) {/*CRSERVO STUFF TO FIX*/
                transferArm.setPower(-transferArmPower); //maybe swap the sign
            }
            transferRotation.setPosition(transferRotationIntakePosition); //should be resting position
        }
    }
}
