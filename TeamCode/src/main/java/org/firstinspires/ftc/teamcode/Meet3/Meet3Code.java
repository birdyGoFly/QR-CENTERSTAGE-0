
package org.firstinspires.ftc.teamcode.Meet3;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.Meet3.utility.StateENUMs.robotMode.boardPosition;
import static org.firstinspires.ftc.teamcode.Meet3.utility.StateENUMs.robotMode.drivingPosition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Meet3.utility.StateENUMs;
import org.firstinspires.ftc.teamcode.utildata.ArmPositionENUM;



public class Meet3Code {

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
    private Servo transferArm = null;
    private Servo transferDoor = null;
    private Servo leftFlipoutIntakeServo = null;
    private Servo rightFlipoutIntakeServo = null;

    //SERVO POSITION VARIABLES
    private boolean armToBoardPosition = false;
    private double turnPower = 1; /* maybe change this*/
    private boolean turnTransferWheel = false;

    private int doorOpenPosition = 123;/*mesure the value*/
    private int doorClosedPosition = 123;/*change this*//*assuming that this is the starting position*/
    private int transferArmDepositPosition = 123123123;/*mesure the value*/
    private int transferArmIntakePosition = 0;/*change this*//*assuming that this is the starting position*/
    private int transferRotationDepositPosition = 123123123;/*mesure the value*/
    private int transferRotationIntakePosition = 0;/*change this*//*assuming that this is the starting position*/

    //MOTOR POSITION VARIABLES
    private int extentionLength = 25 /*change this value probably*/;
    private double extentionPower = 1;
    private int extentionChange = 1; // how much a bumper trigger increases or decreases the extention length
    private int sliderRest = 0; /* this is the rest. Maybe change*/


    //VARIABLES I USE TO SEE HOW MANY TIMES "A" or "B" HAS BEEN PRESSED XD
    private int numAPress = 0;
    private boolean hasABeenPressed = false;
    private boolean BHasBeenPressed = false;
    private StateENUMs.robotMode activeRobotMode = drivingPosition;



    public void init()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSliderExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSliderExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSliderExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSliderExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //telemetry.addData("Current Status", "Robot Has Been Initialized");

        transferDoor.setPosition(doorClosedPosition);

    }
    public void init_loop() {
        //Anything here would run after the PLAY button was pressed
    }

    public void loop()
    {

        switch (activeRobotMode) {
            case drivingPosition:

                break;
            case boardPosition:
                if(gamepad1.a){ //if A is pressed
                    hasABeenPressed = true;
                    if(numAPress == 0){
                        //first press opens door
                        transferDoor.setPosition(doorOpenPosition);
                    }else if(numAPress == 1){
                        //second press turns wheel
                        transferWheel.setPower(turnPower);
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
                        transferWheel.setPower(turnPower); // makes sure the transfer wheel starts turning when switching into driving mode
                        BHasBeenPressed = true;
                        armToBoardPosition = false; //puts the sliders, arm, and transferRotation to the right position
                        activeRobotMode = drivingPosition; //switches mode
                    } else { //makes sure B cannot be held
                        BHasBeenPressed = false;
                    }
                }
                break;
        }
        //Runs the sliders, arm, and transferRotation to the right position
        if(armToBoardPosition){
            leftSliderExtension.setTargetPosition(extentionLength);
            leftSliderExtension.setPower(extentionPower);
            leftSliderExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSliderExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightSliderExtension.setTargetPosition(extentionLength);
            rightSliderExtension.setPower(extentionPower);
            rightSliderExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSliderExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            transferArm.setPosition(transferArmDepositPosition);
            transferRotation.setPosition(transferRotationDepositPosition);
        }else{
            leftSliderExtension.setTargetPosition(sliderRest); // should be resting position
            leftSliderExtension.setPower(extentionPower);
            leftSliderExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftSliderExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightSliderExtension.setTargetPosition(sliderRest); // should be resting position
            rightSliderExtension.setPower(extentionPower);
            rightSliderExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightSliderExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            transferArm.setPosition(transferArmIntakePosition); //should be resting position
            transferRotation.setPosition(transferRotationIntakePosition); //should be resting position
        }












    }
}
