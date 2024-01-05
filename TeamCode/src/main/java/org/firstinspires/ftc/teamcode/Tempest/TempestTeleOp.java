
package org.firstinspires.ftc.teamcode.Tempest;

import static org.firstinspires.ftc.teamcode.Tempest.utility.StateENUMs.robotMode.boardPosition;
import static org.firstinspires.ftc.teamcode.Tempest.utility.StateENUMs.robotMode.drivingPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Tempest.utility.StateENUMs;
import org.firstinspires.ftc.teamcode.utildata.PixelColor;
//import org.firstinspires.ftc.teamcode.Tempest.utility.IntakeController;


@TeleOp(name="Tempest TeleOp", group="Iterative OpMode")
public class TempestTeleOp extends OpMode
{

    //╔╦╗┌─┐┌┬┐┌─┐┬─┐┌─┐
    //║║║│ │ │ │ │├┬┘└─┐
    //╩ ╩└─┘ ┴ └─┘┴└─└─┘
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotorEx leftSliderExtension = null;
    private DcMotorEx rightSliderExtension = null;
    private DcMotor intakeMotor = null;

    //╔═╗┌─┐┬─┐┬  ┬┌─┐┌─┐
    //╚═╗├┤ ├┬┘└┐┌┘│ │└─┐
    //╚═╝└─┘┴└─ └┘ └─┘└─┘
//
    private CRServo transferWheel = null;
    private Servo transferRotation = null;
    private Servo transferArm = null;
    private Servo transferDoor = null;
    private Servo leftFlipoutIntakeServo = null;
    private Servo rightFlipoutIntakeServo = null;

    //SERVO POSITION VARIABLES//
    private boolean armToBoardPosition = false;
    private double transferWheelTurnPower = 1; /* maybe change this*/
    private boolean turnTransferWheel = false;

    //SENSORS
    /** Transfer color sensor setup*/
    ColorSensor sensorColor1;
    DistanceSensor sensorDistance1;
    ColorSensor sensorColor2;
    DistanceSensor sensorDistance2;

    //hsvValues is an array that will hold the hue, saturation, and value information for the transfer color sensors.
    float hsvValues[] = {0F, 0F, 0F};

    //Values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;
    PixelColor.PixelColors DetectedColor;
    //----------------------------------------------------------------------------------------------
    //private IntakeController intakeController = new IntakeController();
    private double intakeTargetPosition = 0;
    private int intakeMaxPosition = 100;
    private double transferArmBoardTarget = 0.7; /*measure value*/ //Arm rotation target for pixel placement
    private double transferArmRestTarget = 0; /*measure value*/ //Arm rotation target when stowing transfer within the robot
    private double transferArmRotationTarget = 0;
    private double transferArmRotationSpeed = 0.02;
    private double doorOpenPosition = 0.2;/*measure the value*/
    private double doorClosedPosition = 0.42;/*change this*//*assuming that this is the starting position*/
    private double transferArmPower = 1;
    private double transferRotationDepositPosition = 0.435;/*measure the value*/
    private double transferRotationRestPosition = 0;/*change this to 0.965 if you want it to be angled *//*assuming that this is the starting position*/
    private double transferRotationIntakePosition = 0;/*change this*//*assuming that this is the intake position*/
    private double transferRotationTarget = 0;
    private double transferRotationSpeed = 0.01;
    private double rightIntakeMIN = 0.435; /*this is when the right flipout intake is retracted*/
    private double leftIntakeMIN = 0.605; /*this is when the left flipout intake is retracted*/
    private double rightIntakeMAX = 0.15; /*this is when the right flipout intake is deployed*/
    private double leftIntakeMAX = 0.87; /*this is when the left flipout intake is deployed*/


    //MOTOR POSITION VARIABLES
    private int extensionLength = 0 /*change this value probably*/;
    private double extensionPower = 1;
    private int extensionChange = 25; // how much a bumper trigger increases or decreases the extention length
    private int sliderRest = 0; /* this is the retracted position. Maybe change*/
    private int intakeMotorPower = 1;

    private int sliderFirstRow = 0;
    ////private double armFirstRow =





    //Variables to check how many times "A" or "B" has been pressed
    private int numAPress = 0;
    private boolean isAPressed = false;
    private boolean BHasBeenPressed = false;
    private StateENUMs.robotMode activeRobotMode = drivingPosition;

    //DEBUG SLIDER SYNC CODE
    int sliderTarget = 0;

    double Kp1 = 1;
    double Kp2 = 2;

    int synchronizationKillswitchThreshold = 60;
    boolean autoKillswitchEnabled = false;
    double orientationAdjustmentSensitivity = 0.25; //TODO: TUNE THIS VALUE
    double armPosition = 0;
    int motorDirection = 1;
    boolean buttonCheck = false;
    double intakeTargetPercentage = 0;
    double powerMultiplier = 1;


    @Override
    public void init()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("Current Status", "Robot Has Been Initialized");

        // WHEEL Motors
        frontLeft = hardwareMap.get(DcMotor.class, "front left");
        frontRight = hardwareMap.get(DcMotor.class, "front right");
        backRight = hardwareMap.get(DcMotor.class, "back right");
        backLeft = hardwareMap.get(DcMotor.class, "back left");

        // SLIDER Motors
        leftSliderExtension = hardwareMap.get(DcMotorEx.class, "left slider");
        rightSliderExtension = hardwareMap.get(DcMotorEx.class, "right slider");

        //INTAKE//
        leftFlipoutIntakeServo = hardwareMap.get(Servo.class, "left intake servo");
        rightFlipoutIntakeServo = hardwareMap.get(Servo.class, "right intake servo");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake motor");


        //TRANSFER & ARM Servos
        transferWheel = hardwareMap.get(CRServo.class, "transfer wheel"); //The wheel for taking pixels from the intake and holding them for placement on the backdrop
        transferRotation = hardwareMap.get(Servo.class, "transfer rotation");//This is essentially the "wrist" of the robot and it is controls the orientation of the transfer
        transferArm = hardwareMap.get(Servo.class, "arm rotation"); //This is the rotation for the arm holding the transfer, essentially the "elbow" or "shoulder" of the robot
        transferDoor = hardwareMap.get(Servo.class, "door"); //The door is for dropping pixels out of the transfer
/*
        //Get a reference to the color sensor.
        sensorColor1 = hardwareMap.get(ColorSensor.class, "Transfer Color Sensor 1");
        //Get a reference to the distance sensor that shares the same name.
        sensorDistance1 = hardwareMap.get(DistanceSensor.class, "Transfer Color Sensor 1");
        //Get a reference to the other color sensor.
        sensorColor2 = hardwareMap.get(ColorSensor.class, "Transfer Color Sensor 2");
        //Get a reference to the other distance sensor that shares the same name.
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "Transfer Color Sensor 2");


 */


        //-------------------------------------------------

        leftSliderExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSliderExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSliderExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSliderExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        transferDoor.setPosition(doorClosedPosition);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSliderExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSliderExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSliderExtension.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSliderExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSliderExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSliderExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    @Override
    public void init_loop() {
        //Anything here would run after the PLAY button was pressed
    }

    @Override
    public void loop()
    {
        telemetry.setMsTransmissionInterval(100);
        telemetry.addData("Left Slider Position", leftSliderExtension.getCurrentPosition());
        telemetry.addData("Right Slider Position", rightSliderExtension.getCurrentPosition());
        telemetry.addData("Left Slider Error", sliderTarget-leftSliderExtension.getCurrentPosition());
        telemetry.addData("Right Slider Error", sliderTarget-rightSliderExtension.getCurrentPosition());
        telemetry.addData("Left Slider isBusy", leftSliderExtension.isBusy());
        telemetry.addData("Right Slider isBusy", rightSliderExtension.isBusy());
        telemetry.addData("Left Slider Velocity", leftSliderExtension.getVelocity());
        telemetry.addData("Right Slider Velocity", rightSliderExtension.getVelocity());
        telemetry.addData("Slider Synchronization Error", Math.abs(Math.abs(leftSliderExtension.getCurrentPosition())-Math.abs(rightSliderExtension.getCurrentPosition())));
        telemetry.addData("Slider Killswitch Enabled", autoKillswitchEnabled);
        telemetry.addData("Arm Encoder Rotation", armPosition);
        telemetry.addData("Transfer Arm Rotation Target", transferArmRotationTarget);
        telemetry.addData("Transfer Rotation Target", transferRotationTarget);
        telemetry.update();

        //get our analog input from the hardwareMap
        AnalogInput analogInput = hardwareMap.get(AnalogInput.class, "transferArm");

        // get the voltage of our analog line
        // divide by 3.3 (the max voltage) to get a value between 0 and 1
        // multiply by 360 to convert it to 0 to 360 degrees
        double armPosition = analogInput.getVoltage() / 3.3 * 360;

        /* This sets the power to vary depending on the error. Does not work as of 12/24/23 (happy holidays)*/

       // double power1 = Kp1 * (leftSliderExtension.getTargetPosition() - leftSliderExtension.getCurrentPosition());
       // double power2 = Kp1 * (rightSliderExtension.getTargetPosition() - rightSliderExtension.getCurrentPosition());

        //leftSliderExtension.setPower(power1);
        //rightSliderExtension.setPower(power2);

        transferArm.setPosition(transferArmRotationTarget);
        transferRotation.setPosition(transferRotationTarget);

        if (transferArmRotationTarget < 0)
        {
            transferArmRotationTarget = 0;
        }
        if (transferRotationTarget < 0)
        {
            transferRotationTarget = 0;
        }

        leftSliderExtension.setTargetPosition(sliderTarget);
        rightSliderExtension.setTargetPosition(sliderTarget);

        leftSliderExtension.setPower(extensionPower);
        rightSliderExtension.setPower(extensionPower);

        sliderAutoSafetyKillswitch(leftSliderExtension.getCurrentPosition(), rightSliderExtension.getCurrentPosition(), synchronizationKillswitchThreshold);

        if(intakeTargetPercentage < 0)
        {
            intakeTargetPercentage = 0;
        }
        else if(intakeTargetPercentage > 100)
        {
            intakeTargetPercentage = 100;
        }
/*
        if(gamepad1.dpad_up)
        {
            intakeTargetPercentage += 1;
        }
        else if(gamepad1.dpad_down)
        {
            intakeTargetPercentage -= 1;
        }

        if(gamepad1.dpad_right)
        {
            intakeTargetPercentage = 0;
        }
*/
        //intakePercentageTarget(intakeTargetPercentage);

        rightFlipoutIntakeServo.setPosition(((0.87-0.605)*(intakeTargetPercentage/100))+0.605);
        leftFlipoutIntakeServo.setPosition(((0.15-0.435)*(intakeTargetPercentage/100))+0.435);
        telemetry.addData("target", ((0.15-0.435)*(intakeTargetPercentage/100))+0.435);



        // convert the RGB values to HSV values.
        // multiply by the SCALE_FACTOR.
        // then cast it back to int (SCALE_FACTOR is a double)
        //This is for sensor 1
        /*
        Color.RGBToHSV((int) (sensorColor1.red() * SCALE_FACTOR),
                (int) (sensorColor1.green() * SCALE_FACTOR),
                (int) (sensorColor1.blue() * SCALE_FACTOR),
                hsvValues);
        //This is for sensor 2
        Color.RGBToHSV((int) (sensorColor2.red() * SCALE_FACTOR),
                (int) (sensorColor2.green() * SCALE_FACTOR),
                (int) (sensorColor2.blue() * SCALE_FACTOR),
                hsvValues);
        //Display the name of the pixel color on the telemetry
        sensor1PixelDetection();
        sensor2PixelDetection();


         */
        // change the background color to match the color detected by the RGB sensor.
        // pass a reference to the hue, saturation, and value array as an argument
        // to the HSVToColor method.
        /*
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });


         */

        /*DRIVING CODE COPIED FROM MEET 2*/// -------------------------------------------------------
        double y = (-gamepad1.right_stick_y * motorDirection) * powerMultiplier;
        double x = ((gamepad1.right_stick_x * 1.1) * motorDirection) * powerMultiplier; // The multiplier is to counteract imperfect strafing.
        double rx = (gamepad1.left_stick_x) * powerMultiplier;

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

        if (gamepad1.dpad_left && !buttonCheck)
        {
            buttonCheck = true;
        }

        if (!gamepad1.dpad_left && buttonCheck)
        {
            buttonCheck = false;
            motorDirection *= -1;
        }

        ///////////////////////////////////////////////////////////////////////////////////////////

        switch (activeRobotMode) {
            case drivingPosition:
                powerMultiplier = 1;
                /*DRIVING CODE COPIED FROM MEET 2*/// -------------------------------------------------------
                /*
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

                 */

/*
                setIntakePosition(intakeTargetPosition);

                if(intakeTargetPosition < 0)
                {
                    intakeTargetPosition = 0;
                }
                else if(intakeTargetPosition > 100)
                {
                    intakeTargetPosition = 100;
                }

                if(gamepad1.dpad_up)
                {
                    intakeTargetPosition += 0.001;
                }
                else if(gamepad1.dpad_down)
                {
                    intakeTargetPosition -= 0.001;
                }

                if(gamepad1.dpad_right)
                {
                    intakeTargetPosition = 0;
                }

 */


                ///////////////////////////////////////////////////////////////////////////////////////////
                // FLOOR-MODE TRANSFER CONTROL | Exclusively use this mode if there are arm issues and you need to be a pushbot
                if(gamepad1.x) { //If X is pressed the intake will spit out pixels (spin in reverse). The transfer wheel will not spin
                    intakeMotor.setPower(-intakeMotorPower); //Rotate the intake in reverse
                }else if(gamepad1.y){ //If Y is pressed the robot will spit out pixels as well as pixels already stored in the transfer wheel, by rotating both the wheel and intake in reverse
                    intakeMotor.setPower(-intakeMotorPower); //Spin the intake in reverse
                    transferWheel.setPower(-transferWheelTurnPower); //Turns the transfer wheel in reverse to spit out pixels
                    //(transferRotationRestPosition);
                }else if(gamepad1.a) { //if A is pressed  A will intake pixels
                    intakeTargetPercentage = 100; //deploy the intakes
                    intakeMotor.setPower(intakeMotorPower); //turn the intakes
                    intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    transferWheel.setPower(transferWheelTurnPower); // turns the transfer wheel
                    //transferRotation.setPosition(transferRotationIntakePosition);
                }else{
                    intakeTargetPercentage = 0; //store the intakes
                    intakeMotor.setPower(0); //Stop turning the intake motor
                    transferWheel.setPower(0); //Keeps the transfer wheel from turning when nothing is pressed
                    //transferRotation.setPosition(transferRotationIntakePosition);
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
                //----------------------------------------------------------------------------------
                //This is only for when the robot is in board (or placement) position ----------
            case boardPosition:
                powerMultiplier = 0.5;
                if(gamepad1.a){ //Check if A is pressed for placing 1st and 2nd pixel
                    isAPressed = true; //Register that A has been pressed
                    if(numAPress == 0){
                        //first press opens door
                        transferDoor.setPosition(doorOpenPosition);
                    }else if(numAPress == 1){
                        //second press turns wheel
                        transferWheel.setPower(transferWheelTurnPower);
                    }
                }else if(isAPressed){ //When A is released, register that as a numAPress
                    isAPressed = false;
                    numAPress++;
                    //notes that A has been pressed
                }else if(gamepad1.right_bumper){ //if A is not pressed then checks if the bumpers are pressed
                    extensionLength += extensionChange;
                } else if (gamepad1.left_bumper) {
                    extensionLength -= extensionChange;
                }else { //if nothing was pressed then checks if B was pressed
                    if (gamepad1.b && !BHasBeenPressed) {
                        transferWheel.setPower(0);
                        transferDoor.setPosition(doorClosedPosition);
                        // makes sure the transfer wheel stops
                        // turning when switching into driving mode
                        BHasBeenPressed = true;
                        armToBoardPosition = false;
                        //Puts the sliders, arm, and transferRotation to the right position
                        activeRobotMode = drivingPosition; //switches mode
                        numAPress = 0; //resets how many times A has been pressed
                    } else if(!gamepad1.b){ //makes sure B cannot be held
                        BHasBeenPressed = false;
                    }
                }
                break;
        }
        //Runs the sliders, arm, and transferRotation to the right position for pixel placement
        if(armToBoardPosition){
            sliderTarget = extensionLength;
            //transferRotation.setPosition(transferRotationDepositPosition); /*COMMENTED OUT FOR DEBUGGING, very jittery, assumed to be related to conflicting commands*/
            if (transferArmRotationTarget <= 0.4)
            {
                transferArmRotationTarget += transferArmRotationSpeed * 2;
            }
            else if (transferArmRotationTarget > 0.4 && transferArmRotationTarget < transferArmBoardTarget)
            {
                transferArmRotationTarget += transferArmRotationSpeed * 0.75;
            }
            if (transferRotationTarget < transferRotationDepositPosition && transferArmRotationTarget <= transferArmBoardTarget)
            {
                transferRotationTarget += transferRotationSpeed * (transferArmRotationTarget * 4);
            }
            else if (transferArmRotationTarget >= transferArmBoardTarget)
            {
                transferRotationTarget = transferRotationDepositPosition;
            }//

        }
        else
        {
            sliderTarget = sliderRest;
            if (transferArmRotationTarget > 0.5)
            {
                transferArmRotationTarget -= transferArmRotationSpeed * 3;
            }
            else if (transferArmRotationTarget <= 0.5 && transferArmRotationTarget > transferArmRestTarget)
            {
                transferArmRotationTarget -= transferArmRotationSpeed;
            }
            if (transferRotationTarget > transferRotationRestPosition)
            {
                transferRotationTarget -= transferRotationSpeed * 4;
            }
        }
    }

    //FUNCTIONS//
/*
    public void setIntakePosition(double percentage)
    {
        //Make sure percentage is within the valid range
        percentage = Math.max(0, Math.min(100,percentage));

        //Set servo positions
        rightFlipoutIntakeServo.setPosition(rightStoredIntakePosition + (rightIntakePosition - rightStoredIntakePosition) * (percentage / 100));
        leftFlipoutIntakeServo.setPosition(leftStoredIntakePosition + (leftIntakePosition - leftStoredIntakePosition) * (percentage / 100));

        telemetry.addData("Right Intake Target", (rightStoredIntakePosition + (rightIntakePosition - rightStoredIntakePosition) * (percentage / 100)));
        telemetry.addData("Left Intake Target", (leftStoredIntakePosition + (leftIntakePosition - leftStoredIntakePosition) * (percentage / 100)));

        telemetry.addData("Servo Target", intakeTargetPosition);
    }
    */
    void sliderAutoSafetyKillswitch(int leftSliderPosition, int rightSliderPosition, int syncKillswitchThreshold) //Kill power to both sliders to prevent the arm from ripping itself apart
    {
        if (Math.abs(Math.abs(leftSliderPosition)-Math.abs(rightSliderPosition)) > syncKillswitchThreshold)
        {
            leftSliderExtension.setMotorDisable();
            rightSliderExtension.setMotorDisable();
            boolean autoKillswitchEnabled = true;
        }
        if (autoKillswitchEnabled = true && gamepad1.right_bumper)
        {
            leftSliderExtension.setMotorEnable();
            rightSliderExtension.setMotorEnable();
        }
    }

    void autoBackdropOrientation(double leftDistanceSensor, double rightDistanceSensor, double rotationSensitivity)
    {
      //double orientationError = getOrientationError(/*left sensor*/, /*right sensor*/);
      double leftWheels = 0;
      double rightWheels = 0;

      //TODO: Incorporate the adjusted orientation into the drive wheels

    }

    double getOrientationError(double leftDistanceSensor, double rightDistanceSensor)
    {
        return leftDistanceSensor - rightDistanceSensor;
    }

    void sensor1PixelDetection()
    {
        if(sensorDistance1.getDistance(DistanceUnit.CM) < 4 && sensorColor1.blue() > ((sensorColor1.red() + sensorColor1.green()) / 2))
        {
            DetectedColor = PixelColor.PixelColors.PURPLE;
            telemetry.addData("Pixel 1 Color", DetectedColor);
        }
        else if(sensorDistance1.getDistance(DistanceUnit.CM) < 4 && sensorColor1.green() > sensorColor1.red() && sensorColor1.red() > sensorColor1.blue())
        {
            DetectedColor = PixelColor.PixelColors.YELLOW;
            telemetry.addData("Pixel 1 Color", DetectedColor);
        }
        else if(sensorDistance1.getDistance(DistanceUnit.CM) < 4 && sensorColor1.green() > ((sensorColor1.red() + sensorColor1.blue()) / 2))
        {
            DetectedColor = PixelColor.PixelColors.GREEN;
            telemetry.addData("Pixel 1 Color", DetectedColor);
        }
        else if(sensorDistance1.getDistance(DistanceUnit.CM) >= 4)
        {
            telemetry.addData("Pixel 1 Color", "NONE");
        }
    }

    void sensor2PixelDetection()
    {
        if(sensorDistance2.getDistance(DistanceUnit.CM) < 4 && sensorColor2.blue() > ((sensorColor2.red() + sensorColor2.green()) / 2))
        {
            DetectedColor = PixelColor.PixelColors.PURPLE;
            telemetry.addData("Pixel 2 Color", DetectedColor);
        }
        else if(sensorDistance2.getDistance(DistanceUnit.CM) < 4 && sensorColor2.green() > sensorColor2.red() && sensorColor2.red() > sensorColor2.blue())
        {
            DetectedColor = PixelColor.PixelColors.YELLOW;
            telemetry.addData("Pixel 2 Color", DetectedColor);
        }
        else if(sensorDistance2.getDistance(DistanceUnit.CM) < 4 && sensorColor2.green() > ((sensorColor2.red() + sensorColor2.blue()) / 2))
        {
            DetectedColor = PixelColor.PixelColors.GREEN;
            telemetry.addData("Pixel 2 Color", DetectedColor);
        }
        else if(sensorDistance2.getDistance(DistanceUnit.CM) >= 4)
        {
            telemetry.addData("Pixel 2 Color", "NONE");
        }
    }
/*
    void intakePercentageTarget(double targetPercentage)
    {
        telemetry.addData("targetPercentage", targetPercentage);
        telemetry.addData("target", ((0.15-0.435)*(targetPercentage/100))+0.435);

    }
*/
}
