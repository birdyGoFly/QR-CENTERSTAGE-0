/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="My First TeleOp WITH ARM", group="Iterative OpMode")

public class MyFirstTeleOpWITHARM extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private DcMotor armExtension = null;
    private DcMotor armRotation = null;
    private Servo gate = null;
    private Servo drone = null;
    private Servo wrist = null;
    private Servo grab1 = null;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower = 0;
    double rightPower = 0;

    boolean buttonCheck = false;
    boolean open = false;

    int toDegrees = 4;

    double linAcRotationPower = 0.25;
    double linAcPower = 1;

    boolean isLinAcActivated = false;

    int armRotationTarget = 0;

    boolean intakeMode = false;

    //boolean hangMode = false;   I made a change here (I commented it out)

    boolean hangCheck = false;
    boolean launch = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.get(DcMotor.class, "front left");
        frontRight = hardwareMap.get(DcMotor.class, "front right");
        backRight = hardwareMap.get(DcMotor.class, "back right");
        backLeft = hardwareMap.get(DcMotor.class, "back left");
        armExtension = hardwareMap.get(DcMotor.class, "linear actuator");
        armRotation = hardwareMap.get(DcMotor.class, "linear actuator rotation");
        gate = hardwareMap.get(Servo.class, "gate servo");
        drone = hardwareMap.get(Servo.class, "drone servo");
        wrist = hardwareMap.get(Servo.class, "wrist servo");
        grab1 = hardwareMap.get(Servo.class, "left grabber servo");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);


        armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRotation.setPower(0.25);

        armExtension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtension.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //linAc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //linAc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        grab1.setPosition(0);
        drone.setPosition(0);
        drone.setPosition(0.25);
        moveGate(open);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {



/*
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
//        double drive = -gamepad1.left_stick_y;
//        double turn  =  gamepad1.right_stick_x;
//        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
//        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
         leftPower  = -gamepad1.left_stick_y ;
         rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        frontLeft.setPower(leftPower);
        backLeft.setPower(leftPower);
        frontRight.setPower(rightPower);
        backRight.setPower(rightPower);

        */


        double y = -gamepad1.right_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.right_stick_x * 1.1; // Counteract imperfect strafing
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

        //-------------------------------------- GATE STUFF

        if (gamepad1.a && !buttonCheck) {
            buttonCheck = true;
        }

        if (!gamepad1.a && buttonCheck) {
            buttonCheck = false;
            open = !open;
        }

        moveGate(open);
        //-------------------------------------- DRONE OPERATION

        if(gamepad1.x){
            launch = true;
            drone.setPosition(0.5);
        }

        //-------------------------------------- ARM OPERATION
/*
        if (gamepad1.right_bumper) {
            linAcRotation.setPower(linAcRotationPower);
            isLinAcActivated = true;
        }
        if (gamepad1.right_trigger > 0.05) {
            linAcRotation.setPower(-linAcRotationPower * gamepad1.right_trigger);
            isLinAcActivated = true;
        }
        if (!gamepad1.right_bumper && gamepad1.right_trigger < 0.05 && isLinAcActivated) {
            linAcRotation.setPower(0);
        }


 */

        if (gamepad1.left_bumper) {
            armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armExtension.setPower(linAcPower);
        }
        else if (gamepad1.left_trigger > 0.05) {
            armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armExtension.setPower(-linAcPower);
        }
        //else if (!gamepad1.left_bumper && gamepad1.left_trigger < 0.05 && !hangCheck) {
          //  armExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
          //  armExtension.setPower(0);
        //}

        if(gamepad1.a)
        {
            intakeMode = false;

            grab1.setPosition(0.45);

            armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRotation.setPower(0.5);
            armRotation.setTargetPosition(500);
        }
        else
        {
            intakeMode = true;
        }
/*
        if(gamepad1.b)
        {
            wrist.setPosition(0);
        }
        else
        {
            wrist.setPosition(0.45);
        }

 */




        if (gamepad1.y) {
            intakeMode = false;
            hangMode(true);
        }
        if (!gamepad1.y) {
            hangMode(false);
        }

        if(gamepad1.right_trigger >= 0.1){
            armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRotationTarget = armRotationTarget + 3;
            armRotation.setPower(.3);
        }
        else if(gamepad1.right_bumper){
            armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRotationTarget = armRotationTarget + 3;
            armRotation.setPower(.3);
        }



        //--------------------------------------
/*
        if (!isLinAcActivated) {
            linAcRotation.setPower(0.1);
        }
*/

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("linAcRotation", "%7d", armRotation.getCurrentPosition()/toDegrees/*I made a change right here*/);
        telemetry.addData("linAcRotation Encoder", "%7d", armRotation.getCurrentPosition());
        telemetry.addData("linAcRotation Target", "%7d", armRotationTarget);
        telemetry.addData("linAcExtension", "%7d", armExtension.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void moveGate(boolean isGateOpen) {
        if (isGateOpen) {
            gate.setPosition(1);
        } else {
            gate.setPosition(0.75);
        }

    }

    public void intakePosition(boolean intakeMode) {
        if (intakeMode) {
            //grab1.setPosition(0);

            armRotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armRotation.setTargetPosition(0);
            armRotation.setPower(1);

            wrist.setPosition(0);

        } else {
            //linAcRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void hangMode(boolean hangTime) {
        if (hangTime) {
            //linAc.setTargetPosition(9515);
            armRotationTarget = 360;
            armRotation.setPower(0.75);
            armExtension.setPower(1);
            armExtension.setTargetPosition(9000);
            hangCheck = true;
            //if(linAc.getCurrentPosition()>=){

            }
        else {
            hangCheck = false;
        }
            if(gamepad1.right_bumper)
            armRotation.setPower(0.25);
            armRotationTarget = 0;
            armExtension.setTargetPosition(0);
            hangCheck = false;

    }
}


/*
//            linAcRotation.setPower(0.75;
            //linAcRotation.setTargetPosition(-90*toDegrees);
            linAc.setPower(1);
            //linAc.setTargetPosition(9430);
            //linAc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //linAcRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/
