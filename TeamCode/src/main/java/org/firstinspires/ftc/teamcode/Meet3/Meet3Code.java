
package org.firstinspires.ftc.teamcode.Meet3;

import static org.firstinspires.ftc.teamcode.Meet3.utility.StateENUMs.robotMode.drivingPosition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

    private Servo transferWheel = null;
    private Servo transferRotation = null;
    private Servo transferArm = null;
    private Servo transferDoor = null;
    private Servo leftFlipoutIntakeServo = null;
    private Servo rightFlipoutIntakeServo = null;



    private StateENUMs.robotMode activeRobotMode = drivingPosition;



    public void init()
    {
        //telemetry.addData("Current Status", "Robot Has Been Initialized");

    }
    public void init_loop() {
        //Anything here would run after the PLAY button was pressed
    }

    public void loop()
    {

//        if(gamepad1.dpad)
//        {
//
//        }

        switch (activeRobotMode) {
            case drivingPosition:
                break;

            case intakePosition:
                break;
            case boardPosition:
                
                break;
        }
        if(activeRobotMode == drivingPosition)
        {

        }












    }
}
