package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.ColorSensor;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

        import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Disabled
@TeleOp
public class ColorSensorTest extends LinearOpMode {
    // Define a variable for our color sensor
    ColorSensor color;

    RevBlinkinLedDriver blinkin;




    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        color = hardwareMap.get(ColorSensor.class, "color sensor");

        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "led");


        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();

            if((color.red() > 700 /* && color.red() < 900*/) && (color.green() > 1000 && color.green() < 1600) && (color.blue() > 2000 && color.blue() < 3000))
            {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
            } else if (false) {

            }
            else
            {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);
            }


        }
    }
}
