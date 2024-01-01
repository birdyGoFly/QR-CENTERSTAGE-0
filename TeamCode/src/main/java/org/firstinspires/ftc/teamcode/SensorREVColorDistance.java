package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import java.util.Locale;


import org.firstinspires.ftc.teamcode.utildata.PixelColor;




/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Disabled
@TeleOp(name = "Sensor: REVColorDistance", group = "Sensor")
// Comment this out to add to the opmode list
public class SensorREVColorDistance extends LinearOpMode {


    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     *
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sensor as two separate sensors that share the same name in your op mode.
     *
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     *
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     *
     */
    ColorSensor sensorColor1;
    DistanceSensor sensorDistance1;
    ColorSensor sensorColor2;
    DistanceSensor sensorDistance2;
    private CRServo servo;


    @Override
    public void runOpMode() {
        PixelColor.PixelColors DetectedColor;
/*
       enum PixelColor
       {
           WHITE = 0,
           GREEN = 1,
           PURPLE = 2,
           YELLOW = 3,
       }
*/


        // get a reference to the color sensor.
        sensorColor1 = hardwareMap.get(ColorSensor.class, "Transfer Color Sensor 1");


        // get a reference to the distance sensor that shares the same name.
        sensorDistance1 = hardwareMap.get(DistanceSensor.class, "Transfer Color Sensor 1");


        // get a reference to the color sensor.
        sensorColor2 = hardwareMap.get(ColorSensor.class, "Transfer Color Sensor 2");


        // get a reference to the distance sensor that shares the same name.
        sensorDistance2 = hardwareMap.get(DistanceSensor.class, "Transfer Color Sensor 2");


        servo = hardwareMap.get(CRServo.class, "transfer wheel");


        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};


        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;


        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;


        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        // wait for the start button to be pressed.
        waitForStart();


        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor1.red() * SCALE_FACTOR),
                    (int) (sensorColor1.green() * SCALE_FACTOR),
                    (int) (sensorColor1.blue() * SCALE_FACTOR),
                    hsvValues);


            Color.RGBToHSV((int) (sensorColor2.red() * SCALE_FACTOR),
                    (int) (sensorColor2.green() * SCALE_FACTOR),
                    (int) (sensorColor2.blue() * SCALE_FACTOR),
                    hsvValues);


            // send the info back to driver station using telemetry function.
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance1.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor1.alpha());
            telemetry.addData("Red  ", sensorColor1.red());
            telemetry.addData("Green", sensorColor1.green());
            telemetry.addData("Blue ", sensorColor1.blue());
            telemetry.addData("Hue", hsvValues[0]);


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
            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });
            telemetry.update();
        }
        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
}



