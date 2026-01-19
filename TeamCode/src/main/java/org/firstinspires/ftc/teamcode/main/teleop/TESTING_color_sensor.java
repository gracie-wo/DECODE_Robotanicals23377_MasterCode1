package org.firstinspires.ftc.teamcode.main.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "color sensor", group = "testing")
public class TESTING_color_sensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        double hue = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) sensor).getLightDetected());
            NormalizedRGBA colors = sensor.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());

            //Determining the amount of red, green, and blue
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);

            //Determining HSV and alpha
            telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
            telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
            telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            telemetry.update();
        }

    }
}