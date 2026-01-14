package org.firstinspires.ftc.teamcode.main.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Disabled
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

            telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));


            if(hue < 350 && hue > 225){
                telemetry.addData("Color Detected:", "Purple");
                telemetry.update();
            }

            if(hue > 90 && hue < 225){
                telemetry.addData("Color Detected:", "Green");
                telemetry.update();
            }
            telemetry.update();
        }

    }
}