package org.firstinspires.ftc.teamcode.main.disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Disabled
@TeleOp(name = "touch sensors", group = "Testing")
public class DISABLED_TESTING_Touch_sensors extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        TouchSensor tsensor_row_1 = hardwareMap.get(TouchSensor.class, "row1");
        TouchSensor tsensor_row_2 = hardwareMap.get(TouchSensor.class, "row2");

        DcMotor launcher = hardwareMap.dcMotor.get("launch");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();



        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean touch_row_1 = false;
            boolean touch_row_2 = true;

            if(tsensor_row_1.isPressed()){
                touch_row_1 = true;
            }

            if(tsensor_row_2.isPressed()){
                touch_row_2 = true;
            }

            if(gamepad1.a){
                launcher.setPower(1);
                touch_row_2 = false;
                touch_row_1 = false;
            }
        }


    }
}