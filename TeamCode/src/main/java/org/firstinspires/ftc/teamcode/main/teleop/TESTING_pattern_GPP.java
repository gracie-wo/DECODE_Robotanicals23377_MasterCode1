package org.firstinspires.ftc.teamcode.main.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import java.util.List;

@TeleOp(name = "GPP Color Sensor", group = "testing")
public class TESTING_pattern_GPP extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
//
//        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(0);
//        LLResult llResult = limelight.getLatestResult();
//        limelight.start();

//        DcMotor intake = hardwareMap.dcMotor.get("intake");
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo spindex = hardwareMap.get(Servo.class, "spindex");
        NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        double hue = 0.0;
        String color_detected = "None";

//        DcMotor launcher = hardwareMap.dcMotor.get("launcher");
//        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime timer = new ElapsedTime();


        boolean adjusted = false;
        int ballPickUp = 1;
        boolean onetwothreeShoot = false;
        boolean threetwooneShoot = false;
        boolean twothreeoneShoot = false;
        boolean detected = false;
        boolean sensing = false;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());
//            mosaic(llResult, limelight);

            if(hue < 350 && hue > 225){
                color_detected = "Purple";
//                telemetry.addData("Color Detected:", "Purple");
//                telemetry.update();
                detected = true;
            } else if(hue > 90 && hue < 225){
                color_detected = "Green";
//                telemetry.addData("Color Detected:", "Green");
//                telemetry.update();
                detected = true;
            } else {
                color_detected = "None";
//                telemetry.addData("Color Detected:", "None");
//                telemetry.update();
                detected = false;
            }

            if(gamepad1.y){
                spindex.setPosition(1);
//                intake.setPower();
                ballPickUp = 1;
                onetwothreeShoot = false;
                threetwooneShoot = false;
                twothreeoneShoot = false;
                adjusted = false;
                sensing = true;
            }

            if(sensing) {
                //green ball first
                if (color_detected.equals("Green") && ballPickUp == 1 && !adjusted) {
                    onetwothreeShoot = true;
                    adjusted = true;
                    spindex.setPosition(0.56);
                    ballPickUp = 2;
                    timer.reset();
                } else if (adjusted && ballPickUp == 2 && detected && (timer.time() > 0.3)) {
                    spindex.setPosition(0.1);
                    ballPickUp = 3;
                    timer.reset();
                } else if (ballPickUp == 3 && detected && (timer.time() > 0.3)) {
                    //intake.setPower(-1)
                    if (onetwothreeShoot) {
                        spindex.setPosition(0);
                    } else if (threetwooneShoot) {
                        spindex.setPosition(0.87);
                    } else if (twothreeoneShoot) {
                        spindex.setPosition(0.43);
                    }
                } else if (color_detected.equals("Purple") && ballPickUp == 1 && !adjusted) {
                    spindex.setPosition(0.56);
                    ballPickUp = 2;
                    timer.reset();
                } else if (color_detected.equals("Green") && ballPickUp == 2 && !adjusted && (timer.time() > 0.3)) {
                    threetwooneShoot = true;
                    adjusted = true;
                    spindex.setPosition(0.1);
                    ballPickUp = 3;
                    timer.reset();
                } else if (color_detected.equals("Purple") && ballPickUp == 2 && !adjusted && (timer.time() > 0.3)) {
                    twothreeoneShoot = true;
                    adjusted = true;
                    spindex.setPosition(0.1);
                    ballPickUp = 3;
                    timer.reset();
                }
            }

            if(gamepad1.x){
//                intake.setPower(0);
            }

            if(gamepad1.dpad_up){
                spindex.setPosition(0);
            }

            if(gamepad1.dpad_right){
                spindex.setPosition(0.43);
            }


            if(gamepad1.dpad_down){
                spindex.setPosition(0.87);
            }
            //add command to stop

//            if(gamepad2.b && onetwothreeShoot){
//                spindex.setPosition(0, 0.43, 0.87);
//            }
//
//            if(threetwooneShoot){
//                spindex.setPosition(0.87, 0.43, 0);
//            }
//
//            if(twothreeoneShoot){
//                spindex.setPosition(0.43, 0.87, )
//            }

            if(gamepad1.right_bumper){
                if(onetwothreeShoot){
                    telemetry.addData("Shoot", "one two three");
                } else if(threetwooneShoot){
                    telemetry.addData("Shoot", "three two one");
                } else if(twothreeoneShoot){
                    telemetry.addData("Shoot", "two three one");
                }

                telemetry.update();
                onetwothreeShoot = false;
                threetwooneShoot = false;
                twothreeoneShoot = false;
                adjusted = false;
            }






//            if (llResult != null && llResult.isValid()){
//                Pose3D botPose = llResult.getBotpose_MT2();
//                telemetry.addData("Tx" , llResult.getTx());
//                telemetry.addData("Ty", llResult.getTy());
//                telemetry.addData("Ta", llResult.getTa());
//                telemetry.addData("BotPose", botPose.toString());
////            telemetry.addData("Yaw", botPose.getOrientation().getYaw());
//                telemetry.update();
//            }
        }

    }

    public void mosaic(LLResult llResult, Limelight3A limelight){
        int n = 0;
        while(n <= 10) {
            llResult = limelight.getLatestResult();

            int tagId = 21;

            if (llResult != null && llResult.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    // This is the AprilTag ID
                    tagId = (int) fiducial.getFiducialId();

                    if (tagId == 21) {
                        telemetry.addData("Detected Tag ID", "GPP");
                    } else if (tagId == 22) {
                        telemetry.addData("Detected Tag ID", "PGP");
                    } else if (tagId == 23) {
                        telemetry.addData("Detected Tag ID", "PPG");
                    }

                    telemetry.update();
                }
            }

            n++;
        }
    }
}