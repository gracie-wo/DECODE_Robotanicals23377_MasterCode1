package org.firstinspires.ftc.teamcode.main.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

        Servo kicker_rotate = hardwareMap.get(Servo.class, "kicker1");
        CRServo kicker_continuous = hardwareMap.get(CRServo.class, "kicker2");

//        DcMotor launcher = hardwareMap.dcMotor.get("launcher");
//        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ElapsedTime timer = new ElapsedTime();

        //intake & spindex
        boolean adjusted = false;
        int ballPickUp = 1;
        boolean onetwothreeShoot = false;
        boolean threetwooneShoot = false;
        boolean twothreeoneShoot = false;
        boolean detected = false;
        boolean sensing = false;

        //launch spindex
        boolean in_position = false;
        boolean spinToLaunch = false;
        boolean stopLaunchSequence = false;
        int rotate_state = 0;
        int current_state = 0;
        int wait_time = 0;
        boolean end_state = false;
        double spinTime = 0.4;
        boolean start = false;
        boolean restart = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            NormalizedRGBA colors = sensor.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());

            if(hue < 245 && hue > 220){
                color_detected = "Purple";
                telemetry.addData("Color Detected:", "Purple");
                telemetry.update();
                detected = true;
            } else if(hue > 120 && hue < 185){
                color_detected = "Green";
                telemetry.addData("Color Detected:", "Green");
                telemetry.update();
                detected = true;
            } else {
                color_detected = "None";
                telemetry.addData("Color Detected:", "None");
                telemetry.update();
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

                    in_position = true;
                    sensing = false;
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

            if(gamepad1.dpad_up){
                spindex.setPosition(0);
            }

            if(gamepad1.dpad_right){
                spindex.setPosition(0.43);
            }


            if(gamepad1.dpad_down){
                spindex.setPosition(0.87);
            }


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

            //launching spindex
            if(gamepad1.dpad_left){
                stopLaunchSequence = true;
                kicker_continuous.setPower(0);
                kicker_rotate.setPosition(0.3);
                restart = true;
                start = false;
            }

            if(gamepad1.left_bumper && !start){
                if(!restart) {
                    spinToLaunch = false;
                    stopLaunchSequence = false;
                    rotate_state = 0;
                    current_state = 0;
                    wait_time = 0;
                    end_state = false;
                    spinTime = 0.4;
                    start = true;

                    if (!in_position) {
                        if (onetwothreeShoot) {
                            spindex.setPosition(0);
                        } else if (threetwooneShoot) {
                            spindex.setPosition(0.87);
                        } else if (twothreeoneShoot) {
                            spindex.setPosition(0.43);
                        }

                        spinToLaunch = true;
                        timer.reset();
                    }
                } else {
                    stopLaunchSequence = false;
                    rotate_state = 0;
                    wait_time = 1;
                    kicker_continuous.setPower(1);
                    timer.reset();
                }
            }

            if(spinToLaunch && timer.time() > 0.6){
                spinToLaunch = false;
                in_position = true;
            }

            if(start && in_position && !stopLaunchSequence){
                if(onetwothreeShoot){
                    spindex.setPosition(0);
                } else if(threetwooneShoot){
                    spindex.setPosition(0.87);
                } else if(twothreeoneShoot){
                    spindex.setPosition(0.43);
                }

                kicker_continuous.setPower(1);
                kicker_rotate.setPosition(0.6);
                rotate_state = 1;
                wait_time = 0;
                timer.reset();

                in_position = false;
            }

            if(!stopLaunchSequence && rotate_state == 1 && timer.time() > 0.5){
                kicker_rotate.setPosition(0.3);
                current_state++;
                rotate_state = 0;
                timer.reset();

                if(end_state){
                    kicker_continuous.setPower(0);
                    start = false;
                    restart = false;
                }
            }

            if(!stopLaunchSequence && current_state == 1 && timer.time() > 0.3){
                if(onetwothreeShoot){
                    spindex.setPosition(0.43);
                } else if(threetwooneShoot){
                    spindex.setPosition(0.43);
                } else if (twothreeoneShoot){
                    spindex.setPosition(0.87);
                }
                timer.reset();
                current_state++;
                wait_time = 1;
            }

            if(!stopLaunchSequence && current_state == 3 && timer.time() > 0.3){
                wait_time = 1;
                if(onetwothreeShoot) {
                    spindex.setPosition(0.87);
                } else if(threetwooneShoot){
                    spindex.setPosition(0);
                } else if(twothreeoneShoot){
                    spindex.setPosition(0);
                    spinTime = 0.6;
                }
                current_state++;
                end_state = true;
                timer.reset();
            }

            if(!stopLaunchSequence && wait_time == 1 && timer.time() > spinTime){
                kicker_rotate.setPosition(0.6);
                rotate_state = 1;
                wait_time = 0;
                timer.reset();
                spinTime = 0.4;
            }

        }

    }

}