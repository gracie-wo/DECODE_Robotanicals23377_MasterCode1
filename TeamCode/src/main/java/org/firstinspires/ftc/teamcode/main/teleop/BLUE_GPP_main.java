package org.firstinspires.ftc.teamcode.main.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@TeleOp(name = "Blue GPP", group = "Blue Main")
public class BLUE_GPP_main extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        LLResult llResult = limelight.getLatestResult();
        limelight.start();

        VoltageSensor controlHubVoltageSensor;
        controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        double voltChange = voltSpeed(controlHubVoltageSensor);

        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//OFFSETS NEED TO BE CHANGED
        odo.setOffsets(44, 60.2, DistanceUnit.MM);


        DcMotor intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo spindex = hardwareMap.get(Servo.class, "spindex");
        NormalizedColorSensor sensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        double hue = 0.0;
        String color_detected = "None";

        Servo kicker_rotate = hardwareMap.get(Servo.class, "kicker1");
        CRServo kicker_continuous = hardwareMap.get(CRServo.class, "kicker2");

        DcMotor launcher = hardwareMap.dcMotor.get("launcher");
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo rotator = hardwareMap.get(Servo.class, "rotator");

        //accel forward to target speed
        final double NEWR_P = 2;
        //ability to change intertia (change direction
        final double NEWR_I = 0.2;
        //jerk lmao
        final double NEWR_D = 0.7;
        //idek
        final double NEWR_F = 20.0;

        DcMotorControllerEx motorControllerExR = (DcMotorControllerEx)launcher.getController();
        int motorIndexR = ((DcMotorEx)launcher).getPortNumber();

        PIDFCoefficients pidfNewR = new PIDFCoefficients(NEWR_P, NEWR_I, NEWR_D, NEWR_F);
        motorControllerExR.setPIDFCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER, pidfNewR);

        ElapsedTime timer = new ElapsedTime();

        //color sensor
        int green = 0;
        int purple = 0;

        //intake & spindex
        int kicker_start = 0;
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
        boolean onlyKicker= false;

        //launching
        boolean launchDistanceChange = false;
        int secondThird = 0;

        boolean camera_on = false;
        double launchPosition = 0.4;
        double launchPower = 0;
        double distance = 0;

        DcMotor frontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRight = hardwareMap.dcMotor.get("backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            llResult = limelight.getLatestResult();

            NormalizedRGBA colors = sensor.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());

            if(llResult != null && llResult.isValid()){
                telemetry.addData("Tag", "Seen");
                telemetry.update();
            } else {
                telemetry.addData("Tag", "NOT Seen");
                telemetry.update();
            }

            //run kicker quickly
            if(kicker_start == 0){
                timer.reset();
                kicker_continuous.setPower(0.1);
                kicker_rotate.setPosition(0.3);
                kicker_start = 1;
            }

            if(kicker_start == 1 && timer.time() > 0.1){
                kicker_continuous.setPower(0);
                kicker_start = 2;
            }

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;


            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad2.options){
                imu.resetYaw();
            }

            double botHeading = 0.0;

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower* 1);
            backLeft.setPower(backLeftPower* 1);
            frontRight.setPower(frontRightPower* 1);
            backRight.setPower(backRightPower* 1);

//---------------------------------------GAMEPAD 1----------------------------------------
            //intake & spindex
            if(hue < 245 && hue > 220){
                purple++;
//                telemetry.addData("Color Detected:", "Purple");
//                telemetry.update();
            } else if(hue > 120 && hue < 185){
                green++;
//                telemetry.addData("Color Detected:", "Green");
//                telemetry.update();
            } else {
                purple = 0;
                green = 0;
                color_detected = "None";
//                telemetry.addData("Color Detected:", "None");
//                telemetry.update();
                detected = false;
            }

            if(purple >= 10){
                color_detected = "Purple";
                detected = true;
            }
            if(green >= 10){
                color_detected = "Green";
                detected = true;
            }

            if(gamepad1.y){
                spindex.setPosition(1);
                intake.setPower(1);
                ballPickUp = 1;
                onetwothreeShoot = false;
                threetwooneShoot = false;
                twothreeoneShoot = false;
                adjusted = false;
                sensing = true;
                timer.reset();
            }

            //turn camera on/off
            if(gamepad1.right_bumper){
                in_position = false;
                rotator.setPosition(0.4);
                camera_on = true;
            }

            if(gamepad1.left_bumper){
                in_position = false;
                camera_on = false;
                telemetry.clear();
                telemetry.addData("Camera:", "Off");
                telemetry.update();
            }


            if(camera_on && llResult.getTx() < -5){
                if(launchPosition > 0){
                    launchPosition -= 0.002;
                    rotator.setPosition(launchPosition);
                }
            }

            if(camera_on && llResult.getTx() > 5){
                if(launchPosition < 0.8){
                    launchPosition += 0.002;
                    rotator.setPosition(launchPosition);
                }
            }

            if(gamepad1.x){
                intake.setPower(0);
            }

            if(gamepad1.a){
                intake.setPower(-1);
            }

            if(sensing) {
                //green ball first
                if (color_detected.equals("Green") && ballPickUp == 1 && !adjusted && timer.time() > 1) {
                    onetwothreeShoot = true;
                    adjusted = true;
                    spindex.setPosition(0.56);
                    ballPickUp = 2;
                    timer.reset();
                } else if (adjusted && ballPickUp == 2 && detected && (timer.time() > 1)) {
                    spindex.setPosition(0.1);
                    ballPickUp = 3;
                    timer.reset();
                } else if (ballPickUp == 3 && detected && (timer.time() > 1)) {
                    intake.setPower(-1);
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
                } else if (color_detected.equals("Green") && ballPickUp == 2 && !adjusted && (timer.time() > 1)) {
                    threetwooneShoot = true;
                    adjusted = true;
                    spindex.setPosition(0.1);
                    ballPickUp = 3;
                    timer.reset();
                } else if (color_detected.equals("Purple") && ballPickUp == 2 && !adjusted && (timer.time() > 1)) {
                    twothreeoneShoot = true;
                    adjusted = true;
                    spindex.setPosition(0.1);
                    ballPickUp = 3;
                    timer.reset();
                }
            }

            if(gamepad1.dpad_up){
                sensing = false;
                spindex.setPosition(1);
            }

            if(gamepad1.dpad_right){
                sensing = false;
                spindex.setPosition(0.1);
            }

            if(gamepad1.dpad_down){
                sensing = false;
                spindex.setPosition(0.56);
            }

            //intake position
            if(gamepad1.dpad_left){
                sensing = false;
                onetwothreeShoot = true;
                spindex.setPosition(0);
            }

//-------------------------------GAMEPAD 2--------------------------------------------------
            //MANUAL launch positions
            if(gamepad2.dpad_up){
                spindex.setPosition(0);
            }

            if(gamepad2.dpad_right){
                spindex.setPosition(0.43);
            }


            if(gamepad2.dpad_down){
                spindex.setPosition(0.87);
            }

            //intake position
            if(gamepad2.dpad_left){
                spindex.setPosition(1);
            }

            //manual kicker
            if(gamepad2.y){
                kicker_continuous.setPower(1);
                kicker_rotate.setPosition(0.6);
                onlyKicker = true;
                timer.reset();
            }

            if(onlyKicker && timer.time() > 0.5){
                kicker_continuous.setPower(0);
                kicker_rotate.setPosition(0.3);
                onlyKicker = false;
            }

            if(gamepad2.right_bumper){
                launchDistanceChange = true;
                //may need to delete
                voltChange = voltSpeed(controlHubVoltageSensor);
            }

            if(gamepad2.left_bumper){
                in_position = false;
                launchDistanceChange = false;
                launcher.setPower(0);
            }

            if(camera_on && launchDistanceChange && llResult != null && llResult.isValid()){
                distance = getDistanceFromTags(llResult.getTa());

                if(secondThird <= 1){
                    launchPower = (0.0024 * (distance)) + voltChange;
                } else if (secondThird == 2) {
                    launchPower = (0.0024 * distance) + voltChange + 0.18;
                } else {
                    launchPower = (0.0024 * distance) + voltChange + 0.13;
                }

                launcher.setPower(launchPower);
            } else if(launchDistanceChange){
                launcher.setPower((0.0025 * 175) + voltChange);
            }

            //stop auto launch sequence
            if(gamepad2.a){
                stopLaunchSequence = true;
                kicker_continuous.setPower(0);
                kicker_rotate.setPosition(0.3);
                restart = true;
                start = false;

                secondThird = 0;
            }

            //START AUTO LAUNCH SEQUENCE
            if(gamepad2.b && !start){
                if(!restart) {
                    spinToLaunch = false;
                    stopLaunchSequence = false;
                    rotate_state = 0;
                    current_state = 0;
                    wait_time = 0;
                    end_state = false;
                    spinTime = 0.4;
                    start = true;
                    secondThird = 0;

                    if(!onetwothreeShoot && !twothreeoneShoot && !threetwooneShoot){
                        onetwothreeShoot = true;
                    }

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

                secondThird++;

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
                    secondThird = 0;
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

                secondThird++;
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

                secondThird++;

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

    public double getDistanceFromTags(double ta){
        //CHANGE SCALE NUM (CALCULATE)

        double scale = 29280.39;
        double distance = Math.sqrt(scale/ta);
        return distance;
    }

    public double voltSpeed(VoltageSensor controlHubVoltageSensor){
        double voltage = controlHubVoltageSensor.getVoltage();
        double power;

        if(voltage >= 12.6 && voltage <= 13){
            power = ((-0.0585 * voltage) + 0.844191);
        } else {
            power = ((-0.0600978 * voltage) + 0.844191);
        }

        if(power < 0){
            return 0;
        } else {
            return power;
        }
    }

}