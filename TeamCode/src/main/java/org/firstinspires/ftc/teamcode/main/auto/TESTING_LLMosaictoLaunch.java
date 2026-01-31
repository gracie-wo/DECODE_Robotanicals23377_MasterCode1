package org.firstinspires.ftc.teamcode.main.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.List;

@Disabled
//blue april tag
@Config
@Autonomous(name = "TESTING_LLMosaic to Launch", group = "testing")
public class TESTING_LLMosaictoLaunch extends LinearOpMode {

    //limelight
    String pattern = "GPP";

    //MAY CAUSE ERRORS
    private Limelight3A limelight;
    private LLResult llResult;


    //------------------------------------LIMELIGHT-----------------------------------------
    public class Limelight{
        public Limelight(HardwareMap hardwareMap){
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0);
            llResult = limelight.getLatestResult();
            limelight.start();
        }

        public class MoasicDetect implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                int n = 0;
                while(n <= 2) {
                    llResult = limelight.getLatestResult();
                    limelight.pipelineSwitch(0);

                    int tagId = 21;
                    if (llResult != null && llResult.isValid()) {
                        Pose3D botPose = llResult.getBotpose_MT2();
                        telemetry.addData("Distance", getDistanceFromTags(llResult.getTa()));
                        telemetry.addData("Tx", llResult.getTx());
                        telemetry.addData("Ty", llResult.getTy());
                        telemetry.addData("Ta", llResult.getTa());
                        telemetry.addData("BotPose", botPose.toString());
//                      telemetry.addData("Yaw", botPose.getOrientation().getYaw());
                        telemetry.update();
                        List<LLResultTypes.FiducialResult> fiducials = llResult.getFiducialResults();

                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            // This is the AprilTag ID
                            tagId = (int) fiducial.getFiducialId();

                            if (tagId == 21) {
                                telemetry.addData("Detected Tag ID", "GPP");
                                pattern = "GPP";
                            } else if (tagId == 22) {
                                telemetry.addData("Detected Tag ID", "PGP");
                                pattern = "PGP";
                            } else if (tagId == 23) {
                                telemetry.addData("Detected Tag ID", "PPG");
                                pattern = "PPG";
                            }


                            telemetry.update();
                        }
                    }

                    n++;
                }

                return false;
            }
        }

        public Action mosaicDetect(){
            return new MoasicDetect();
        }

        public class SwitchPipeline implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                limelight.pipelineSwitch(2);

                return false;
            }
        }

        public Action switchPipeline(){
            return new SwitchPipeline();
        }
     }

     //------------------------------------MOTORS--------------------------------------------
    public class Intake {
        private DcMotorEx intake;

        public Intake(HardwareMap hardwareMap){
            intake = hardwareMap.get(DcMotorEx.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public class IntakeOn implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(1);
                return false;
            }
        }

        public Action intakeOn() {
            return new IntakeOn();
        }

        public class IntakeOff implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPower(0);
                return false;
            }
        }

        public Action intakeOff() {
            return new IntakeOff();
        }
    }

    public class Launcher {
        private DcMotorEx launcher;

        private double voltChange;
        private VoltageSensor controlHubVoltageSensor;
        private double launchPower;

        //accel forward to target speed
        final double NEWR_P = 2;
        //ability to change intertia (change direction
        final double NEWR_I = 0.2;
        //jerk lmao
        final double NEWR_D = 0.7;
        //idek
        final double NEWR_F = 20.0;

        public Launcher(HardwareMap hardwareMap){
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            DcMotorControllerEx motorControllerExR = (DcMotorControllerEx)launcher.getController();
            int motorIndexR = ((DcMotorEx)launcher).getPortNumber();

            PIDFCoefficients pidfNewR = new PIDFCoefficients(NEWR_P, NEWR_I, NEWR_D, NEWR_F);
            motorControllerExR.setPIDFCoefficients(motorIndexR, DcMotor.RunMode.RUN_USING_ENCODER, pidfNewR);

            controlHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
            voltChange = voltSpeed(controlHubVoltageSensor);

            launchPower = (0.0025 * 175) + voltChange;
        }

        public class LaunchOn implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                llResult = limelight.getLatestResult();
                voltChange = voltSpeed(controlHubVoltageSensor);
//
                if(llResult != null && llResult.isValid()){
                    double distance = getDistanceFromTags(llResult.getTa());
                    launchPower = (0.0025 * (distance)) + voltChange;
                } else {
                    launchPower = (0.0025 * 185) + voltChange;
                }
                //185
                launcher.setPower(launchPower);
                return false;
            }
        }

        public Action launchOn() {
            return new LaunchOn();
        }

        public class LaunchOff implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.setPower(0);
                return false;
            }
        }

        public Action launchOff() {
            return new LaunchOff();
        }
    }

    //----------------------------SERVOS---------------------------------------------------
    public class Spindex {
        private Servo spindex;

        int linePickUp;

        public Spindex(HardwareMap hardwareMap){
            spindex = hardwareMap.get(Servo.class, "spindex");
            linePickUp = 1;
            //values for pickup (1, 2, 3): 1, 0.1, 0.56
        }

        //intake
        public class SpindexIntakeOne implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if(linePickUp == 1){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(0.1);
                        return false;
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(1);
                        return false;
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(1);
                        return false;
                    }
                } else if(linePickUp == 2){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(0.56);
                        return false;
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(0.56);
                        return false;
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(0.1);
                        return false;
                    }
                } else if(linePickUp == 3){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(1);
                        return false;
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(0.1);
                        return false;
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(0.56);
                        return false;
                    }
                }

                spindex.setPosition(0.1);
                return false;
            }
        }

        public Action spindexIntakeOne(){
            return new SpindexIntakeOne();
        }


        public class SpindexIntakeTwo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if(linePickUp == 1){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(0.56);
                        return false;
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(0.56);
                        return false;
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(0.1);
                        return false;
                    }
                } else if(linePickUp == 2){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(1);
                        return false;
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(0.1);
                        return false;
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(0.56);
                        return false;
                    }
                } else if(linePickUp == 3){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(0.56);
                        return false;
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(0.56);
                        return false;
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(0.1);
                        return false;
                    }
                }

                spindex.setPosition(0.56);
                return false;
            }
        }

        public Action spindexIntakeTwo(){
            return new SpindexIntakeTwo();
        }


        public class SpindexIntakeThree implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if(linePickUp == 1){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(1);
                        return false;
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(0.1);
                        return false;
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(0.56);
                        return false;
                    }

                    linePickUp = 2;
                } else if(linePickUp == 2){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(0.1);
                        return false;
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(1);
                        return false;
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(1);
                        return false;
                    }

                    linePickUp = 3;
                } else if(linePickUp == 3){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(0.1);
                        return false;
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(1);
                        return false;
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(1);
                        return false;
                    }
                }

                spindex.setPosition(1);
                return false;
            }
        }

        public Action spindexIntakeThree(){
            return new SpindexIntakeThree();
        }

        //launch first time
        public class StartSpindexLaunchOne implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                telemetry.addData("Mosaic", pattern);
                telemetry.update();

                if(pattern.equals("GPP")){
                    spindex.setPosition(0);
                } else if(pattern.equals("PPG")){
                    spindex.setPosition(0.87);
                } else if(pattern.equals("PGP")){
                    spindex.setPosition(0.43);
                } else {
                    //do GPP
                    spindex.setPosition(0);
                }

                return false;
            }
        }

        public Action startSpindexLaunchOne(){
            return new StartSpindexLaunchOne();
        }

        public class StartSpindexLaunchTwo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if(pattern.equals("GPP")){
                    spindex.setPosition(0.43);
                } else if(pattern.equals("PPG")){
                    spindex.setPosition(0.43);
                } else if(pattern.equals("PGP")){
                    spindex.setPosition(0);
                } else {
                    spindex.setPosition(0.43);
                }
                return false;
            }
        }

        public Action startSpindexLaunchTwo(){
            return new StartSpindexLaunchTwo();
        }

        public class StartSpindexLaunchThree implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if(pattern.equals("GPP")){
                    spindex.setPosition(0.87);
                } else if(pattern.equals("PPG")){
                    spindex.setPosition(0);
                } else if(pattern.equals("PGP")){
                    spindex.setPosition(0.87);
                } else {
                    spindex.setPosition(0.87);
                }
                return false;
            }
        }

        public Action startSpindexLaunchThree(){
            return new StartSpindexLaunchThree();
        }

        //launch
        public class SpindexLaunchOne implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                spindex.setPosition(0);
                return false;
            }
        }

        public Action spindexLaunchOne(){
            return new SpindexLaunchOne();
        }

        public class SpindexLaunchTwo implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                spindex.setPosition(0.43);
                return false;
            }
        }

        public Action spindexLaunchTwo(){
            return new SpindexLaunchTwo();
        }

        public class SpindexLaunchThree implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                spindex.setPosition(0.87);
                return false;
            }
        }

        public Action spindexLaunchThree(){
            return new SpindexLaunchThree();
        }
    }

    public class Rotator {
        private Servo rotator;
        private double launchPosition;
        private boolean adjusted;

        public Rotator(HardwareMap hardwareMap){
            rotator = hardwareMap.get(Servo.class, "rotator");
            launchPosition = 0.5;
            adjusted = false;
        }

        public class LLRotate implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                //find usual position
                launchPosition = 0.4;
                rotator.setPosition(launchPosition);
                adjusted = false;

                while (!adjusted) {
                    llResult = limelight.getLatestResult();

                    if(llResult != null && llResult.isValid()){
                        if(llResult.getTx() < -5){
                            if(launchPosition > 0){
//--------------------------------CHANGE----------------------------------------------
                                launchPosition -= 0.002;
                                rotator.setPosition(launchPosition);
                            }
                        } else if(llResult.getTx() > 5){
                            if(launchPosition < 0.8){
                                launchPosition += 0.002;
                                rotator.setPosition(launchPosition);
                            }
                        } else {
                            adjusted = true;
                        }
                    }
                }
                return false;
            }
        }

        public Action llRotate(){
            return new LLRotate();
        }


        public class RotateLaunch implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rotator.setPosition(0.4);
                return false;
            }
        }

        public Action rotateLaunch(){
            return new RotateLaunch();
        }

        public class RotateMosaic implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rotator.setPosition(0);
                return false;
            }
        }

        public Action rotateMosaic(){
            return new RotateMosaic();
        }


    }
     public class KickerCont {
        private CRServo kickerCont;

        public KickerCont(HardwareMap hardwareMap){
            kickerCont = hardwareMap.get(CRServo.class, "kicker2");
        }

        public class KickerContOn implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                kickerCont.setPower(1);
                return false;
            }
        }

        public Action kickerContOn(){
            return new KickerContOn();
        }


        public class KickerContOff implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                kickerCont.setPower(0);
                return false;
            }
        }

        public Action kickerContOff(){
            return new KickerContOff();
        }
    }

    public class KickerRotate {
        private Servo kickerRotate;

        public KickerRotate(HardwareMap hardwareMap){
            kickerRotate = hardwareMap.get(Servo.class, "kicker1");
        }

        public class KickerRotateUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                kickerRotate.setPosition(0.6);
                return false;
            }
        }

        public Action kickerRotateUp(){
            return new KickerRotateUp();
        }


        public class KickerRotateDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                kickerRotate.setPosition(0.3);
                return false;
            }
        }

        public Action kickerRotateDown(){
            return new KickerRotateDown();
        }

        public class KickerRotateDownInit implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                kickerRotate.setPosition(0.3);
                return false;
            }
        }

        public Action kickerRotateDownInit(){
            return new KickerRotateDownInit();
        }
    }

    @Override
    public void runOpMode() {

        //instantiate at (0,0)
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(-55));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Limelight limelight = new Limelight(hardwareMap);

        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        KickerRotate kickerRotate = new KickerRotate(hardwareMap);
        KickerCont kickerCont = new KickerCont(hardwareMap);
        Spindex spindex = new Spindex(hardwareMap);
        Rotator rotator = new Rotator(hardwareMap);

        TrajectoryActionBuilder one = drive.actionBuilder(initialPose)
                .strafeToConstantHeading (new Vector2d(4, 0));
        TrajectoryActionBuilder two = one.fresh()
                .strafeToConstantHeading (new Vector2d(8, 0));
        TrajectoryActionBuilder three = two.fresh()
                .strafeToConstantHeading (new Vector2d(12, 0));

        Action oneA = one.build();
        Action twoA = two.build();
        Action threeA = three.build();



        TrajectoryActionBuilder launch1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading (new Vector2d(-28, -42), Math.toRadians(-55));

        Action launch1A = launch1.build();

        waitForStart();
        if (isStopRequested()) return;


        // ------------------------- RUN AUTO -------------------------
        Actions.runBlocking(
                new SequentialAction(
                        limelight.mosaicDetect()
//                        new SleepAction(0.5)

//                        //USE MORE PARALLEL ACTIONS BRO
//                        //can parallel action rotator spinning and spindex spinning
//                        spindex.startSpindexLaunchOne(),
//                        new SleepAction(0.8)
//                        //prob don't need this if you just time how long it takes for rotator to spin to yknow
//                        //but 0.8 = how much time to spin one full circle = worst time complexity
//
//
//                        spindex.startSpindexLaunchTwo(),
//                        new SleepAction(0.6),
//                        //0.6 = how much time wait for spindex to spin to next location (second time)
//
//                        spindex.startSpindexLaunchThree(),
//                        new SleepAction(0.8)
//                        //0.8 = how much time wait for spindex to spin to next location for THIRD time
                )
        );


    }

    public double getDistanceFromTags(double ta){
        //CHANGE SCALE NUM (CALCULATE)

        double scale = 29280.39;
        double distance = Math.sqrt(scale/ta) ;
        return distance;
    }

    public double voltSpeed(VoltageSensor controlHubVoltageSensor){
        double voltage = controlHubVoltageSensor.getVoltage();

        if(voltage >= 13.5){
            return 0;
        }else if(voltage >= 13.1){
            return 0.05;
        } else if (voltage >= 12.6){
            return 0.1;
        } else if (voltage >= 12.1){
            return 0.125;
        } else if (voltage >= 11.6){
            return 0.15;
        } else if (voltage >= 11.1){
            return 0.175;
        } else if (voltage >= 10.6){
            return 0.2;
        } else {
            return 0.05;
        }
    }
}