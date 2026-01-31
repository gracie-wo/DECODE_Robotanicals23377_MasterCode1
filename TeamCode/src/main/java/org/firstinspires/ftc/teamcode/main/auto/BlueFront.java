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


@Config
@Autonomous(name = "BLUE front", group = "Blue Main")
public class BlueFront extends LinearOpMode {

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
                while(n <= 10) {
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

        public class SwitchPipeline2 implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                limelight.pipelineSwitch(1);

                return false;
            }
        }

        public Action switchPipeline2(){
            return new SwitchPipeline2();
        }

        public class SwitchPipeline0 implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                limelight.pipelineSwitch(0);

                return false;
            }
        }

        public Action switchPipeline0(){
            return new SwitchPipeline0();
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

            launchPower = (0.0024 * 140) + voltChange;
        }

        public class LaunchOnOne implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//                llResult = limelight.getLatestResult();
                voltChange = voltSpeed(controlHubVoltageSensor);
//
//                if(llResult != null && llResult.isValid()){
//                    double distance = getDistanceFromTags(llResult.getTa());
//                    launchPower = (0.0025 * (distance)) + voltChange;
//                } else {
                launchPower = (0.0024 * 140) + voltChange;
//                }
                //185
                launcher.setPower(launchPower);
                return false;
            }
        }

        public Action launchOnOne() {
            return new LaunchOnOne();
        }

        public class LaunchOnTwo implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//                llResult = limelight.getLatestResult();

                voltChange = voltSpeed(controlHubVoltageSensor);
////
//                if(llResult != null && llResult.isValid()){
//                    double distance = getDistanceFromTags(llResult.getTa());
//                    launchPower = (0.0025 * (distance)) + voltChange + 0.18;
//                } else {
                launchPower = (0.0024 * 140) + voltChange + 0.18;
//                }
                //185
                launcher.setPower(launchPower);
                return false;
            }
        }

        public Action launchOnTwo() {
            return new LaunchOnTwo();
        }

        public class LaunchOnThree implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//                llResult = limelight.getLatestResult();
                voltChange = voltSpeed(controlHubVoltageSensor);
//
//                if(llResult != null && llResult.isValid()){
//                    double distance = getDistanceFromTags(llResult.getTa());
//                    launchPower = (0.0025 * (distance)) + voltChange + 0.13;
//                } else {
                launchPower = (0.0024 * 140) + voltChange + 0.13;
//                }
                //185
                launcher.setPower(launchPower);
                return false;
            }
        }

        public Action launchOnThree() {
            return new LaunchOnThree();
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

        public class LaunchOnStart implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //VALUE OF 185 MAY NEED TO BE ADJUSTED
                launcher.setPower((0.0024 * 200) + voltChange);
                return false;
            }
        }

        public Action launchOnStart() {
            return new LaunchOnStart();
        }

        public class LaunchOn2 implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //VALUE OF 185 MAY NEED TO BE ADJUSTED
                launcher.setPower((0.0024 * 250) + voltChange);
                return false;
            }
        }

        public Action launchOn2() {
            return new LaunchOn2();
        }

        public class LaunchOn3 implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //VALUE OF 185 MAY NEED TO BE ADJUSTED
                launcher.setPower((0.0024 * 220) + voltChange);
                return false;
            }
        }

        public Action launchOn3() {
            return new LaunchOn3();
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
                telemetry.addData("Line num", linePickUp);
                telemetry.update();
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
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(0.1);
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(0.56);
                    }

                    linePickUp = 2;
                    return false;

                } else if(linePickUp == 2){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(0.1);
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(1);
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(1);
                    }

                    linePickUp = 3;
                    return false;
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

        public class SpindexIntakeThreeStay implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                if(linePickUp == 2){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(1);
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(0.1);
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(0.56);
                    }

                    return false;

                } else if(linePickUp == 3){
                    if(pattern.equals("GPP")){
                        spindex.setPosition(0.1);
                    } else if(pattern.equals("PGP")){
                        spindex.setPosition(1);
                    } else if(pattern.equals("PPG")){
                        spindex.setPosition(1);
                    }

                    return false;
                } else if(linePickUp == 4){
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

        public Action spindexIntakeThreeStay() {
            return new SpindexIntakeThreeStay();
        }

        //launch first time
        public class StartSpindexLaunchOne implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
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
                rotator.setPosition(0.35);
                return false;
            }
        }

        public Action rotateLaunch(){
            return new RotateLaunch();
        }

        public class RotateLaunchStart implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet){
                rotator.setPosition(0.75);
                return false;
            }
        }

        public Action rotateLaunchStart(){
            return new RotateLaunchStart();
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
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(55));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Limelight limelight = new Limelight(hardwareMap);

        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        KickerRotate kickerRotate = new KickerRotate(hardwareMap);
        KickerCont kickerCont = new KickerCont(hardwareMap);
        Spindex spindex = new Spindex(hardwareMap);
        Rotator rotator = new Rotator(hardwareMap);


        TrajectoryActionBuilder launch1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading (new Vector2d(-28, 33), Math.toRadians(25));

        TrajectoryActionBuilder one = launch1.fresh()
                .strafeToLinearHeading (new Vector2d(-37, -18), Math.toRadians(90))
                .strafeToLinearHeading (new Vector2d(-37, -15), Math.toRadians(90));
        TrajectoryActionBuilder two = one.fresh()
                .strafeToConstantHeading (new Vector2d(-35, -12));
        TrajectoryActionBuilder three = two.fresh()
                .strafeToConstantHeading (new Vector2d(-33, -6));

        TrajectoryActionBuilder launch2 = three.fresh()
                .strafeToLinearHeading (new Vector2d(-28, -33), Math.toRadians(55));

        TrajectoryActionBuilder one2 = launch2.fresh()
                .strafeToLinearHeading (new Vector2d(-59, -20), Math.toRadians(90))
                .strafeToLinearHeading (new Vector2d(-59, -17), Math.toRadians(90));
        TrajectoryActionBuilder two2 = one2.fresh()
                .strafeToConstantHeading (new Vector2d(-57, -13));
        TrajectoryActionBuilder three2 = two2.fresh()
                .strafeToConstantHeading (new Vector2d(-55, -5));

        TrajectoryActionBuilder launch3 = three2.fresh()
                .strafeToLinearHeading (new Vector2d(-28, -33), Math.toRadians(55));


        Action oneA = one.build();
        Action twoA = two.build();
        Action threeA = three.build();

        Action launch1A = launch1.build();
        Action launch2A = launch2.build();
        Action launch3A = launch3.build();

        Action one2A = one2.build();
        Action two2A = two2.build();
        Action three2A = three2.build();

        Actions.runBlocking(limelight.switchPipeline0());
        Actions.runBlocking(kickerRotate.kickerRotateDownInit());
        Actions.runBlocking(rotator.rotateLaunch());



        waitForStart();
        if (isStopRequested()) return;


        // ------------------------- RUN AUTO -------------------------
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                launcher.launchOnStart(),
                                new SequentialAction(
                                        new ParallelAction(
                                                launch1A,
                                                rotator.rotateMosaic()
                                        ),

                                        limelight.mosaicDetect(),

                                        new ParallelAction(
                                                limelight.switchPipeline2(),
                                                spindex.startSpindexLaunchOne(),
                                                rotator.rotateLaunchStart(),
                                                kickerCont.kickerContOn(),
                                                new SleepAction(0.6)
                                        )
                                )
                        ),


                        new ParallelAction(
                                launcher.launchOnOne(),
                                new SequentialAction(
                                        kickerRotate.kickerRotateUp(),
                                        new SleepAction(0.5),
                                        kickerRotate.kickerRotateDown(),
                                        new SleepAction(0.3)
                                )
                        ),

                        new ParallelAction(
                                launcher.launchOnTwo(),
                                spindex.startSpindexLaunchTwo(),
                                new SleepAction(0.4)

                        ),

                        new ParallelAction(
                                launcher.launchOnTwo(),
                                new SequentialAction(
                                        kickerRotate.kickerRotateUp(),
                                        new SleepAction(0.5),
                                        kickerRotate.kickerRotateDown(),
                                        new SleepAction(0.3)
                                )
                        ),

                        new ParallelAction(
                                launcher.launchOnThree(),
                                spindex.startSpindexLaunchThree(),
                                new SleepAction(0.6)
                        ),

                        new ParallelAction(
                                launcher.launchOnThree(),
                                new SequentialAction(
                                        kickerRotate.kickerRotateUp(),
                                        new SleepAction(0.5),
                                        kickerRotate.kickerRotateDown(),
                                        new SleepAction(0.3)
                                )
                        ),

                        //go to intake 1
                        new ParallelAction(
                                launcher.launchOff(),
                                kickerCont.kickerContOff(),
                                intake.intakeOn(),
                                new SequentialAction(
                                        new ParallelAction(
                                                oneA,
                                                spindex.spindexIntakeOne()
                                        ),
                                        new SleepAction(0.4),
                                        new ParallelAction(
                                                twoA,
                                                spindex.spindexIntakeTwo()
                                        ),
                                        new SleepAction(0.6),
                                        new ParallelAction(
                                                threeA,
                                                spindex.spindexIntakeThree()
                                        )
                                )
                        ),

                        new ParallelAction(
                                launcher.launchOn2(),
                                new SequentialAction(
                                        spindex.spindexIntakeThreeStay(),
                                        //may cause  error
                                        new SleepAction(0.6),
                                        spindex.spindexLaunchOne()
                                ),
                                rotator.rotateLaunch(),
                                kickerCont.kickerContOn(),
                                launch2A
                        ),

                        new ParallelAction(
                                intake.intakeOff(),
                                launcher.launchOnOne(),
                                new SequentialAction(
                                        kickerRotate.kickerRotateUp(),
                                        new SleepAction(0.5),
                                        kickerRotate.kickerRotateDown(),
                                        new SleepAction(0.3)
                                )
                        ),

                        new ParallelAction(
                                launcher.launchOnTwo(),
                                spindex.spindexLaunchTwo(),
                                new SleepAction(0.4)
                        ),

                        new ParallelAction(
                                launcher.launchOnTwo(),
                                new SequentialAction(
                                        kickerRotate.kickerRotateUp(),
                                        new SleepAction(0.5),
                                        kickerRotate.kickerRotateDown(),
                                        new SleepAction(0.3)
                                )
                        ),

                        new ParallelAction(
                                launcher.launchOnThree(),
                                spindex.spindexLaunchThree(),
                                new SleepAction(0.4)
                        ),

                        new ParallelAction(
                                launcher.launchOnThree(),
                                new SequentialAction(
                                        kickerRotate.kickerRotateUp(),
                                        new SleepAction(0.5),
                                        kickerRotate.kickerRotateDown(),
                                        new SleepAction(0.3)
                                )
                        ),

                        new ParallelAction(
                                launcher.launchOff(),
                                kickerCont.kickerContOff(),
                                intake.intakeOn(),
                                new SequentialAction(
                                        new ParallelAction(
                                                one2A,
                                                spindex.spindexIntakeOne()
                                        ),
                                        new SleepAction(0.4),
                                        new ParallelAction(
                                                two2A,
                                                spindex.spindexIntakeTwo()
                                        ),
                                        new SleepAction(0.4),
                                        new ParallelAction(
                                                three2A,
                                                spindex.spindexIntakeThree()
                                        )
                                )
                        ),


                        new ParallelAction(
                                new SequentialAction(
                                        spindex.spindexIntakeThreeStay(),
                                        //may cause  error
                                        new SleepAction(0.6),
                                        spindex.spindexLaunchOne()
                                ),
                                kickerCont.kickerContOn(),
                                rotator.rotateLaunch(),
                                launcher.launchOnStart(),
                                launch3A
                        ),

                        new ParallelAction(
                                intake.intakeOff(),
                                launcher.launchOnOne(),
                                new SequentialAction(
                                        kickerRotate.kickerRotateUp(),
                                        new SleepAction(0.5),
                                        kickerRotate.kickerRotateDown(),
                                        new SleepAction(0.3)
                                )
                        ),

                        new ParallelAction(
                                launcher.launchOnTwo(),
                                spindex.spindexLaunchTwo(),
                                new SleepAction(0.4)
                        ),

                        new ParallelAction(
                                launcher.launchOnTwo(),
                                new SequentialAction(
                                        kickerRotate.kickerRotateUp(),
                                        new SleepAction(0.5),
                                        kickerRotate.kickerRotateDown(),
                                        new SleepAction(0.3)
                                )
                        ),

                        new ParallelAction(
                                launcher.launchOnThree(),
                                spindex.spindexLaunchThree(),
                                new SleepAction(0.4)
                        ),

                        new ParallelAction(
                                launcher.launchOnThree(),
                                new SequentialAction(
                                        kickerRotate.kickerRotateUp(),
                                        new SleepAction(0.5),
                                        kickerRotate.kickerRotateDown(),
                                        new SleepAction(0.3)
                                )
                        )


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