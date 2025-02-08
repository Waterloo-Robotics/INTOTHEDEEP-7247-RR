package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name="ScoreRR")
public class ScoreRR extends LinearOpMode {
    public class ScoringClaw{
        private Servo claw;
        public ScoringClaw(HardwareMap hardwareMap){
            claw = hardwareMap.get(Servo.class, "Claw");
        }

        public class Open implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action Open() {
            return new ScoreRR.ScoringClaw.Open();
        }

        public class Close implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1);
                return false;
            }
        }
        public Action close() {
            return new ScoreRR.ScoringClaw.Close();
        }
    }

    public class Wrist {
        private Servo wrist;
        public Wrist(HardwareMap hardwareMap){
            wrist = hardwareMap.get(Servo.class, "Wrist");
        }

        public class Score implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(0);
                return false;
            }
        }
        public Action Score () {
            return new ScoreRR.Wrist.Score();
        }

        public class Pickup implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                wrist.setPosition(1);
                return false;
            }
        }
        public Action Pickup () {
            return new ScoreRR.Wrist.Pickup();
        }
    }
    public class Slide {
        private DcMotor slide;
        public Slide(HardwareMap hardwareMap){
            slide = hardwareMap.get(DcMotor.class, "Slide");
        }

        public class Basket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(-1500);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(.5);
                return slide.isBusy();
            }
        }
        public Action Basket () {
            return new ScoreRR.Slide.Basket();
        }

        public class Retract implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(.5);
                return slide.isBusy();
            }
        }
        public Action Retract() {
            return new ScoreRR.Slide.Retract();
        }
    }

    public class Arm {
        private DcMotor arm;
        public Arm(HardwareMap hardwareMap){
            arm = hardwareMap.get(DcMotor.class, "Arm");
        }

        public class ArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(-2554);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.5);
                return arm.isBusy();
            }
        }
        public Action ArmUp () {
            return new ScoreRR.Arm.ArmUp();
        }

        public class ArmDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.5);
                return arm.isBusy();
            }
        }
        public Action ArmDown() {
            return new ScoreRR.Arm.ArmDown();
        }
    }

    @Override
    public void runOpMode(){
        /* X+ is to the right
//        -18
         *  Y+ is away from you
         *  0 Heading is towards back of field */
        Pose2d startpose = new Pose2d(-18, -63.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startpose);
        ScoringClaw scoringClaw = new ScoringClaw(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        TrajectoryActionBuilder park = drive.actionBuilder(startpose)
                .strafeTo(new Vector2d(-48, -63.5))
                .waitSeconds(1);

        TrajectoryActionBuilder shift = park.endTrajectory().fresh()
                        .turnTo(Math.toRadians(15));


        TrajectoryActionBuilder turn =shift.endTrajectory().fresh()
                .turnTo(Math.toRadians(90));

        TrajectoryActionBuilder two = turn.endTrajectory().fresh()
                        .strafeTo(new Vector2d(-53, -32));

        TrajectoryActionBuilder score = two.endTrajectory().fresh()
                        .strafeTo(new Vector2d (-53,-56 ))
                                .turnTo(Math.toRadians(40));

        TrajectoryActionBuilder three = score.endTrajectory().fresh()
                        .turnTo(Math.toRadians(90))
                                .strafeTo(new Vector2d(-63, -32));

        TrajectoryActionBuilder threesco = three.endTrajectory().fresh()
                .strafeTo(new Vector2d (-53,-56 ))
                .turnTo(Math.toRadians(40));



        Actions.runBlocking(new ParallelAction(
                scoringClaw.close()
        ));


        arm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        if (isStopRequested())return;

        Actions.runBlocking(new ParallelAction(
                park.build(),
                arm.ArmUp(),
                slide.Basket(),
                wrist.Score()
        ));
        Actions.runBlocking(new SequentialAction(
                shift.build()
                ));

        Actions.runBlocking(new SequentialAction(
                scoringClaw.Open(),
                wrist.Pickup(),
                slide.Retract(),
                arm.ArmDown()

        ));
        Actions.runBlocking(new SequentialAction(
                turn.build()
        ));

        Actions.runBlocking(new SequentialAction(
                two.build(),
                scoringClaw.close()));


        sleep(500);

        Actions.runBlocking(new SequentialAction(
                score.build(),
                slide.Basket(),
                arm.ArmUp(),
                wrist.Score()));
        sleep(1000);
        Actions.runBlocking(new ParallelAction(
                scoringClaw.Open(),
                wrist.Pickup(),
                slide.Retract(),
                arm.ArmDown()));
        Actions.runBlocking(new SequentialAction(
                three.build(),
                scoringClaw.close()));


        Actions.runBlocking(new SequentialAction(
                threesco.build(),
                slide.Basket(),
                arm.ArmUp(),
                wrist.Score()
                ));

        sleep(1000);
        Actions.runBlocking(new ParallelAction(
                scoringClaw.Open(),
                wrist.Pickup(),
                slide.Retract(),
                arm.ArmDown()

        ));
    }
}
