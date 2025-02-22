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

        public Slide(HardwareMap hardwareMap) {
            slide = hardwareMap.get(DcMotor.class, "Slide");
        }

        public class Basket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(-1400);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(.6);
                return slide.isBusy();
            }
        }

        public Action Basket() {
            return new ScoreRR.Slide.Basket();
        }

        public class Retract implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(0);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(.6);
                return slide.isBusy();
            }
        }

        public Action Retract() {
            return new ScoreRR.Slide.Retract();
        }

        public class Mid implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slide.setTargetPosition(-650);
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slide.setPower(.6);
                return slide.isBusy();
            }
        }
        public Action Mid() {
            return new ScoreRR.Slide.Mid();
        }
    }
    public class Arm {
        private DcMotorEx arm;
        public Arm(HardwareMap hardwareMap){
            arm = (DcMotorEx) hardwareMap.get(DcMotor.class, "Arm");
            arm.setTargetPositionTolerance(35);
        }

        public class ArmUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setTargetPosition(-2554);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.8);
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
                arm.setPower(.8);
                return arm.isBusy();
            }
        }
        public Action ArmDown() {
            return new ScoreRR.Arm.ArmDown();
        }

        public class ArmHold  implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                arm.setTargetPosition(-0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.3);
                return arm.isBusy();
            }}
        public Action ArmHold() {
            return new ScoreRR.Arm.ArmHold();
        }

        public class Armbit  implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                arm.setTargetPosition(-750);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.3);
                return arm.isBusy();
            }}
        public Action Armbit() {
            return new ScoreRR.Arm.Armbit();
        }
    }


    @Override
    public void runOpMode(){
        /* X+ is to the right
//        -18
         *  Y+ is away from you
         *  0 Heading is towards back of field */
        Pose2d startpose = new Pose2d(-46.5, -63.5, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startpose);
        ScoringClaw scoringClaw = new ScoringClaw(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        TrajectoryActionBuilder park = drive.actionBuilder(startpose)
//                .strafeTo(new Vector2d(-48, -63.5))
                .strafeToLinearHeading(new Vector2d(-54, -63.5), Math.toRadians(15));


//        TrajectoryActionBuilder shift = park.endTrajectory().fresh()
//                        .turnTo(Math.toRadians(15));


//        TrajectoryActionBuilder turn =park.endTrajectory().fresh()
//                .turnTo(Math.toRadians(90));

        TrajectoryActionBuilder two = park.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-60, -32), Math.toRadians(90));



        TrajectoryActionBuilder score = two.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-60, -56), Math.toRadians(40));



        TrajectoryActionBuilder three = score.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-70, -32), Math.toRadians(90));

        TrajectoryActionBuilder threesco = three.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-60, -56), Math.toRadians(40));

        TrajectoryActionBuilder Boom = threesco.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-74, -38), Math.toRadians(123));

        TrajectoryActionBuilder YASSS = Boom.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-60, -56), Math.toRadians(40));





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

        sleep(500);
        Actions.runBlocking(new ParallelAction(
                scoringClaw.Open(),
                wrist.Pickup(),
                slide.Retract(),
                arm.ArmDown()

        ));


        Actions.runBlocking(new SequentialAction(
                two.build(),
                scoringClaw.close()));

        Actions.runBlocking(new SequentialAction(
                score.build()));

        Actions.runBlocking(new ParallelAction(
                slide.Basket(),
                arm.ArmUp(),
                wrist.Score()));
        sleep(250);

        Actions.runBlocking(new SequentialAction(
                scoringClaw.Open()
        ));
        sleep(250);
        Actions.runBlocking(new ParallelAction(
                wrist.Pickup(),
                slide.Retract(),
                arm.ArmDown()));
        Actions.runBlocking(new SequentialAction(
                three.build(),
                scoringClaw.close()));

        Actions.runBlocking(new SequentialAction(
                threesco.build()));

        Actions.runBlocking(new ParallelAction(
                slide.Basket(),
                arm.ArmUp(),
                wrist.Score()
                ));
        sleep(250);
Actions.runBlocking(new SequentialAction(
        scoringClaw.Open()
));

        sleep(250);
        Actions.runBlocking(new ParallelAction(
                wrist.Pickup(),
                slide.Retract()
        ));

        Actions.runBlocking(new ParallelAction(
                Boom.build()
        ));

        Actions.runBlocking(new ParallelAction(
                arm.ArmHold(),
                slide.Mid()
        ));

        Actions.runBlocking(new SequentialAction(
                scoringClaw.close(),
               arm.Armbit(),
               YASSS.build()
        ));

        Actions.runBlocking(new ParallelAction(
                slide.Basket(),
                arm.ArmUp(),
                wrist.Score()));
        sleep(250);
        Actions.runBlocking(new SequentialAction(
                scoringClaw.Open()
        ));

        Actions.runBlocking(new ParallelAction(
               wrist.Pickup(),
                slide.Retract(),
                arm.ArmDown()

        ));




    }
}
