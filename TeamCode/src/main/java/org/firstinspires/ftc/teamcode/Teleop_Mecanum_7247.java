/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Timer;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="teleop 7247", group="Linear OpMode")

public class Teleop_Mecanum_7247 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor Arm = null;
    private DcMotor Slide = null;
    private Servo Claw = null;
    private Servo Wrist = null;

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.1 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    /* Slide State Variables */
    boolean slide_homed;
    ElapsedTime slide_home_timer;
    boolean slide_home_timer_started;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Claw = hardwareMap.get(Servo.class, "Claw");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
         // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /* Slide Init */
        slide_homed = false;
        slide_home_timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        slide_home_timer_started = false;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = gamepad1.left_stick_y * .5;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * .5;
            double turn    =  gamepad1.right_stick_x * .5;
            double Arm_power    =  gamepad2.left_stick_y * .5;
            boolean ClawClose = gamepad2.right_bumper;
            boolean ClawOpen = gamepad2.left_bumper;
            boolean WristPickUp = gamepad2.b;
            boolean WristFlat = gamepad2.a;
            boolean Score = gamepad2.dpad_up;
            boolean PickUp = gamepad2.dpad_down;
            boolean Retract = gamepad2.dpad_left;
            boolean Lowscore = gamepad2.dpad_right;


            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial - lateral - turn;
            double rightFrontPower = axial + lateral + turn;
            double leftBackPower   = axial + lateral - turn;
            double rightBackPower  = axial - lateral + turn;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            double new_target;

            new_target = (int)(Arm.getCurrentPosition() + Arm_power * 300);

            if (new_target < -2700) {
                new_target = -2700;
            }

            if (new_target > 0) {
                new_target = 0;
            }
            Arm.setPower(1);
            Arm.setTargetPosition( (int)new_target);

//            if (Arm_power < 0 && Arm.getCurrentPosition() < -2700) {
//
//                Arm.setPower(0);
//
//            } else {
//
//                Arm.setPower(Arm_power);
//
//            }

            // 1. Do we know where the slide is?
            // Yes. -> Do normal controls
            if (slide_homed) {

            }
            // No. We need to find it
            else
            {
                // Just start moving down
                Slide.setPower(0.35);
                if (!slide_home_timer_started) {
                    slide_home_timer.reset();
                    slide_home_timer_started = true;
                }

                // If the slide has more or less stopped
                if ((((DcMotorEx)Slide).getVelocity(AngleUnit.DEGREES) < 30) &&
                        (slide_home_timer.milliseconds() > 250) && slide_home_timer_started)
                {
                    Slide.setPower(0);
                    Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Slide.setTargetPosition(0);
                    Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slide_homed = true;
                }

            }

            if(PickUp && Arm.getCurrentPosition() < -580) {
                Slide.setTargetPosition(-900);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(.8);
                Claw.setPosition(1);

                }
//
//
             if(Score && Arm.getCurrentPosition() < -875) {
                 Slide.setTargetPosition(-1500);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(.8);
                Wrist.setPosition(0);
                }
//
//            }
//
//            if(LowScore) {
//                Slide.setTargetPosition(0);
//                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                while(opModeIsActive() && Slide.getCurrentPosition() >= -2506){
//                    Slide.setPower(.5);
//                }
//            }
             if(Retract) {
                 Wrist.setPosition(1);
                Slide.setTargetPosition(0);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slide.setPower(.8);
                }
//
            if (WristPickUp) {
                Wrist.setPosition(0);
            }


            if (WristFlat) {
                Wrist.setPosition(1);
            }

            if (ClawClose) {
                Claw.setPosition(1);
            }

            if (ClawOpen) {
                Claw.setPosition(-1);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Slide Power", "%4.2f", Slide.getPower());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Wrist Position", Wrist.getPosition());
            telemetry.addData("Slide Position", Slide.getCurrentPosition());
            telemetry.addData("Arm Position",  Arm.getCurrentPosition());
            telemetry.addData("Arm Target Position", Arm.getTargetPosition());
            telemetry.update();


        }
    }}
//Back left is port 0 Front left is port 1 Back right is port 2 front right is port 3