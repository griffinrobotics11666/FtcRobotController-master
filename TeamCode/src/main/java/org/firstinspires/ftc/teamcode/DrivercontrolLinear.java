/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Driver control catz are cool Linear", group="Driver Control")
@Config
//@Disabled
public class DrivercontrolLinear extends LinearOpMode
{
    int ARM_COUNTS_PER_INCH=275*5;
    int ROTATOR_COUNTERS_PER_DEGREE = 7;
    int flipperState = 0;
    int state0Position = 0;
    public static int state1Position = 55;
    int state2Position = 85;
    public static int state3Position = 125;
    public static double CLAW_CLOSED_POSITION=1; //Top Bucket
    public static double CLAW_OPENED_POSITION=.7;
    public static double TURNER_COLLECT_POSITION=.175;
    public static double TURNER_DROP_POSITION=0;
    boolean turnerIsDrop = false;
    boolean aLastState = false;
    boolean aCurrentState = false;
    boolean bLastState = false;
    boolean bCurrentState = false;
    boolean clawIsOpen = false;
    double lastExtendTime = 0;
    double lastFlipperRetractTime = 0;
    double lastFlipperExtendTime = 0;
    double delayTime = 250;
    double slowFactor = 3;
    public static double carouselSpeed = 1;

    private ElapsedTime runtime = new ElapsedTime(); //clock

    Hardwarerobot robot   = new Hardwarerobot();
    @Override

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        robot.flipper.setTargetPosition(robot.flipper.getCurrentPosition());
        robot.armExtender.setTargetPosition(robot.armExtender.getCurrentPosition());
        telemetry.addData("Status", "Ready");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double leftFrontPower;
            double leftBackPower;
            double rightFrontPower;
            double rightBackPower;
            double carouselPower;
            double armExtendorPower;
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double strafe = gamepad1.right_stick_x;
            aLastState = aCurrentState;
            aCurrentState = gamepad2.a;
            bLastState = bCurrentState;
            bCurrentState = gamepad2.b;
            int currentAngle = robot.flipper.getCurrentPosition()/ROTATOR_COUNTERS_PER_DEGREE;
/*
            if (currentAngle > 90) {
                robot.turner.setPosition(TURNER_DROP_POSITION);
            }
            else {
                robot.turner.se                                     tPosition(TURNER_COLLECT_POSITION);
            }
            */
            carouselPower = gamepad1.right_trigger - gamepad1.left_trigger;
            if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                armExtendorPower = -1;
            } else if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                armExtendorPower = 1;
            } else if ((gamepad1.left_bumper && gamepad1.right_bumper) || (!gamepad1.left_bumper && !gamepad1.right_bumper)) {
                armExtendorPower = 0;
            } else {
                armExtendorPower = 0;
            }
            /*if (gamepad1.y) {
                robot.armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
             */
            if (gamepad1.dpad_up){
                lastExtendTime = runtime.milliseconds();
                extendArm(.5);
            }
            if (gamepad1.dpad_down){
                retractArm(.5);
            }
            if (gamepad2.dpad_up && (runtime.milliseconds() - lastFlipperExtendTime > delayTime)){
                lastFlipperExtendTime = runtime.milliseconds();
                extendFlipper(1);
            }
            if (gamepad2.dpad_down && (runtime.milliseconds() - lastFlipperRetractTime > delayTime)){
                lastFlipperRetractTime = runtime.milliseconds();
                retractFlipper(1);
            }
            if (gamepad2.x) {
                dropPointFlipper(1);
            }
            if (aCurrentState && ! aLastState) {
                robot.claw.setPosition(CLAW_CLOSED_POSITION);
                if (!clawIsOpen) {
                    robot.claw.setPosition(CLAW_OPENED_POSITION);
                    clawIsOpen = true;
                } else {
                    robot.claw.setPosition(CLAW_CLOSED_POSITION);
                    clawIsOpen = false;
                }
            }

            if (bCurrentState && ! bLastState) {
                robot.turner.setPosition(TURNER_COLLECT_POSITION );
                if (!turnerIsDrop){
                    robot.turner.setPosition(TURNER_DROP_POSITION);
                    turnerIsDrop = true;
                    robot.claw.setPosition(CLAW_OPENED_POSITION);
                    clawIsOpen = true;
                }
                else {
                    robot.turner.setPosition(TURNER_COLLECT_POSITION);
                    turnerIsDrop = false;
                    robot.claw.setPosition(CLAW_CLOSED_POSITION);
                    clawIsOpen = false;
                }
            }
            leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            rightBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);

            if( gamepad1.right_bumper){
                leftFrontPower = leftFrontPower / slowFactor;
                leftBackPower = leftBackPower / slowFactor;
                rightFrontPower = rightFrontPower / slowFactor;
                rightBackPower = rightBackPower / slowFactor;
            }


            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.leftBackDrive.setPower(leftBackPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.rightBackDrive.setPower(rightBackPower);

            if(carouselPower>0){
                robot.carousel.setPower(carouselSpeed);
            }
            else if(carouselPower<0){
                robot.carousel.setPower(-carouselSpeed);
            }
            else {
                robot.carousel.setPower(0);
            }


            robot.armExtender.setPower(armExtendorPower);
            telemetry.addData("Status", "Run Time: %.2f", runtime.seconds());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.addData("clawPosition",  "at %.2f :", robot.claw.getPosition());
            telemetry.addData("turnerPosition",  "at %.2f :", robot.turner.getPosition());
            telemetry.addData("flipperState",   flipperState);
            telemetry.update();
        }
    }
    public void extendArm(double speed) {
        int newTarget;
        int distance = 6;

        if (opModeIsActive()) {

            newTarget = (int)(distance * ARM_COUNTS_PER_INCH);

            robot.armExtender.setTargetPosition(newTarget);

            robot.armExtender.setPower(Math.abs(speed));
        }
    }

    public void retractArm(double speed) {
        int newTarget;
        int distance = 0;

        if (opModeIsActive()) {
            newTarget = (int) (distance * ROTATOR_COUNTERS_PER_DEGREE);

            robot.armExtender.setTargetPosition(newTarget);

            robot.armExtender.setPower(Math.abs(speed));
            //robot.armExtender.setPower(0);
            //robot.armExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void extendFlipper(double speed) {
        int newTarget;
        int distance = 0;

        if (opModeIsActive()) {


            if(flipperState == 0 || flipperState == 1) {
                distance = (state3Position);
                flipperState = 3;
            }
            /*
            else if(flipperState == 1) {
                distance = state2Position;
                flipperState = 2;
            }
            else if(flipperState == 2) {
                distance = state3Position;
                flipperState = 3;
            }

             */
            else {
                distance = state3Position;
                flipperState = 3;
            }
            newTarget = (int)(distance * ROTATOR_COUNTERS_PER_DEGREE);

            robot.flipper.setTargetPosition(newTarget);
            robot.flipper.setPower(Math.abs(speed));
            //robot.flipper.setPower(0);
            //robot.flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void retractFlipper(double speed) {
        int newTarget;
        int distance=0;

        if (opModeIsActive()) {

            if(flipperState == 3 || flipperState == 1) {
                distance = (state0Position);
                flipperState = 0;
            }
            /*
            else if(flipperState == 2) {
                distance = state1Position;
                flipperState = 1;
            }
            else if(flipperState == 1) {
                distance = state0Position;
                flipperState = 0;
            }
             */
            else {
                distance = state0Position;
                flipperState = 0;
            }


            newTarget = (int)(distance * ROTATOR_COUNTERS_PER_DEGREE);

            robot.flipper.setTargetPosition(newTarget);
            robot.flipper.setPower(Math.abs(speed));
        }

    }
    public void dropPointFlipper(double speed) {
        int newTarget;
        int distance=0;

        if (opModeIsActive()) {

            if(flipperState == 3 || flipperState == 0) {
                distance = (state1Position);
                flipperState = 1;
            }
            /*
            else if(flipperState == 2) {
                distance = state1Position;
                flipperState = 1;
            }
            else if(flipperState == 1) {
                distance = state0Position;
                flipperState = 0;
            }
             */
            else {
                distance = state1Position;
                flipperState = 1;
            }


            newTarget = (int)(distance * ROTATOR_COUNTERS_PER_DEGREE);

            robot.flipper.setTargetPosition(newTarget);
            robot.flipper.setPower(Math.abs(speed));
        }
    }
}
    /*public void extendFlipper(double speed) {
        int newTarget;
        int distance = 90;

        if (opModeIsActive()) {

            newTarget = (int)(distance * ROTATOR_COUNTERS_PER_DEGREE);

            robot.flipper.setTargetPosition(newTarget);
            robot.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.flipper.setPower(Math.abs(speed));

            while (opModeIsActive() && (robot.flipper.isBusy()))  {

                telemetry.addData("Path1",  "Running to %7d :", newTarget);
                telemetry.addData("Path2",  "Running at %7d :", robot.flipper.getCurrentPosition());
                telemetry.update();
            }
            robot.flipper.setPower(0);

            robot.flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    public void retractFlipper(double speed) {
        int newTarget;
        int distance=0;

        if (opModeIsActive()) {

            newTarget = (int)(distance * ROTATOR_COUNTERS_PER_DEGREE);

            robot.flipper.setTargetPosition(newTarget);
            robot.flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.flipper.setPower(Math.abs(speed));

            while (opModeIsActive() && (robot.flipper.isBusy()))  {

                telemetry.addData("Path1",  "Running to %7d :", newTarget);
                telemetry.addData("Path2",  "Running at %7d :",
                        robot.flipper.getCurrentPosition());
                telemetry.update();
            }

            robot.flipper.setPower(0);
            robot.flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        */