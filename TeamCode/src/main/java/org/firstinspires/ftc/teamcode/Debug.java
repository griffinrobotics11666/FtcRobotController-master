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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Debug", group="Driver Control")
@Disabled
public class Debug extends LinearOpMode
{
    public final int ARM_COUNTS_PER_INCH=275;
    public final int ROTATOR_COUNTERS_PER_DEGREE = 15;
    public static int statePosition = 234;
    public static int extendPosition = 6;
    public static double armSpeed = 0.5;
    public static double flipperSpeed = 1.0;
    public static double CLAW_CLOSED_POSITION=.04;
    public static double CLAW_OPENED_POSITION=.2;
    public static double TURNER_COLLECT_POSITION=.25;
    public static double TURNER_DROP_POSITION=.65;
    boolean xLastState = false;
    boolean xCurrentState = false;
    boolean turnerIsDrop = false;
    boolean aLastState = false;
    boolean aCurrentState = false;
    boolean clawIsOpen = false;

    private ElapsedTime runtime = new ElapsedTime(); //clock

    Hardwarerobot robot   = new Hardwarerobot();
    @Override

    public void runOpMode() {
        robot.init(hardwareMap);
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
            aLastState = aCurrentState;
            aCurrentState = gamepad1.a;
            xLastState = xCurrentState;
            xCurrentState = gamepad1.x;

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
            if (gamepad1.y) {
                robot.armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.armExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.dpad_up){
                extendArm(armSpeed);
            }
            if (gamepad1.dpad_down){
                retractArm(armSpeed);
            }
            if (gamepad2.dpad_up){
                extendFlipper(flipperSpeed);
            }
            if (gamepad2.dpad_down){
                retractFlipper(flipperSpeed);
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
            if (xCurrentState && ! xLastState) {
                robot.turner.setPosition(TURNER_COLLECT_POSITION );
                if (!turnerIsDrop){
                    robot.turner.setPosition(TURNER_DROP_POSITION);
                    turnerIsDrop = true;
                }
                else {
                    robot.turner.setPosition(TURNER_COLLECT_POSITION);
                    turnerIsDrop = false;
                }
            }
            leftFrontPower = Range.clip(drive + turn, -1.0, 1.0);
            leftBackPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFrontPower = Range.clip(drive - turn, -1.0, 1.0);
            rightBackPower = Range.clip(drive - turn, -1.0, 1.0);

            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.leftBackDrive.setPower(leftBackPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.rightBackDrive.setPower(rightBackPower);
            robot.carousel.setPower(carouselPower);
            robot.armExtender.setPower(armExtendorPower);
            telemetry.addData("Status", "Run Time: %.2f", runtime.seconds());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.addData("clawPosition",  "at %.2f :", robot.claw.getPosition());
            telemetry.addData("turnerPosition",  "at %.2f :", robot.turner.getPosition());
            telemetry.addData("Path2",  "Running at %7d :", robot.armExtender.getCurrentPosition());
            telemetry.update();
        }
    }

    public void extendArm(double speed) {
        int newTarget;
        if (opModeIsActive()) {
            newTarget = (int)(extendPosition * ARM_COUNTS_PER_INCH);

            robot.armExtender.setTargetPosition(newTarget);
            robot.armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armExtender.setPower(Math.abs(speed));

            while (opModeIsActive() && (robot.armExtender.isBusy()))  {
                telemetry.addData("Path1",  "Running to %7d :", newTarget);
                telemetry.addData("Path2",  "Running at %7d :", robot.armExtender.getCurrentPosition());
                telemetry.update();
            }

            robot.armExtender.setPower(0);
            robot.armExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void retractArm(double speed) {
        int newTarget;
        int distance = 0;

        if (opModeIsActive()) {
            newTarget = (int) (distance * ROTATOR_COUNTERS_PER_DEGREE);

            robot.armExtender.setTargetPosition(newTarget);
            robot.armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armExtender.setPower(Math.abs(speed));

            while (opModeIsActive() && (robot.armExtender.isBusy())) {

                telemetry.addData("Path1", "Running to %7d :", newTarget);
                telemetry.addData("Path2", "Running at %7d :", robot.armExtender.getCurrentPosition());
                telemetry.update();
            }
            robot.armExtender.setPower(0);
            robot.armExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void extendFlipper(double speed) {
        int newTarget;
        if (opModeIsActive()) {
            newTarget = (int)(statePosition * ROTATOR_COUNTERS_PER_DEGREE);

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
        }
    }

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
                telemetry.addData("Path2",  "Running at %7d :", robot.flipper.getCurrentPosition());
                telemetry.update();
            }
            robot.flipper.setPower(0);
            robot.flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
