package org.firstinspires.ftc.teamcode.purepursuit.actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.purepursuit.Robot;
import org.firstinspires.ftc.teamcode.purepursuit.math.Maths;

public class ActionList {
    Robot bot;

    public ActionList(Robot bot) {
        this.bot = bot;
    }

    public Action setShooterVelocity(int vel) {
        return new InstantAction(() -> bot.setShooterVelocity(vel));
    }
    public SequentialAction combine(Action... actions) {
        return new SequentialAction(actions);
    }

    public Action setHood(double pos) {
        return new InstantAction(() -> bot.hood.setPosition(pos));
    }
    public Action setIntakePower(double power) {
        return new InstantAction(() -> bot.intakePower(power));
    }
    public Action setTransferPower(double power) {
        return new InstantAction(() -> bot.transferPower(power));
    }
    public Action stopDt() {
        return new InstantAction(() -> bot.drive.setPowers());
    }
    public Action setTurretPos(int pos) {
        return new InstantAction(() -> bot.setTurretPos(pos));
    }
    public Action odomTurretBlueClose() {
        return new InstantAction(() -> {
            Pose2d pose = bot.drive.localizer.getPose();
            double goalHeading = Math.atan2(Robot.blueGoalFar.y - pose.position.y, Robot.blueGoalFar.x - pose.position.x);
            double heading = goalHeading - pose.heading.toDouble();
            heading = Math.atan2(Math.sin(heading), Math.cos(heading));
            Robot.turretTarget = Maths.clamp((Math.toDegrees(heading) / Robot.ticksToDegrees), -630, 630) + Robot.turretOffset;
        });
    }
    public Action openHardStop() {
        return new InstantAction(() -> bot.hardstop.setPosition(Robot.HardstopOpen));
    }
    public Action closeHardStop() {
        return new InstantAction(() -> bot.hardstop.setPosition(Robot.HardstopClose));
    }
    public Action startCheckLoop() {
        return new InstantAction(() -> Robot.runCheckLoop = true);
    }
    public Action stopCheckLoop() {
        return new InstantAction(() -> Robot.runCheckLoop = false);
    }
}
