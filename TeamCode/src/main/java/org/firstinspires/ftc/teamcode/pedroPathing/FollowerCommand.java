package org.firstinspires.ftc.teamcode.pedroPathing;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public class FollowerCommand extends CommandBase {
    private Follower follower;
    private PathChain path;
    private double velocity;
    private boolean roll;

    public FollowerCommand(Follower follower, PathChain path) {
        this(follower, path, 1.0);
    }

    public FollowerCommand(Follower follower, PathChain path, double velocity) {
        this(follower, path, velocity, false);
    }

    public FollowerCommand(Follower follower, PathChain path, double velocity, boolean roll) {
        this.follower = follower;
        this.path = path;
        this.velocity = velocity;
        this.roll = roll;
    }

    public void initialize() {
        follower.followPath(path, velocity, true);
//        follower.followPath(path);
    }

    @Override
    public void execute() {
//        follower.update();
    }

    public boolean isFinished() {
//        return follower.atParametricEnd();
        if (roll) {
            if (follower.getCurrentTValue() >= 0.95) {
                follower.pausePathFollowing();
                return true;
            }
        }
        return !follower.isBusy();
    }
}
