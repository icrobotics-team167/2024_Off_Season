// Copyright (c) 2024 FRC 167
// https://github.com/icrobotics-team167
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.cotc.drive;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class RepulsorFieldPlanner {
  abstract static class Obstacle {
    double strength;
    boolean positive;

    public Obstacle(double strength, boolean positive) {
      this.strength = strength;
      this.positive = positive;
    }

    public abstract Translation2d getForceAtPosition(Translation2d position, Translation2d target);

    protected double distToForceMag(double dist, double maxRange) {
      if (MathUtil.isNear(0, dist, 1e-2)) {
        dist = 1e-2;
      }
      var forceMag = strength / (dist * dist);
      forceMag -= strength / (maxRange * maxRange);
      forceMag *= positive ? 1 : -1;
      return forceMag;
    }
  }

  static class PointObstacle extends Obstacle {
    Translation2d loc;
    double effectMaxRange = 0.5;

    public PointObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = loc.getDistance(position);
      if (dist > effectMaxRange) {
        return Translation2d.kZero;
      }
      var outwardsMag = distToForceMag(loc.getDistance(position), effectMaxRange);
      var outwardsVector = new Translation2d(outwardsMag, position.minus(loc).getAngle());
      // theta = angle between position->target vector and obstacle->position vector
      var theta = target.minus(position).getAngle().minus(position.minus(loc).getAngle());
      double mag = outwardsMag * Math.signum(Math.sin(theta.getRadians() / 2)) / 2;

      var sidewaysVector =
          outwardsVector.rotateBy(Rotation2d.kCCW_90deg).div(outwardsVector.getNorm()).times(mag);

      return outwardsVector.plus(sidewaysVector);
    }
  }

  static class SnowmanObstacle extends Obstacle {
    Translation2d loc;
    final double primaryMaxRange = 1.5;
    final double secondaryMaxRange = 1;

    public SnowmanObstacle(Translation2d loc, double strength, boolean positive) {
      super(strength, positive);
      this.loc = loc;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var targetToLoc = loc.minus(target);
      var targetToLocAngle = targetToLoc.getAngle();
      // 1 meter away from loc, opposite target.
      var sidewaysCircle = new Translation2d(1, targetToLoc.getAngle()).plus(loc);
      var dist = loc.getDistance(position);
      var sidewaysDist = sidewaysCircle.getDistance(position);
      if (dist > primaryMaxRange && sidewaysDist > secondaryMaxRange) {
        return Translation2d.kZero;
      }
      var sidewaysMag = distToForceMag(sidewaysCircle.getDistance(position), primaryMaxRange) / 2;
      var outwardsMag = distToForceMag(loc.getDistance(position), secondaryMaxRange);
      var initial = new Translation2d(outwardsMag, position.minus(loc).getAngle());

      // flip the sidewaysMag based on which side of the goal-sideways circle the robot is on
      var sidewaysTheta =
          target.minus(position).getAngle().minus(position.minus(sidewaysCircle).getAngle());

      double sideways = sidewaysMag * Math.signum(Math.sin(sidewaysTheta.getRadians()));
      var sidewaysAngle = targetToLocAngle.rotateBy(Rotation2d.kCCW_90deg);
      return new Translation2d(sideways, sidewaysAngle).plus(initial);
    }
  }

  static class HorizontalObstacle extends Obstacle {
    double y;

    public HorizontalObstacle(double y, double strength, boolean positive) {
      super(strength, positive);
      this.y = y;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getY() - y);
      if (dist > .5) {
        return Translation2d.kZero;
      }
      return new Translation2d(0, distToForceMag(y - position.getY(), .5));
    }
  }

  static class VerticalObstacle extends Obstacle {
    double x;

    public VerticalObstacle(double x, double strength, boolean positive) {
      super(strength, positive);
      this.x = x;
    }

    public Translation2d getForceAtPosition(Translation2d position, Translation2d target) {
      var dist = Math.abs(position.getX() - x);
      if (dist > .5) {
        return Translation2d.kZero;
      }
      return new Translation2d(distToForceMag(x - position.getX(), .5), 0);
    }
  }

  public static final double GOAL_STRENGTH = 1;

  static final List<Obstacle> FIELD_OBSTACLES =
      List.of(
          new SnowmanObstacle(new Translation2d(5.56, 2.74), 0.4, true),
          new SnowmanObstacle(new Translation2d(3.45, 4.07), 0.4, true),
          new SnowmanObstacle(new Translation2d(5.56, 5.35), 0.4, true),
          new SnowmanObstacle(new Translation2d(11.0, 2.74), 0.4, true),
          new SnowmanObstacle(new Translation2d(13.27, 4.07), 0.4, true),
          new SnowmanObstacle(new Translation2d(11.0, 5.35), 0.4, true));
  static final double FIELD_LENGTH = 16.42;
  static final double FIELD_WIDTH = 8.16;
  static final List<Obstacle> WALLS =
      List.of(
          new HorizontalObstacle(0.0, 0.5, true),
          new HorizontalObstacle(FIELD_WIDTH, 0.5, false),
          new VerticalObstacle(0.0, 0.5, true),
          new VerticalObstacle(FIELD_LENGTH, 0.5, false));

  private final List<Obstacle> fixedObstacles = new ArrayList<>();
  private Translation2d goal = new Translation2d(1, 1);

  private static final int ARROWS_X = 20;
  private static final int ARROWS_Y = 10;
  private static final int ARROWS_SIZE = (ARROWS_X + 1) * (ARROWS_Y + 1);
  private final ArrayList<Pose2d> arrows = new ArrayList<>(ARROWS_SIZE);

  public RepulsorFieldPlanner() {
    fixedObstacles.addAll(FIELD_OBSTACLES);
    fixedObstacles.addAll(WALLS);
    for (int i = 0; i < ARROWS_SIZE; i++) {
      arrows.add(new Pose2d());
    }
  }

  private final Pose2d arrowBackstage = new Pose2d(-10, -10, Rotation2d.kZero);

  private Translation2d lastGoal;
  private Pose2d[] arrowList;

  // A grid of arrows drawn in AScope
  Pose2d[] getArrows() {
    if (goal.equals(lastGoal)) {
      return arrowList;
    }
    for (int x = 0; x <= ARROWS_X; x++) {
      for (int y = 0; y <= ARROWS_Y; y++) {
        var translation =
            new Translation2d(x * FIELD_LENGTH / ARROWS_X, y * FIELD_WIDTH / ARROWS_Y);
        var force = getObstacleForce(translation, goal).plus(getGoalForce(translation, goal));
        if (force.getNorm() < 1e-6) {
          arrows.set(x * (ARROWS_Y + 1) + y, arrowBackstage);
        } else {
          var rotation = force.getAngle();

          arrows.set(x * (ARROWS_Y + 1) + y, new Pose2d(translation, rotation));
        }
      }
    }
    lastGoal = goal;
    arrowList = arrows.toArray(new Pose2d[0]);
    return arrowList;
  }

  Translation2d getGoalForce(Translation2d curLocation, Translation2d goal) {
    var displacement = goal.minus(curLocation);
    if (displacement.getNorm() == 0) {
      return new Translation2d();
    }
    var direction = displacement.getAngle();
    var mag = GOAL_STRENGTH * (1 + 1.0 / (0.0001 + displacement.getNorm()));
    return new Translation2d(mag, direction);
  }

  Translation2d getObstacleForce(Translation2d curLocation, Translation2d target) {
    var force = Translation2d.kZero;
    for (Obstacle obs : fixedObstacles) {
      force = force.plus(obs.getForceAtPosition(curLocation, target));
    }
    return force;
  }

  Translation2d getForce(Translation2d curLocation, Translation2d target) {
    return getGoalForce(curLocation, target).plus(getObstacleForce(curLocation, target));
  }

  private SwerveSample sample(Translation2d trans, Rotation2d rot, double vx, double vy) {
    return new SwerveSample(
        0,
        trans.getX(),
        trans.getY(),
        rot.getRadians(),
        vx,
        vy,
        0,
        0,
        0,
        0,
        new double[4],
        new double[4]);
  }

  public void setGoal(Translation2d goal) {
    this.goal = goal;
  }

  public SwerveSample getCmd(Pose2d pose, double maxSpeed) {
    double stepSize_m = maxSpeed * 0.02; // TODO
    var curTrans = pose.getTranslation();
    var err = curTrans.minus(goal);
    if (err.getNorm() < stepSize_m * 1.5) {
      return sample(goal, pose.getRotation(), 0, 0);
    } else {
      var obstacleForce = getObstacleForce(curTrans, goal);

      var netForce = getGoalForce(curTrans, goal).plus(obstacleForce);
      // Calculate how quickly to move in this direction
      var closeToGoalMax = maxSpeed * Math.min(err.getNorm() / 2, 1);

      stepSize_m = Math.min(maxSpeed, closeToGoalMax) * 0.02;
      var step = new Translation2d(stepSize_m, netForce.getAngle());
      var intermediateGoal = curTrans.plus(step);
      return sample(intermediateGoal, pose.getRotation(), step.getX() / 0.02, step.getY() / 0.02);
    }
  }

  public ArrayList<Translation2d> getTrajectory(Translation2d current, double stepSize_m) {
    ArrayList<Translation2d> trajectory = new ArrayList<>();
    Translation2d robot = current;
    for (int i = 0; i < 400; i++) {
      var err = robot.minus(goal);
      if (err.getNorm() < stepSize_m * 1.5) {
        trajectory.add(goal);
        break;
      } else {
        var netForce = getForce(robot, goal);
        if (netForce.getNorm() == 0) {
          break;
        }
        var step = new Translation2d(stepSize_m, netForce.getAngle());
        var intermediateGoal = robot.plus(step);
        trajectory.add(intermediateGoal);
        robot = intermediateGoal;
      }
    }
    return trajectory;
  }
}
