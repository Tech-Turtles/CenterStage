package org.firstinspires.ftc.teamcode.utility.pathplanner.path;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.utility.math.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.utility.math.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.utility.pathplanner.json.JSONArray;
import org.firstinspires.ftc.teamcode.utility.pathplanner.json.JSONObject;
import org.firstinspires.ftc.teamcode.utility.pathplanner.json.parser.JSONParser;
import org.firstinspires.ftc.teamcode.utility.pathplanner.util.GeometryUtil;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;

/** A PathPlanner path. NOTE: This is not a trajectory and isn't directly followed. */
public class PathPlannerPath {
    private List<Translation2d> bezierPoints;
    private List<RotationTarget> rotationTargets;
    private List<ConstraintsZone> constraintZones;
    private PathConstraints globalConstraints;
    private GoalEndState goalEndState;
    private List<PathPoint> allPoints;
    private boolean reversed;
    private Rotation2d previewStartingRotation;


    /**
     * Create a new path planner path
     *
     * @param bezierPoints List of points representing the cubic Bezier curve of the path
     * @param holonomicRotations List of rotation targets along the path
     * @param constraintZones List of constraint zones along the path
     * @param globalConstraints The global constraints of the path
     * @param goalEndState The goal end state of the path
     * @param reversed Should the robot follow the path reversed (differential drive only)
     */
    public PathPlannerPath(
            List<Translation2d> bezierPoints,
            List<RotationTarget> holonomicRotations,
            List<ConstraintsZone> constraintZones,
            PathConstraints globalConstraints,
            GoalEndState goalEndState,
            boolean reversed,
            Rotation2d previewStartingRotation) {
        this.bezierPoints = bezierPoints;
        this.rotationTargets = holonomicRotations;
        this.constraintZones = constraintZones;
        this.globalConstraints = globalConstraints;
        this.goalEndState = goalEndState;
        this.reversed = reversed;
        this.allPoints = createPath(this.bezierPoints, this.rotationTargets, this.constraintZones);
        this.previewStartingRotation = previewStartingRotation;

        precalcValues();
    }

    /**
     * Simplified constructor to create a path with no rotation targets, or constraint zones.
     *
     * <p>You likely want to use bezierFromPoses to create the bezier points.
     *
     * @param bezierPoints List of points representing the cubic Bezier curve of the path
     * @param constraints The global constraints of the path
     * @param goalEndState The goal end state of the path
     * @param reversed Should the robot follow the path reversed (differential drive only)
     */
    public PathPlannerPath(
            List<Translation2d> bezierPoints,
            PathConstraints constraints,
            GoalEndState goalEndState,
            boolean reversed) {
        this(
                bezierPoints,
                Collections.emptyList(),
                Collections.emptyList(),
                constraints,
                goalEndState,
                reversed,
                Rotation2d.fromDegrees(0));
    }

    /**
     * Simplified constructor to create a path with no rotation targets, or constraint zones.
     *
     * <p>You likely want to use bezierFromPoses to create the bezier points.
     *
     * @param bezierPoints List of points representing the cubic Bezier curve of the path
     * @param constraints The global constraints of the path
     * @param goalEndState The goal end state of the path
     */
    public PathPlannerPath(
            List<Translation2d> bezierPoints, PathConstraints constraints, GoalEndState goalEndState) {
        this(bezierPoints, constraints, goalEndState, false);
    }

    private PathPlannerPath(PathConstraints globalConstraints, GoalEndState goalEndState) {
        this.bezierPoints = new ArrayList<>();
        this.rotationTargets = new ArrayList<>();
        this.constraintZones = new ArrayList<>();
        this.globalConstraints = globalConstraints;
        this.goalEndState = goalEndState;
        this.reversed = false;
        this.allPoints = new ArrayList<>();
    }

    /**
     * Creat a path with pre-generated points. This should already be a smooth path.
     *
     * @param pathPoints Path points along the smooth curve of the path
     * @param constraints The global constraints of the path
     * @param goalEndState The goal end state of the path
     * @return A PathPlannerPath following the given pathpoints
     */
    public static PathPlannerPath fromPathPoints(
            List<PathPoint> pathPoints, PathConstraints constraints, GoalEndState goalEndState) {
        PathPlannerPath path = new PathPlannerPath(constraints, goalEndState);
        path.allPoints = pathPoints;
        path.precalcValues();

        return path;
    }

    /**
     * Create the bezier points necessary to create a path using a list of poses
     *
     * @param poses List of poses. Each pose represents one waypoint.
     * @return Bezier points
     */
    public static List<Translation2d> bezierFromPoses(List<Pose2d> poses) {
        if (poses.size() < 2) {
            throw new IllegalArgumentException("Not enough poses");
        }

        List<Translation2d> bezierPoints = new ArrayList<>();

        // First pose
        bezierPoints.add(poses.get(0).getTranslation());
        bezierPoints.add(
                new Translation2d(
                        poses.get(0).getTranslation().getDistance(poses.get(1).getTranslation()) / 3.0,
                        poses.get(0).getRotation()));

        // Middle poses
        for (int i = 1; i < poses.size() - 2; i++) {
            // Prev control
            bezierPoints.add(
                    new Translation2d(
                            poses.get(i).getTranslation().getDistance(poses.get(i - 1).getTranslation()) / 3.0,
                            poses.get(i).getRotation().plus(Rotation2d.fromDegrees(180))));
            // Anchor
            bezierPoints.add(poses.get(i).getTranslation());
            // Next control
            bezierPoints.add(
                    new Translation2d(
                            poses.get(i).getTranslation().getDistance(poses.get(i + 1).getTranslation()) / 3.0,
                            poses.get(i).getRotation()));
        }

        // Last pose
        bezierPoints.add(
                new Translation2d(
                        poses
                                .get(poses.size() - 1)
                                .getTranslation()
                                .getDistance(poses.get(poses.size() - 2).getTranslation())
                                / 3.0,
                        poses.get(poses.size() - 1).getRotation().plus(Rotation2d.fromDegrees(180))));
        bezierPoints.add(poses.get(poses.size() - 1).getTranslation());

        return bezierPoints;
    }

    /**
     * Create the bezier points necessary to create a path using a list of poses
     *
     * @param poses List of poses. Each pose represents one waypoint.
     * @return Bezier points
     */
    public static List<Translation2d> bezierFromPoses(Pose2d... poses) {
        return bezierFromPoses(Arrays.asList(poses));
    }

    /**
     * Hot reload the path. This is used internally.
     *
     * @param pathJson Updated JSONObject for the path
     */
    public void hotReload(JSONObject pathJson) {
        PathPlannerPath updatedPath = PathPlannerPath.fromJson(pathJson);

        this.bezierPoints = updatedPath.bezierPoints;
        this.rotationTargets = updatedPath.rotationTargets;
        this.constraintZones = updatedPath.constraintZones;
        this.globalConstraints = updatedPath.globalConstraints;
        this.goalEndState = updatedPath.goalEndState;
        this.allPoints = updatedPath.allPoints;
        this.reversed = updatedPath.reversed;
    }

    /**
     * Load a path from a path file in storage
     *
     * @param pathName The name of the path to load
     * @return PathPlannerPath created from the given file name
     */
    public static PathPlannerPath fromPathFile(String pathName) {
        try (BufferedReader br =
                     new BufferedReader(
                             new FileReader(
                                     AppUtil.getInstance().getSettingsFile(pathName + ".path")
                             ))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            PathPlannerPath path = PathPlannerPath.fromJson(json);
            return path;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    private static PathPlannerPath fromJson(JSONObject pathJson) {
        List<Translation2d> bezierPoints =
                bezierPointsFromWaypointsJson((JSONArray) pathJson.get("waypoints"));
        PathConstraints globalConstraints =
                PathConstraints.fromJson((JSONObject) pathJson.get("globalConstraints"));
        GoalEndState goalEndState = GoalEndState.fromJson((JSONObject) pathJson.get("goalEndState"));
        boolean reversed = (boolean) pathJson.get("reversed");
        List<RotationTarget> rotationTargets = new ArrayList<>();
        List<ConstraintsZone> constraintZones = new ArrayList<>();

        for (Object rotJson : (JSONArray) pathJson.get("rotationTargets")) {
            rotationTargets.add(RotationTarget.fromJson((JSONObject) rotJson));
        }

        for (Object zoneJson : (JSONArray) pathJson.get("constraintZones")) {
            constraintZones.add(ConstraintsZone.fromJson((JSONObject) zoneJson));
        }

        Rotation2d previewStartingRotation = Rotation2d.fromDegrees(0);
        if (pathJson.containsKey("previewStartingState")) {
            JSONObject previewStartingStateJson = (JSONObject) pathJson.get("previewStartingState");
            if (previewStartingStateJson != null) {
                previewStartingRotation =
                        Rotation2d.fromDegrees(
                                ((Number) previewStartingStateJson.get("rotation")).doubleValue());
            }
        }

        return new PathPlannerPath(
                bezierPoints,
                rotationTargets,
                constraintZones,
                globalConstraints,
                goalEndState,
                reversed,
                previewStartingRotation);
    }

    private static List<Translation2d> bezierPointsFromWaypointsJson(JSONArray waypointsJson) {
        List<Translation2d> bezierPoints = new ArrayList<>();

        // First point
        JSONObject firstPoint = (JSONObject) waypointsJson.get(0);
        bezierPoints.add(pointFromJson((JSONObject) firstPoint.get("anchor")));
        bezierPoints.add(pointFromJson((JSONObject) firstPoint.get("nextControl")));

        // Mid points
        for (int i = 1; i < waypointsJson.size() - 1; i++) {
            JSONObject point = (JSONObject) waypointsJson.get(i);
            bezierPoints.add(pointFromJson((JSONObject) point.get("prevControl")));
            bezierPoints.add(pointFromJson((JSONObject) point.get("anchor")));
            bezierPoints.add(pointFromJson((JSONObject) point.get("nextControl")));
        }

        // Last point
        JSONObject lastPoint = (JSONObject) waypointsJson.get(waypointsJson.size() - 1);
        bezierPoints.add(pointFromJson((JSONObject) lastPoint.get("prevControl")));
        bezierPoints.add(pointFromJson((JSONObject) lastPoint.get("anchor")));

        return bezierPoints;
    }

    private static Translation2d pointFromJson(JSONObject pointJson) {
        double x = ((Number) pointJson.get("x")).doubleValue();
        double y = ((Number) pointJson.get("y")).doubleValue();

        return new Translation2d(x, y);
    }

    /**
     * Get the constraints for a point along the path
     *
     * @param idx Index of the point to get constraints for
     * @return The constraints that should apply to the point
     */
    public PathConstraints getConstraintsForPoint(int idx) {
        if (getPoint(idx).constraints != null) {
            return getPoint(idx).constraints;
        }

        return globalConstraints;
    }

    private static List<PathPoint> createPath(
            List<Translation2d> bezierPoints,
            List<RotationTarget> holonomicRotations,
            List<ConstraintsZone> constraintZones) {
        if (bezierPoints.size() < 4) {
            throw new IllegalArgumentException("Not enough bezier points");
        }

        List<PathPoint> points = new ArrayList<>();

        int numSegments = (bezierPoints.size() - 1) / 3;
        for (int s = 0; s < numSegments; s++) {
            int iOffset = s * 3;
            Translation2d p1 = bezierPoints.get(iOffset);
            Translation2d p2 = bezierPoints.get(iOffset + 1);
            Translation2d p3 = bezierPoints.get(iOffset + 2);
            Translation2d p4 = bezierPoints.get(iOffset + 3);

            int segmentIdx = s;
            List<RotationTarget> segmentRotations =
                    holonomicRotations.stream()
                            .filter(
                                    target ->
                                            target.getPosition() >= segmentIdx && target.getPosition() <= segmentIdx + 1)
                            .map(target -> target.forSegmentIndex(segmentIdx))
                            .collect(Collectors.toList());
            List<ConstraintsZone> segmentZones =
                    constraintZones.stream()
                            .filter(zone -> zone.overlapsRange(segmentIdx, segmentIdx + 1))
                            .map(zone -> zone.forSegmentIndex(segmentIdx))
                            .collect(Collectors.toList());

            PathSegment segment =
                    new PathSegment(p1, p2, p3, p4, segmentRotations, segmentZones, s == numSegments - 1);
            points.addAll(segment.getSegmentPoints());
        }

        return points;
    }

    private void precalcValues() {
        if (numPoints() > 0) {
            for (int i = 0; i < allPoints.size(); i++) {
                PathPoint point = allPoints.get(i);
                if (point.constraints == null) {
                    point.constraints = globalConstraints;
                }
                point.curveRadius = getCurveRadiusAtPoint(i, allPoints);

                if (Double.isFinite(point.curveRadius)) {
                    point.maxV =
                            Math.min(
                                    Math.sqrt(
                                            point.constraints.getMaxAccelerationMpsSq() * Math.abs(point.curveRadius)),
                                    point.constraints.getMaxVelocityMps());
                } else {
                    point.maxV = point.constraints.getMaxVelocityMps();
                }

                if (i != 0) {
                    point.distanceAlongPath =
                            allPoints.get(i - 1).distanceAlongPath
                                    + (allPoints.get(i - 1).position.getDistance(point.position));
                }
            }

            allPoints.get(allPoints.size() - 1).rotationTarget =
                    new RotationTarget(-1, goalEndState.getRotation(), goalEndState.shouldRotateFast());
            allPoints.get(allPoints.size() - 1).maxV = goalEndState.getVelocity();
        }
    }

    /**
     * Get all the path points in this path
     *
     * @return Path points in the path
     */
    public List<PathPoint> getAllPathPoints() {
        return allPoints;
    }

    /**
     * Get the number of points in this path
     *
     * @return Number of points in the path
     */
    public int numPoints() {
        return allPoints.size();
    }

    /**
     * Get a specific point along this path
     *
     * @param index Index of the point to get
     * @return The point at the given index
     */
    public PathPoint getPoint(int index) {
        return allPoints.get(index);
    }

    /**
     * Get the global constraints for this path
     *
     * @return Global constraints that apply to this path
     */
    public PathConstraints getGlobalConstraints() {
        return globalConstraints;
    }

    /**
     * Get the goal end state of this path
     *
     * @return The goal end state
     */
    public GoalEndState getGoalEndState() {
        return goalEndState;
    }

    private static double getCurveRadiusAtPoint(int index, List<PathPoint> points) {
        if (points.size() < 3) {
            return Double.POSITIVE_INFINITY;
        }

        if (index == 0) {
            return GeometryUtil.calculateRadius(
                    points.get(index).position,
                    points.get(index + 1).position,
                    points.get(index + 2).position);
        } else if (index == points.size() - 1) {
            return GeometryUtil.calculateRadius(
                    points.get(index - 2).position,
                    points.get(index - 1).position,
                    points.get(index).position);
        } else {
            return GeometryUtil.calculateRadius(
                    points.get(index - 1).position,
                    points.get(index).position,
                    points.get(index + 1).position);
        }
    }

    /**
     * Should the path be followed reversed (differential drive only)
     *
     * @return True if reversed
     */
    public boolean isReversed() {
        return reversed;
    }

    /**
     * Replan this path based on the current robot position and speeds
     *
     * @param startingPose New starting pose for the replanned path
     * @param currentSpeeds Current chassis speeds of the robot
     * @return The replanned path
     */
    public PathPlannerPath replan(Pose2d startingPose, ChassisSpeeds currentSpeeds) {
        ChassisSpeeds currentFieldRelativeSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        currentSpeeds, startingPose.getRotation().unaryMinus());

        Translation2d robotNextControl = null;
        double linearVel =
                Math.hypot(
                        currentFieldRelativeSpeeds.vxMetersPerSecond,
                        currentFieldRelativeSpeeds.vyMetersPerSecond);
        if (linearVel > 0.1) {
            double stoppingDistance =
                    Math.pow(linearVel, 2) / (2 * globalConstraints.getMaxAccelerationMpsSq());

            Rotation2d heading =
                    new Rotation2d(
                            currentFieldRelativeSpeeds.vxMetersPerSecond,
                            currentFieldRelativeSpeeds.vyMetersPerSecond);
            robotNextControl =
                    startingPose.getTranslation().plus(new Translation2d(stoppingDistance, heading));
        }

        int closestPointIdx = 0;
        Translation2d comparePoint =
                (robotNextControl != null) ? robotNextControl : startingPose.getTranslation();
        double closestDist = positionDelta(comparePoint, getPoint(closestPointIdx).position);

        for (int i = 1; i < numPoints(); i++) {
            double d = positionDelta(comparePoint, getPoint(i).position);

            if (d < closestDist) {
                closestPointIdx = i;
                closestDist = d;
            }
        }

        if (closestPointIdx == numPoints() - 1) {
            Rotation2d heading = getPoint(numPoints() - 1).position.minus(comparePoint).getAngle();

            if (robotNextControl == null) {
                robotNextControl =
                        startingPose.getTranslation().plus(new Translation2d(closestDist / 3.0, heading));
            }

            Rotation2d endPrevControlHeading =
                    getPoint(numPoints() - 1).position.minus(robotNextControl).getAngle();

            Translation2d endPrevControl =
                    getPoint(numPoints() - 1)
                            .position
                            .minus(new Translation2d(closestDist / 3.0, endPrevControlHeading));

            // Throw out rotation targets, event markers, and constraint zones since we are skipping all
            // of the path
            return new PathPlannerPath(
                    Arrays.asList(
                            startingPose.getTranslation(),
                            robotNextControl,
                            endPrevControl,
                            getPoint(numPoints() - 1).position),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    globalConstraints,
                    goalEndState,
                    reversed,
                    previewStartingRotation);
        } else if ((closestPointIdx == 0 && robotNextControl == null)
                || (Math.abs(closestDist - startingPose.getTranslation().getDistance(getPoint(0).position))
                <= 0.25
                && linearVel < 0.1)) {
            double distToStart = startingPose.getTranslation().getDistance(getPoint(0).position);

            Rotation2d heading = getPoint(0).position.minus(startingPose.getTranslation()).getAngle();
            robotNextControl =
                    startingPose.getTranslation().plus(new Translation2d(distToStart / 3.0, heading));

            Rotation2d joinHeading = bezierPoints.get(0).minus(bezierPoints.get(1)).getAngle();
            Translation2d joinPrevControl =
                    getPoint(0).position.plus(new Translation2d(distToStart / 2.0, joinHeading));

            List<Translation2d> replannedBezier = new ArrayList<>();
            replannedBezier.addAll(
                    Arrays.asList(startingPose.getTranslation(), robotNextControl, joinPrevControl));
            replannedBezier.addAll(bezierPoints);

            // keep all rotations, markers, and zones and increment waypoint pos by 1
            return new PathPlannerPath(
                    replannedBezier,
                    rotationTargets.stream()
                            .map((target) -> new RotationTarget(target.getPosition() + 1, target.getTarget()))
                            .collect(Collectors.toList()),
                    constraintZones.stream()
                            .map(
                                    (zone) ->
                                            new ConstraintsZone(
                                                    zone.getMinWaypointPos() + 1,
                                                    zone.getMaxWaypointPos() + 1,
                                                    zone.getConstraints()))
                            .collect(Collectors.toList()),
                    globalConstraints,
                    goalEndState,
                    reversed,
                    previewStartingRotation);
        }

        int joinAnchorIdx = numPoints() - 1;
        for (int i = closestPointIdx; i < numPoints(); i++) {
            if (getPoint(i).distanceAlongPath
                    >= getPoint(closestPointIdx).distanceAlongPath + closestDist) {
                joinAnchorIdx = i;
                break;
            }
        }

        Translation2d joinPrevControl = getPoint(closestPointIdx).position;
        Translation2d joinAnchor = getPoint(joinAnchorIdx).position;

        if (robotNextControl == null) {
            double robotToJoinDelta = startingPose.getTranslation().getDistance(joinAnchor);
            Rotation2d heading = joinPrevControl.minus(startingPose.getTranslation()).getAngle();
            robotNextControl =
                    startingPose.getTranslation().plus(new Translation2d(robotToJoinDelta / 3.0, heading));
        }

        if (joinAnchorIdx == numPoints() - 1) {
            // Throw out rotation targets, event markers, and constraint zones since we are skipping all
            // of the path
            return new PathPlannerPath(
                    Arrays.asList(startingPose.getTranslation(), robotNextControl, joinPrevControl, joinAnchor),
                    Collections.emptyList(),
                    Collections.emptyList(),
                    globalConstraints,
                    goalEndState,
                    reversed,
                    previewStartingRotation);
        }

        int nextWaypointIdx = (int) Math.ceil((joinAnchorIdx + 1) * PathSegment.RESOLUTION);
        int bezierPointIdx = nextWaypointIdx * 3;
        double waypointDelta = joinAnchor.getDistance(bezierPoints.get(bezierPointIdx));

        Rotation2d joinHeading = joinAnchor.minus(joinPrevControl).getAngle();
        Translation2d joinNextControl =
                joinAnchor.plus(new Translation2d(waypointDelta / 3.0, joinHeading));

        Rotation2d nextWaypointHeading;
        if (bezierPointIdx == bezierPoints.size() - 1) {
            nextWaypointHeading =
                    bezierPoints.get(bezierPointIdx - 1).minus(bezierPoints.get(bezierPointIdx)).getAngle();
        } else {
            nextWaypointHeading =
                    bezierPoints.get(bezierPointIdx).minus(bezierPoints.get(bezierPointIdx + 1)).getAngle();
        }

        Translation2d nextWaypointPrevControl =
                bezierPoints
                        .get(bezierPointIdx)
                        .plus(new Translation2d(Math.max(waypointDelta / 3.0, 0.15), nextWaypointHeading));

        List<Translation2d> replannedBezier = new ArrayList<>();
        replannedBezier.addAll(
                Arrays.asList(
                        startingPose.getTranslation(),
                        robotNextControl,
                        joinPrevControl,
                        joinAnchor,
                        joinNextControl,
                        nextWaypointPrevControl));
        replannedBezier.addAll(bezierPoints.subList(bezierPointIdx, bezierPoints.size()));

        double segment1Length = 0;
        Translation2d lastSegment1Pos = startingPose.getTranslation();
        double segment2Length = 0;
        Translation2d lastSegment2Pos = joinAnchor;

        for (double t = PathSegment.RESOLUTION; t < 1.0; t += PathSegment.RESOLUTION) {
            Translation2d p1 =
                    GeometryUtil.cubicLerp(
                            startingPose.getTranslation(), robotNextControl, joinPrevControl, joinAnchor, t);
            Translation2d p2 =
                    GeometryUtil.cubicLerp(
                            joinAnchor,
                            joinNextControl,
                            nextWaypointPrevControl,
                            bezierPoints.get(bezierPointIdx),
                            t);

            segment1Length += lastSegment1Pos.getDistance(p1);
            segment2Length += lastSegment2Pos.getDistance(p2);

            lastSegment1Pos = p1;
            lastSegment2Pos = p2;
        }

        double segment1Pct = segment1Length / (segment1Length + segment2Length);

        List<RotationTarget> mappedTargets = new ArrayList<>();
        List<ConstraintsZone> mappedZones = new ArrayList<>();

        for (RotationTarget t : rotationTargets) {
            if (t.getPosition() >= nextWaypointIdx) {
                mappedTargets.add(new RotationTarget(t.getPosition() - nextWaypointIdx + 2, t.getTarget()));
            } else if (t.getPosition() >= nextWaypointIdx - 1) {
                double pct = t.getPosition() - (nextWaypointIdx - 1);
                mappedTargets.add(new RotationTarget(mapPct(pct, segment1Pct), t.getTarget()));
            }
        }

        for (ConstraintsZone z : constraintZones) {
            double minPos = 0;
            double maxPos = 0;

            if (z.getMinWaypointPos() >= nextWaypointIdx) {
                minPos = z.getMinWaypointPos() - nextWaypointIdx + 2;
            } else if (z.getMinWaypointPos() >= nextWaypointIdx - 1) {
                double pct = z.getMinWaypointPos() - (nextWaypointIdx - 1);
                minPos = mapPct(pct, segment1Pct);
            }

            if (z.getMaxWaypointPos() >= nextWaypointIdx) {
                maxPos = z.getMaxWaypointPos() - nextWaypointIdx + 2;
            } else if (z.getMaxWaypointPos() >= nextWaypointIdx - 1) {
                double pct = z.getMaxWaypointPos() - (nextWaypointIdx - 1);
                maxPos = mapPct(pct, segment1Pct);
            }

            if (maxPos > 0) {
                mappedZones.add(new ConstraintsZone(minPos, maxPos, z.getConstraints()));
            }
        }

        // Throw out everything before nextWaypointIdx - 1, map everything from nextWaypointIdx -
        // 1 to nextWaypointIdx on to the 2 joining segments (waypoint rel pos within old segment = %
        // along distance of both new segments)
        return new PathPlannerPath(
                replannedBezier,
                mappedTargets,
                mappedZones,
                globalConstraints,
                goalEndState,
                reversed,
                previewStartingRotation);
    }

    /**
     * Generate a trajectory for this path.
     *
     * @param startingSpeeds The robot-relative starting speeds.
     * @param startingRotation The starting rotation of the robot.
     * @return The generated trajectory.
     */
    public PathPlannerTrajectory getTrajectory(
            ChassisSpeeds startingSpeeds, Rotation2d startingRotation) {
        return new PathPlannerTrajectory(this, startingSpeeds, startingRotation);
    }

    /**
     * Flip a path to the other side of the field, maintaining a global blue alliance origin
     *
     * @return The flipped path
     */
    public PathPlannerPath flipPath() {
        List<Translation2d> newBezier =
                bezierPoints.stream().map(GeometryUtil::flipFieldPosition).collect(Collectors.toList());
        List<RotationTarget> newRotTargets =
                rotationTargets.stream()
                        .map(
                                (t) ->
                                        new RotationTarget(
                                                t.getPosition(),
                                                GeometryUtil.flipFieldRotation(t.getTarget()),
                                                t.shouldRotateFast()))
                        .collect(Collectors.toList());
        GoalEndState newEndState =
                new GoalEndState(
                        goalEndState.getVelocity(),
                        GeometryUtil.flipFieldRotation(goalEndState.getRotation()),
                        goalEndState.shouldRotateFast());
        Rotation2d newPreviewRot = GeometryUtil.flipFieldRotation(previewStartingRotation);

        return new PathPlannerPath(
                newBezier,
                newRotTargets,
                constraintZones,
                globalConstraints,
                newEndState,
                reversed,
                newPreviewRot);
    }

    /**
     * Get a list of poses representing every point in this path. This can be used to display a path
     * on a field 2d widget, for example.
     *
     * @return List of poses for each point in this path
     */
    public List<Pose2d> getPathPoses() {
        return allPoints.stream()
                .map(p -> new Pose2d(p.position, new Rotation2d()))
                .collect(Collectors.toList());
    }

    /**
     * Map a given percentage/waypoint relative position over 2 segments
     *
     * @param pct The percent to map
     * @param seg1Pct The percentage of the 2 segments made up by the first segment
     * @return The waypoint relative position over the 2 segments
     */
    private static double mapPct(double pct, double seg1Pct) {
        double mappedPct;
        if (pct <= seg1Pct) {
            // Map to segment 1
            mappedPct = pct / seg1Pct;
        } else {
            // Map to segment 2
            mappedPct = 1 + ((pct - seg1Pct) / (1.0 - seg1Pct));
        }

        // Round to nearest resolution step
        return Math.round(mappedPct * (1.0 / PathSegment.RESOLUTION)) / (1.0 / PathSegment.RESOLUTION);
    }

    private static double positionDelta(Translation2d a, Translation2d b) {
        Translation2d delta = a.minus(b);

        return Math.abs(delta.getX()) + Math.abs(delta.getY());
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        PathPlannerPath that = (PathPlannerPath) o;
        return Objects.equals(bezierPoints, that.bezierPoints)
                && Objects.equals(rotationTargets, that.rotationTargets)
                && Objects.equals(constraintZones, that.constraintZones)
                && Objects.equals(globalConstraints, that.globalConstraints)
                && Objects.equals(goalEndState, that.goalEndState);
    }

    @Override
    public int hashCode() {
        return Objects.hash(
                bezierPoints,
                rotationTargets,
                constraintZones,
                globalConstraints,
                goalEndState);
    }
}