package org.opentripplanner.routing.edgetype;

import org.opentripplanner.common.model.extras.OptionAttribute;
import org.opentripplanner.common.model.extras.OptionSet;
import org.opentripplanner.common.model.extras.nihOptions.NihOption;
import org.opentripplanner.common.model.extras.nihOptions.fields.CurbRamp;
import org.opentripplanner.common.model.extras.nihOptions.fields.Surface;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.core.StateEditor;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Vertex;
import org.opentripplanner.routing.util.ElevationUtils;
import org.opentripplanner.routing.vertextype.IntersectionVertex;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created by kathrynkillebrew on 1/23/15.
 */
public class StreetEdgeTraversal {

    private static Logger LOG = LoggerFactory.getLogger(StreetEdgeTraversal.class);

    private static final double GREENWAY_SAFETY_FACTOR = 0.1;

    /** return a StateEditor rather than a State so that we can make parking/mode switch modifications for kiss-and-ride. */
    public static StateEditor doTraverse(StreetEdge edge, State s0, RoutingRequest options, TraverseMode traverseMode) {
        boolean walkingBike = options.walkingBike;
        boolean backWalkingBike = s0.isBackWalkingBike();
        TraverseMode backMode = s0.getBackMode();
        Edge backEdge = s0.getBackEdge();
        if (backEdge != null) {
            // No illegal U-turns.
            // NOTE(flamholz): we check both directions because both edges get a chance to decide
            // if they are the reverse of the other. Also, because it doesn't matter which direction
            // we are searching in - these traversals are always disallowed (they are U-turns in one direction
            // or the other).
            // TODO profiling indicates that this is a hot spot.
            if (edge.isReverseOf(backEdge) || backEdge.isReverseOf(edge)) {
                return null;
            }
        }

        // Ensure we are actually walking, when walking a bike
        backWalkingBike &= TraverseMode.WALK.equals(backMode);
        walkingBike &= TraverseMode.WALK.equals(traverseMode);

        /* Check whether this street allows the current mode. If not and we are biking, attempt to walk the bike. */
        if (!edge.canTraverse(options, traverseMode)) {
            if (traverseMode == TraverseMode.BICYCLE) {
                return doTraverse(edge, s0, options.bikeWalkingOptions, TraverseMode.WALK);
            }
            return null;
        }

        // Automobiles have variable speeds depending on the edge type
        double speed = edge.calculateSpeed(options, traverseMode);

        double time = edge.getDistance() / speed;
        double weight;

        if (options.wheelchairAccessible) {
            weight = edge.getSlopeSpeedEffectiveLength() / speed;
        } else if (traverseMode.equals(TraverseMode.BICYCLE)) {
            time = edge.getSlopeSpeedEffectiveLength() / speed;
            switch (options.optimize) {
                case SAFE:
                    weight = edge.bicycleSafetyFactor * edge.getDistance() / speed;
                    break;
                case GREENWAYS:
                    weight = edge.bicycleSafetyFactor * edge.getDistance() / speed;
                    if (edge.bicycleSafetyFactor <= GREENWAY_SAFETY_FACTOR) {
                        // greenways are treated as even safer than they really are
                        weight *= 0.66;
                    }
                    break;
                case FLAT:
                /* see notes in StreetVertex on speed overhead */
                    weight = edge.getDistance() / speed + edge.getSlopeWorkCostEffectiveLength();
                    break;
                case QUICK:
                    weight = edge.getSlopeSpeedEffectiveLength() / speed;
                    break;
                case TRIANGLE:
                    double quick = edge.getSlopeSpeedEffectiveLength();
                    double safety = edge.bicycleSafetyFactor * edge.getDistance();
                    // TODO This computation is not coherent with the one for FLAT
                    double slope = edge.getSlopeWorkCostEffectiveLength();
                    weight = quick * options.triangleTimeFactor + slope
                            * options.triangleSlopeFactor + safety
                            * options.triangleSafetyFactor;
                    weight /= speed;
                    break;
                default:
                    weight = edge.getDistance() / speed;
            }
        } else {
            if (walkingBike) {
                // take slopes into account when walking bikes
                time = edge.getSlopeSpeedEffectiveLength() / speed;
            }
            weight = time;
            if (traverseMode.equals(TraverseMode.WALK)) {
                // take slopes into account when walking
                // FIXME: this causes steep stairs to be avoided. see #1297.
                double costs = ElevationUtils.getWalkCostsForSlope(edge.getDistance(), edge.getMaxSlope());
                // as the cost walkspeed is assumed to be for 4.8km/h (= 1.333 m/sec) we need to adjust
                // for the walkspeed set by the user
                double elevationUtilsSpeed = 4.0 / 3.0;
                weight = costs * (elevationUtilsSpeed / speed);

                time = weight; //treat cost as time, as in the current model it actually is the same (this can be checked for maxSlope == 0)
                /*
                // debug code
                if(weight > 100){
                    double timeflat = length / speed;
                    System.out.format("line length: %.1f m, slope: %.3f ---> slope costs: %.1f , weight: %.1f , time (flat):  %.1f %n", length, elevationProfile.getMaxSlope(), costs, weight, timeflat);
                }
                */
            }
        }

        // Ban edges if they match NihOptions and we're walking
        OptionSet extraOptions = edge.getExtraOptionFields();
        if (extraOptions != null && traverseMode.equals(TraverseMode.WALK)) {
            OptionAttribute surface = extraOptions.getOption(NihOption.SURFACE);
            OptionAttribute curbRamp = extraOptions.getOption(NihOption.CURB_RAMP);
            // If we're in a wheelchair, we need concrete and curb ramps
            if (options.wheelchairAccessible && (
                    !surface.equals(Surface.CONCRETE) ||
                    curbRamp.equals(CurbRamp.NO))) {
                return null;
            }
        }

        if (options.preferBenches && (edge.getBenchCount() > 0)) {
            LOG.info("Found edge {}: {} has a bench!  Weighting it.", edge.getName(), edge.getId());
            weight *= 0.001;
            time *= 0.001;
        }

        if (options.preferToilets && (edge.getToiletCount() > 0)) {
            LOG.info("Found edge {}: {} has a toilet!  Weighting it.", edge.getName(), edge.getId());
            weight *= 0.001;
            time *= 0.001;
        }

        if (edge.isStairs()) {
            weight *= options.stairsReluctance;
        } else {
            // TODO: this is being applied even when biking or driving.
            weight *= options.walkReluctance;
        }

        StateEditor s1 = s0.edit(edge);
        s1.setBackMode(traverseMode);
        s1.setBackWalkingBike(walkingBike);

        /* Compute turn cost. */
        StreetEdge backPSE;
        if (backEdge != null && backEdge instanceof StreetEdge) {
            backPSE = (StreetEdge) backEdge;
            RoutingRequest backOptions = backWalkingBike ?
                    s0.getOptions().bikeWalkingOptions : s0.getOptions();
            double backSpeed = backPSE.calculateSpeed(backOptions, backMode);
            double realTurnCost;  // Units are seconds.

            // Apply turn restrictions
            if (options.arriveBy && !edge.canTurnOnto(backPSE, s0, backMode)) {
                return null;
            } else if (!options.arriveBy && !backPSE.canTurnOnto(edge, s0, traverseMode)) {
                return null;
            }

            /*
             * This is a subtle piece of code. Turn costs are evaluated differently during
             * forward and reverse traversal. During forward traversal of an edge, the turn
             * *into* that edge is used, while during reverse traversal, the turn *out of*
             * the edge is used.
             *
             * However, over a set of edges, the turn costs must add up the same (for
             * general correctness and specifically for reverse optimization). This means
             * that during reverse traversal, we must also use the speed for the mode of
             * the backEdge, rather than of the current edge.
             */

            Vertex fromv = edge.getFromVertex();
            Vertex tov = edge.getToVertex();

            if (options.arriveBy && tov instanceof IntersectionVertex) { // arrive-by search
                IntersectionVertex traversedVertex = ((IntersectionVertex) tov);

                realTurnCost = backOptions.getIntersectionTraversalCostModel().computeTraversalCost(
                        traversedVertex, edge, backPSE, backMode, backOptions, (float) speed,
                        (float) backSpeed);
            } else if (!options.arriveBy && fromv instanceof IntersectionVertex) { // depart-after search
                IntersectionVertex traversedVertex = ((IntersectionVertex) fromv);

                realTurnCost = options.getIntersectionTraversalCostModel().computeTraversalCost(
                        traversedVertex, backPSE, edge, traverseMode, options, (float) backSpeed,
                        (float) speed);
            } else {
                // In case this is a temporary edge not connected to an IntersectionVertex
                LOG.debug("Not computing turn cost for edge {}", edge);
                realTurnCost = 0;
            }

            if (options.preferBenches && (edge.getBenchCount() > 0)) {
                LOG.info("Found edge {}: {} has a bench!  Changing its turn cost.", edge.getName(), edge.getId());
                realTurnCost *= 0.001;
            }

            if (options.preferToilets && (edge.getToiletCount() > 0)) {
                LOG.info("Found edge {}: {} has a toilet!  Changing its turn cost.", edge.getName(), edge.getId());
                realTurnCost *= 0.001;
            }

            if (!traverseMode.isDriving()) {
                s1.incrementWalkDistance(realTurnCost / 100);  // just a tie-breaker
            }

            long turnTime = (long) Math.ceil(realTurnCost);
            time += turnTime;
            weight += options.turnReluctance * realTurnCost;
        }


        if (walkingBike || TraverseMode.BICYCLE.equals(traverseMode)) {
            if (!(backWalkingBike || TraverseMode.BICYCLE.equals(backMode))) {
                s1.incrementTimeInSeconds(options.bikeSwitchTime);
                s1.incrementWeight(options.bikeSwitchCost);
            }
        }

        if (!traverseMode.isDriving()) {
            s1.incrementWalkDistance(edge.getDistance());
        }

        // accumulate feature counts
        s1.incrementBenchCount(edge.getBenchCount());
        s1.incrementToiletCount(edge.getToiletCount());

        /* On the pre-kiss/pre-park leg, limit both walking and driving, either soft or hard. */
        int roundedTime = (int) Math.ceil(time);
        if (options.kissAndRide || options.parkAndRide) {
            if (options.arriveBy) {
                if (!s0.isCarParked()) s1.incrementPreTransitTime(roundedTime);
            } else {
                if (!s0.isEverBoarded()) s1.incrementPreTransitTime(roundedTime);
            }
            if (s1.isMaxPreTransitTimeExceeded(options)) {
                if (options.softPreTransitLimiting) {
                    weight += calculateOverageWeight(s0.getPreTransitTime(), s1.getPreTransitTime(),
                            options.maxPreTransitTime, options.preTransitPenalty,
                            options.preTransitOverageRate);
                } else return null;
            }
        }

        /* Apply a strategy for avoiding walking too far, either soft (weight increases) or hard limiting (pruning). */
        if (s1.weHaveWalkedTooFar(options)) {

            // if we're using a soft walk-limit
            if( options.softWalkLimiting ){
                // just slap a penalty for the overage onto s1
                weight += calculateOverageWeight(s0.getWalkDistance(), s1.getWalkDistance(),
                        options.getMaxWalkDistance(), options.softWalkPenalty,
                        options.softWalkOverageRate);
            } else {
                // else, it's a hard limit; bail
                LOG.debug("Too much walking. Bailing.");
                return null;
            }
        }

        s1.incrementTimeInSeconds(roundedTime);
        s1.incrementWeight(weight);

        return s1;
    }

    private static double calculateOverageWeight(double firstValue, double secondValue, double maxValue,
                                          double softPenalty, double overageRate) {
        // apply penalty if we stepped over the limit on this traversal
        boolean applyPenalty = false;
        double overageValue;

        if(firstValue <= maxValue && secondValue > maxValue){
            applyPenalty = true;
            overageValue = secondValue - maxValue;
        } else {
            overageValue = secondValue - firstValue;
        }

        // apply overage and add penalty if necessary
        return (overageRate * overageValue) + (applyPenalty ? softPenalty : 0.0);
    }
}
