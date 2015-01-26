/* This program is free software: you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public License
 as published by the Free Software Foundation, either version 3 of
 the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. */

package org.opentripplanner.routing.edgetype;

import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.Arrays;

import org.opentripplanner.common.TurnRestriction;
import org.opentripplanner.common.TurnRestrictionType;
import org.opentripplanner.common.geometry.CompactLineString;
import org.opentripplanner.common.geometry.DirectionUtils;
import org.opentripplanner.common.geometry.PackedCoordinateSequence;
import org.opentripplanner.common.model.extras.NumericFieldSet;
import org.opentripplanner.common.model.extras.OptionSet;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.core.State;
import org.opentripplanner.routing.core.StateEditor;
import org.opentripplanner.routing.core.TraverseMode;
import org.opentripplanner.routing.core.TraverseModeSet;
import org.opentripplanner.routing.graph.Edge;
import org.opentripplanner.routing.graph.Graph;
import org.opentripplanner.routing.vertextype.StreetVertex;
import org.opentripplanner.util.BitSetUtils;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.LineString;

/**
 * This represents a street segment.
 *
 * @author novalis
 *
 */
public class StreetEdge extends Edge implements Cloneable {

    private static Logger LOG = LoggerFactory.getLogger(StreetEdge.class);

    private static final long serialVersionUID = 1L;

    /* TODO combine these with OSM highway= flags? */
    public static final int CLASS_STREET = 3;
    public static final int CLASS_CROSSING = 4;
    public static final int CLASS_OTHERPATH = 5;
    public static final int CLASS_OTHER_PLATFORM = 8;
    public static final int CLASS_TRAIN_PLATFORM = 16;
    public static final int ANY_PLATFORM_MASK = 24;
    public static final int CROSSING_CLASS_MASK = 7; // ignore platform
    public static final int CLASS_LINK = 32; // on/offramps; OSM calls them "links"

    // TODO(flamholz): do something smarter with the car speed here.
    public static final float DEFAULT_CAR_SPEED = 11.2f;

    /** If you have more than 8 flags, increase flags to short or int */
    private static final int BACK_FLAG_INDEX = 0;
    private static final int ROUNDABOUT_FLAG_INDEX = 1;
    private static final int HASBOGUSNAME_FLAG_INDEX = 2;
    private static final int NOTHRUTRAFFIC_FLAG_INDEX = 3;
    private static final int STAIRS_FLAG_INDEX = 4;
    private static final int SLOPEOVERRIDE_FLAG_INDEX = 5;
    private static final int WHEELCHAIR_ACCESSIBLE_FLAG_INDEX = 6;

    ///////////////////////////////////////////////////////////////////////////////////
    /* Counts of features on this edge */
    private int benchCount = 0;
    private int toiletCount = 0;

    // optional extra features for weighting
    private OptionSet extraOptionFields;

    private NumericFieldSet extraNumericFields;

    public OptionSet getExtraOptionFields() {
        return extraOptionFields;
    }

    public void setExtraOptionFields(OptionSet extraOptionFields) {
        this.extraOptionFields = extraOptionFields;
    }

    public NumericFieldSet getExtraNumericFields() {
        return extraNumericFields;
    }

    public void setExtraNumericFields(NumericFieldSet extraNumericFields) {
        this.extraNumericFields = extraNumericFields;
    }
    /////////////////////////////////////////////////////////////////////////////////

    /** back, roundabout, stairs, ... */
    private byte flags;

    /**
     * Length is stored internally as 32-bit fixed-point (millimeters). This allows edges of up to ~2100km.
     * Distances used in calculations and exposed outside this class are still in double-precision floating point meters.
     * Someday we might want to convert everything to fixed point representations.
     */
    private int length_mm;

    /**
     * bicycleSafetyWeight = length * bicycleSafetyFactor. For example, a 100m street with a safety
     * factor of 2.0 will be considered in term of safety cost as the same as a 150m street with a
     * safety factor of 1.0.
     */
    protected float bicycleSafetyFactor;

    private int[] compactGeometry;

    private String name;

    /* osm:way:<id> label of OpenStreetMap way ID that this segment lies along */
    private String osmId;

    private StreetTraversalPermission permission;

    private int streetClass = CLASS_OTHERPATH;

    /**
     * The speed (meters / sec) at which an automobile can traverse
     * this street segment.
     */
    private float carSpeed;

    /**
     * The angle at the start of the edge geometry.
     * Internal representation is -180 to +179 integer degrees mapped to -128 to +127 (brads)
     */
    private byte inAngle;

    /** The angle at the start of the edge geometry. Internal representation like that of inAngle. */
    private byte outAngle;

    /* Construct an edge without an OSM ID */
    public StreetEdge(StreetVertex v1, StreetVertex v2, LineString geometry,
                      String name, double length, StreetTraversalPermission permission,
                      boolean back) {
        this(v1, v2, geometry, name, length, permission, back, "");
    }

    public StreetEdge(StreetVertex v1, StreetVertex v2, LineString geometry,
                      String name, double length, StreetTraversalPermission permission,
                      boolean back, String osmId) {
        super(v1, v2);
        this.setBack(back);
        this.setGeometry(geometry);
        this.length_mm = (int) (length * 1000); // CONVERT FROM FLOAT METERS TO FIXED MILLIMETERS
        this.bicycleSafetyFactor = 1.0f;
        this.name = name;
        this.setPermission(permission);
        this.osmId = osmId;
        this.setCarSpeed(DEFAULT_CAR_SPEED);
        this.setWheelchairAccessible(true); // accessible by default
        if (geometry != null) {
            try {
                for (Coordinate c : geometry.getCoordinates()) {
                    if (Double.isNaN(c.x)) {
                        System.out.println("X DOOM");
                    }
                    if (Double.isNaN(c.y)) {
                        System.out.println("Y DOOM");
                    }
                }
                // Conversion from radians to internal representation as a single signed byte.
                // We also reorient the angles since OTP seems to use South as a reference
                // while the azimuth functions use North.
                // FIXME Use only North as a reference, not a mix of North and South!
                // Range restriction happens automatically due to Java signed overflow behavior.
                // 180 degrees exists as a negative rather than a positive due to the integer range.
                double angleRadians = DirectionUtils.getLastAngle(geometry);
                outAngle = (byte) Math.round(angleRadians * 128 / Math.PI + 128);
                angleRadians = DirectionUtils.getFirstAngle(geometry);
                inAngle = (byte) Math.round(angleRadians * 128 / Math.PI + 128);
            } catch (IllegalArgumentException iae) {
                LOG.error("exception while determining street edge angles. setting to zero. there is probably something wrong with this street segment's geometry.");
                inAngle = 0;
                outAngle = 0;
            }
        }
    }

    public boolean canTraverse(RoutingRequest options) {
        if (options.wheelchairAccessible) {
            if (!isWheelchairAccessible()) {
                return false;
            }
            if (getMaxSlope() > options.maxSlope) {
                return false;
            }
        }

        return canTraverse(options.modes);
    }

    public boolean canTraverse(TraverseModeSet modes) {
        return getPermission().allows(modes);
    }

    public boolean canTraverse(RoutingRequest options, TraverseMode mode) {
        if (options.wheelchairAccessible) {
            if (!isWheelchairAccessible()) {
                return false;
            }
            if (getMaxSlope() > options.maxSlope) {
                return false;
            }
        }
        return getPermission().allows(mode);
    }

    public PackedCoordinateSequence getElevationProfile() {
        return null;
    }

    public boolean isElevationFlattened() {
        return false;
    }

    public float getMaxSlope() {
        return 0.0f;
    }

    @Override
    public double getDistance() {
        return length_mm / 1000.0; // CONVERT FROM FIXED MILLIMETERS TO FLOAT METERS
    }

    @Override
    public State traverse(State s0) {
        final RoutingRequest options = s0.getOptions();
        final TraverseMode currMode = s0.getNonTransitMode();
        StateEditor editor = StreetEdgeTraversal.doTraverse(this, s0, options, s0.getNonTransitMode());
        State state = (editor == null) ? null : editor.makeState();
        /* Kiss and ride support. Mode transitions occur without the explicit loop edges used in park-and-ride. */
        if (options.kissAndRide) {
            if (options.arriveBy) {
                // Branch search to "unparked" CAR mode ASAP after transit has been used.
                // Final WALK check prevents infinite recursion.
                if (s0.isCarParked() && s0.isEverBoarded() && currMode == TraverseMode.WALK) {
                    editor = StreetEdgeTraversal.doTraverse(this, s0, options, TraverseMode.CAR);
                    if (editor != null) {
                        editor.setCarParked(false); // Also has the effect of switching to CAR
                        State forkState = editor.makeState();
                        if (forkState != null) {
                            forkState.addToExistingResultChain(state);
                            return forkState; // return both parked and unparked states
                        }
                    }
                }
            } else { /* departAfter */
                // Irrevocable transition from driving to walking. "Parking" means being dropped off in this case.
                // Final CAR check needed to prevent infinite recursion.
                if ( ! s0.isCarParked() && ! getPermission().allows(TraverseMode.CAR) && currMode == TraverseMode.CAR) {
                    editor = StreetEdgeTraversal.doTraverse(this, s0, options, TraverseMode.WALK);
                    if (editor != null) {
                        editor.setCarParked(true); // has the effect of switching to WALK and preventing further car use
                        return editor.makeState(); // return only the "parked" walking state
                    }

                }
            }
        }
        return state;
    }

    /**
     * Calculate the average automobile traversal speed of this segment, given
     * the RoutingRequest, and return it in meters per second.
     */
    private double calculateCarSpeed(RoutingRequest options) {
        return getCarSpeed();
    }

    /**
     * Calculate the speed appropriately given the RoutingRequest and traverseMode.
     */
    public double calculateSpeed(RoutingRequest options, TraverseMode traverseMode) {
        if (traverseMode == null) {
            return Double.NaN;
        } else if (traverseMode.isDriving()) {
            // NOTE: Automobiles have variable speeds depending on the edge type
            return calculateCarSpeed(options);
        }
        return options.getSpeed(traverseMode);
    }

    @Override
    public double weightLowerBound(RoutingRequest options) {
        return timeLowerBound(options) * options.walkReluctance;
    }

    @Override
    public double timeLowerBound(RoutingRequest options) {
        return this.getDistance() / options.getStreetSpeedUpperBound();
    }

    public double getSlopeSpeedEffectiveLength() {
        return getDistance();
    }

    public double getSlopeWorkCostEffectiveLength() {
        return getDistance();
    }

    public void setBicycleSafetyFactor(float bicycleSafetyFactor) {
        this.bicycleSafetyFactor = bicycleSafetyFactor;
    }

    public float getBicycleSafetyFactor() {
        return bicycleSafetyFactor;
    }

    public String getOsmId() {
        return osmId;
    }

    private void writeObject(ObjectOutputStream out) throws IOException {
        out.defaultWriteObject();
    }

    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("PlainStreetEdge(");
        sb.append(getId());
        sb.append(", ");
        sb.append(name);
        sb.append(", ");
        sb.append(getOsmId());
        sb.append(", ");
        sb.append(fromv);
        sb.append(" -> ");
        sb.append(tov);
        sb.append(" length=");
        sb.append(this.getDistance());
        // TODO: move/remove bench/toilet counters
        //sb.append(" benches=");
        //sb.append(benchCount);
        //sb.append(" toilets=");
        //sb.append(toiletCount);
        sb.append(" permission=");
        sb.append(this.getPermission());
        if (extraOptionFields != null) {
            sb.append(extraOptionFields.toString());
        }
        if (extraNumericFields != null) {
            sb.append(extraNumericFields.toString());
        }
        sb.append(")");
        return sb.toString();
    }

    @Override
    public StreetEdge clone() {
        try {
            StreetEdge clone = (StreetEdge) super.clone();
            // add fields specific to StreetEdge
            // TODO: why aren't the other, pre-existing fields being added here?
            clone.osmId = this.osmId;
            clone.benchCount = this.benchCount;
            clone.toiletCount = this.toiletCount;
            clone.extraOptionFields = this.extraOptionFields;
            clone.extraNumericFields = this.extraNumericFields;
            return clone;
        } catch (CloneNotSupportedException e) {
            throw new RuntimeException(e);
        }
    }

    public boolean canTurnOnto(Edge e, State state, TraverseMode mode) {
        Graph graph = state.getOptions().rctx.graph;
        for (TurnRestriction restriction : graph.getTurnRestrictions(this)) {
            /* FIXME: This is wrong for trips that end in the middle of restriction.to
             */

            // NOTE(flamholz): edge to be traversed decides equivalence. This is important since
            // it might be a temporary edge that is equivalent to some graph edge.
            if (restriction.type == TurnRestrictionType.ONLY_TURN) {
                if (!e.isEquivalentTo(restriction.to) && restriction.modes.contains(mode) &&
                        restriction.active(state.getTimeSeconds())) {
                    return false;
                }
            } else {
                if (e.isEquivalentTo(restriction.to) && restriction.modes.contains(mode) &&
                        restriction.active(state.getTimeSeconds())) {
                    return false;
                }
            }
        }
        return true;
    }

    protected boolean detachFrom(Graph graph) {
        if (fromv != null) {
            for (Edge e : fromv.getIncoming()) {
                if (e instanceof StreetEdge) {
                    for (TurnRestriction restriction : graph.getTurnRestrictions(e)) {
                        if (restriction.to == this) {
                            graph.removeTurnRestriction(e, restriction);
                        }
                    }
                }
            }
        }
        return super.detachFrom(graph);
    }

	@Override
	public String getName() {
		return this.name;
	}

	public void setName(String name) {
		this.name = name;
	}

	public LineString getGeometry() {
		return CompactLineString.uncompactLineString(fromv.getLon(), fromv.getLat(), tov.getLon(), tov.getLat(), compactGeometry, isBack());
	}

	private void setGeometry(LineString geometry) {
		this.compactGeometry = CompactLineString.compactLineString(fromv.getLon(), fromv.getLat(), tov.getLon(), tov.getLat(), isBack() ? (LineString)geometry.reverse() : geometry, isBack());
	}

	public void shareData(StreetEdge reversedEdge) {
	    if (Arrays.equals(compactGeometry, reversedEdge.compactGeometry)) {
	        compactGeometry = reversedEdge.compactGeometry;
	    } else {
	        LOG.warn("Can't share geometry between {} and {}", this, reversedEdge);
	    }
	}

	public boolean isWheelchairAccessible() {
		return BitSetUtils.get(flags, WHEELCHAIR_ACCESSIBLE_FLAG_INDEX);
	}

	public void setWheelchairAccessible(boolean wheelchairAccessible) {
        flags = BitSetUtils.set(flags, WHEELCHAIR_ACCESSIBLE_FLAG_INDEX, wheelchairAccessible);
	}

	public StreetTraversalPermission getPermission() {
		return permission;
	}

	public void setPermission(StreetTraversalPermission permission) {
		this.permission = permission;
	}

	public int getStreetClass() {
		return streetClass;
	}

	public void setStreetClass(int streetClass) {
		this.streetClass = streetClass;
	}

	/**
	 * Marks that this edge is the reverse of the one defined in the source
	 * data. Does NOT mean fromv/tov are reversed.
	 */
	public boolean isBack() {
	    return BitSetUtils.get(flags, BACK_FLAG_INDEX);
	}

	public void setBack(boolean back) {
            flags = BitSetUtils.set(flags, BACK_FLAG_INDEX, back);
	}

	public boolean isRoundabout() {
            return BitSetUtils.get(flags, ROUNDABOUT_FLAG_INDEX);
	}

	public void setRoundabout(boolean roundabout) {
	    flags = BitSetUtils.set(flags, ROUNDABOUT_FLAG_INDEX, roundabout);
	}

	public boolean hasBogusName() {
	    return BitSetUtils.get(flags, HASBOGUSNAME_FLAG_INDEX);
	}

	public void setHasBogusName(boolean hasBogusName) {
	    flags = BitSetUtils.set(flags, HASBOGUSNAME_FLAG_INDEX, hasBogusName);
	}

	public boolean isNoThruTraffic() {
            return BitSetUtils.get(flags, NOTHRUTRAFFIC_FLAG_INDEX);
	}

	public void setNoThruTraffic(boolean noThruTraffic) {
	    flags = BitSetUtils.set(flags, NOTHRUTRAFFIC_FLAG_INDEX, noThruTraffic);
	}

    public int getBenchCount() {
         return benchCount;
    }

    public void addBench() {
        benchCount += 1;
    }

    public void setBenchCount(int count) {
        benchCount = count;
    }

    public int getToiletCount() {
        return toiletCount;
    }

    public void addToilet() {
        toiletCount += 1;
    }

    public void setToiletCount(int count) {
        toiletCount = count;
    }

	/**
	 * This street is a staircase
	 */
	public boolean isStairs() {
            return BitSetUtils.get(flags, STAIRS_FLAG_INDEX);
	}

	public void setStairs(boolean stairs) {
	    flags = BitSetUtils.set(flags, STAIRS_FLAG_INDEX, stairs);
	}

	public float getCarSpeed() {
		return carSpeed;
	}

	public void setCarSpeed(float carSpeed) {
		this.carSpeed = carSpeed;
	}

	public boolean isSlopeOverride() {
	    return BitSetUtils.get(flags, SLOPEOVERRIDE_FLAG_INDEX);
	}

	public void setSlopeOverride(boolean slopeOverride) {
	    flags = BitSetUtils.set(flags, SLOPEOVERRIDE_FLAG_INDEX, slopeOverride);
	}

    /**
     * Return the azimuth of the first segment in this edge in integer degrees clockwise from South.
     * TODO change everything to clockwise from North
     */
	public int getInAngle() {
		return this.inAngle * 180 / 128;
	}

    /** Return the azimuth of the last segment in this edge in integer degrees clockwise from South. */
	public int getOutAngle() {
		return this.outAngle * 180 / 128;
	}

}
