/* This program is free software: you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public License
   as published by the Free Software Foundation, either version 3 of
   the License, or (at your option) any later version.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 */

package org.opentripplanner.api.model;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.List;

import javax.xml.bind.annotation.XmlElement;
import javax.xml.bind.annotation.XmlTransient;
import javax.xml.bind.annotation.adapters.XmlJavaTypeAdapter;

import org.opentripplanner.common.model.P2;
import org.opentripplanner.routing.alertpatch.Alert;

import com.fasterxml.jackson.databind.annotation.JsonSerialize;
import org.opentripplanner.util.model.EncodedPolylineBean;

/**
 * Represents one instruction in walking directions. Three examples from New York City:
 * <p>
 * Turn onto Broadway from W 57th St (coming from 7th Ave): <br/>
 * distance = 100 (say) <br/>
 * walkDirection = RIGHT <br/>
 * streetName = Broadway <br/>
 * everything else null/false <br/>
 * </p>
 * <p>
 * Now, turn from Broadway onto Central Park S via Columbus Circle <br/>
 * distance = 200 (say) <br/>
 * walkDirection = CIRCLE_COUNTERCLOCKWISE <br/>
 * streetName = Central Park S <br/>
 * exit = 1 (first exit) <br/>
 * immediately everything else false <br/>
 * </p>
 * <p>
 * Instead, go through the circle to continue on Broadway <br/>
 * distance = 100 (say) <br/>
 * walkDirection = CIRCLE_COUNTERCLOCKWISE <br/>
 * streetName = Broadway <br/>
 * exit = 3 <br/>
 * stayOn = true <br/>
 * everything else false <br/>
 * </p>
 * */
public class WalkStep {

    /**
     * The distance in meters that this step takes.
     */
    public double distance = 0;

    /**
     * The relative direction of this step.
     */
    public RelativeDirection relativeDirection;

    /**
     * The name of the street.
     */
    public String streetName;

    /**
     * The absolute direction of this step.
     */
    public AbsoluteDirection absoluteDirection;

    /**
     * When exiting a highway or traffic circle, the exit name/number.
     */

    public String exit;

    /**
     * Indicates whether or not a street changes direction at an intersection.
     */
    public Boolean stayOn = false;

    /**
     * This step is on an open area, such as a plaza or train platform, and thus the directions should say something like "cross"
     */
    public Boolean area = false;

    /**
     * The name of this street was generated by the system, so we should only display it once, and generally just display right/left directions
     */
    public Boolean bogusName = false;

    /**
     * The longitude of start of the step
     */
    public double lon;

    /**
     * The latitude of start of the step
     */
    public double lat;

    /**
     * Counts of features passed.
     */
    public int benches;
    public int toilets;

    /**
     * NIH features/hazards passed.
     */
    public String rest = "";
    public boolean unevenSurfaces = false;
    public boolean aesthetics = false;
    public float maxSlope = 0;

    // individual step geometry
    public EncodedPolylineBean stepGeometry;
    /////////////////////////////////////

    /**
     * Time the step was last audited, will serialize to an epoch in milliseconds
     * If null, will be blank
     */
    public Date lastAudited;

    /**
     * The elevation profile as a comma-separated list of x,y values. x is the distance from the start of the step, y is the elevation at this
     * distance.
     */
    @XmlTransient
    public List<P2<Double>> elevation;

    @XmlElement
    public List<Alert> alerts;

    public transient double angle;

    public void setDirections(double lastAngle, double thisAngle, boolean roundabout) {
        relativeDirection = getRelativeDirection(lastAngle, thisAngle, roundabout);
        setAbsoluteDirection(thisAngle);
    }

    public String toString() {
        String direction = absoluteDirection.toString();
        if (relativeDirection != null) {
            direction = relativeDirection.toString();
        }
        String str = "WalkStep(" + direction + " on " + streetName + " for " + distance;
        if (benches > 0) {
            str += " benches=" + benches;
        }
        if (toilets > 0) {
            str += " toilets=" + toilets;
        }
        // NIH flags.  TODO: change strings?
        if (aesthetics) {
            str += " aesthetics";
        }
        if (rest.length() > 0) {
            str += " resting place: " + rest;
        }
        if (unevenSurfaces) {
            str += " uneven surface";
        }
        if (maxSlope != 0) {
            str += "max slope: " + maxSlope;
        }
        ////////////////////////////////////
        return str += ")";
    }

    public static RelativeDirection getRelativeDirection(double lastAngle, double thisAngle,
            boolean roundabout) {

        double angleDiff = thisAngle - lastAngle;
        if (angleDiff < 0) {
            angleDiff += Math.PI * 2;
        }
        double ccwAngleDiff = Math.PI * 2 - angleDiff;

        if (roundabout) {
            // roundabout: the direction we turn onto it implies the circling direction
            if (angleDiff > ccwAngleDiff) {
                return RelativeDirection.CIRCLE_CLOCKWISE;
            } else {
                return RelativeDirection.CIRCLE_COUNTERCLOCKWISE;
            }
        }

        // less than 0.3 rad counts as straight, to simplify walking instructions
        if (angleDiff < 0.3 || ccwAngleDiff < 0.3) {
            return RelativeDirection.CONTINUE;
        } else if (angleDiff < 0.7) {
            return RelativeDirection.SLIGHTLY_RIGHT;
        } else if (ccwAngleDiff < 0.7) {
            return RelativeDirection.SLIGHTLY_LEFT;
        } else if (angleDiff < 2) {
            return RelativeDirection.RIGHT;
        } else if (ccwAngleDiff < 2) {
            return RelativeDirection.LEFT;
        } else if (angleDiff < Math.PI) {
            return RelativeDirection.HARD_RIGHT;
        } else {
            return RelativeDirection.HARD_LEFT;
        }
    }

    public void setAbsoluteDirection(double thisAngle) {
        int octant = (int) (8 + Math.round(thisAngle * 8 / (Math.PI * 2))) % 8;
        absoluteDirection = AbsoluteDirection.values()[octant];
    }

    public void addAlerts(Collection<Alert> newAlerts) {
        if (newAlerts == null) {
            return;
        }
        if (alerts == null) {
            alerts = new ArrayList<Alert>(newAlerts);
            return;
        }
        for (Alert alert : newAlerts) {
            if (!alerts.contains(alert)) {
                alerts.add(alert);
            }
        }
    }

    public String streetNameNoParens() {
        int idx = streetName.indexOf('(');
        if (idx <= 0) {
            return streetName;
        }
        return streetName.substring(0, idx - 1);
    }

    @XmlJavaTypeAdapter(ElevationAdapter.class)
    @JsonSerialize
    public List<P2<Double>> getElevation() {
        return elevation;
    }

}
