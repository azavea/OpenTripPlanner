package org.opentripplanner.analyst.request;

import java.awt.AlphaComposite;
import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.OutputStream;
import java.text.DateFormat;
import java.util.Date;
import java.util.TimeZone;

import javax.imageio.ImageIO;
import javax.ws.rs.WebApplicationException;
import javax.ws.rs.core.CacheControl;
import javax.ws.rs.core.Response;
import javax.ws.rs.core.StreamingOutput;

import org.geotools.coverage.grid.GridCoverage2D;
import org.geotools.coverage.grid.io.AbstractGridFormat;
import org.geotools.gce.geotiff.GeoTiffFormat;
import org.geotools.gce.geotiff.GeoTiffWriteParams;
import org.geotools.gce.geotiff.GeoTiffWriter;
import org.opengis.parameter.GeneralParameterValue;
import org.opengis.parameter.ParameterValueGroup;
import org.opentripplanner.analyst.TimeSurface;
import org.opentripplanner.analyst.core.Tile;
import org.opentripplanner.api.parameter.MIMEImageFormat;
import org.opentripplanner.routing.core.RoutingRequest;
import org.opentripplanner.routing.spt.ShortestPathTree;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Renderer {

    private static final Logger LOG = LoggerFactory.getLogger(Renderer.class);

    private TileCache tileCache;

    private static final DateFormat df = DateFormat.getDateTimeInstance();

    public Renderer(TileCache tileCache) {
        this.tileCache = tileCache;
    }

    public Response getResponse (
            TileRequest tileRequest,
            TimeSurface surfA, TimeSurface surfB,
            RenderRequest renderRequest) throws Exception {

        Tile tile = tileCache.get(tileRequest);
        BufferedImage image;
        switch (renderRequest.layer) {
        case TRAVELTIME :
            image = tile.generateImage(surfA, renderRequest);
            break;
        case DIFFERENCE :
            image = tile.linearCombination(1, surfA, -1, surfB, 0, renderRequest);
            break;
        case HAGERSTRAND :
            long elapsed = Math.abs(surfB.dateTime - surfA.dateTime);
            image = tile.linearCombination(-1, surfA, -1, surfB, elapsed/60, renderRequest);
            break;
        default :
            image = tile.generateImage(surfA, renderRequest);
        }
        
        // add a timestamp to the image if requested. 
        // of course this will make it useless as a raster for analysis, but it's good for animations.
        if (renderRequest.timestamp) {
            df.setTimeZone(TimeZone.getTimeZone("America/New_York"));
            String ds = df.format(new Date(surfA.dateTime * 1000));
            shadowWrite(image, ds, String.format("%f, %f", surfA.lat, surfA.lon));
            Graphics2D g2d = image.createGraphics();
            g2d.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC));
            BufferedImage legend = Tile.getLegend(renderRequest.style, 300, 50);
            g2d.drawImage(legend, 0, image.getHeight()-50, null);
            g2d.dispose();
        }
                

        if (!renderRequest.getGeotiff) {
            return generateStreamingImageResponse(image, renderRequest.format);
        } else {
            // get geotiff
            GridCoverage2D gc = tile.getGridCoverage2D(image);
            return generateStreamingGeotiffResponse(gc);
        }
    }
    
    private void shadowWrite(BufferedImage image, String... strings) {
        Graphics2D g2d = image.createGraphics();
        g2d.setFont(new Font("Sans", Font.PLAIN, 25));
        FontMetrics fm = g2d.getFontMetrics();
        int dy = fm.getHeight();
        int xsize = 0;
        for (String s : strings) {
            int w = fm.stringWidth(s);
            if (w > xsize)
                xsize = w;
        }
        int y = 5;
        int x = 5;
        //g2d.fillRect(x, y, xsize, dy * strings.length + fm.getDescent());
        y += dy;
        for (String s : strings) {
            g2d.setPaint(Color.black);
            g2d.drawString(s, x+1, y+1);
            g2d.setPaint(Color.white);
            g2d.drawString(s, x, y);
            y += dy;
        }
        g2d.dispose();
    }
        
    public static Response generateStreamingImageResponse(
            final BufferedImage image, final MIMEImageFormat format) {
        
        if (image == null) {
            LOG.warn("response image is null");
        }
            
        StreamingOutput streamingOutput = new StreamingOutput() {
            public void write(OutputStream outStream) {
                try {
                    ImageIO.write(image, format.type, outStream);
                } catch (Exception e) {
                    LOG.error("exception while preparing image : {}", e.getMessage());
                    throw new WebApplicationException(e);
                }
            }
       };

       CacheControl cc = new CacheControl();
       cc.setMaxAge(3600);
       cc.setNoCache(false);
       return Response.ok(streamingOutput)
                       .type(format.toString())
                       .cacheControl(cc)
                       .build();
    }
    
    
    private static Response generateStreamingGeotiffResponse(final GridCoverage2D coverage) {
        final GeoTiffWriteParams wp = new GeoTiffWriteParams();
        wp.setCompressionMode(GeoTiffWriteParams.MODE_EXPLICIT);
        wp.setCompressionType("LZW");
        final ParameterValueGroup params = new GeoTiffFormat().getWriteParameters();
        params.parameter(AbstractGridFormat.GEOTOOLS_WRITE_PARAMS.getName().toString()).setValue(wp);
        final GeneralParameterValue[] vals = params.values().toArray(new GeneralParameterValue[1]);
        
        StreamingOutput streamingOutput = new StreamingOutput() {
            public void write(OutputStream outStream) {
                try {
                    new GeoTiffWriter(outStream).write(coverage, vals);
                } catch (Exception e) {
                    LOG.error("exception while preparing geotiff : {}", e.getMessage());
                    throw new WebApplicationException(e);
                }
            }
       };

       CacheControl cc = new CacheControl();
       cc.setMaxAge(3600);
       cc.setNoCache(false);
       return Response.ok(streamingOutput)
                       .type("image/geotiff")
                       .cacheControl(cc)
                       .build();
    }

}
