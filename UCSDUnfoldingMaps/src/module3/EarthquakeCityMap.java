package module3;

//Java utilities libraries
import java.util.ArrayList;
//import java.util.Collections;
//import java.util.Comparator;
import java.util.List;

//Processing library
import processing.core.PApplet;

//Unfolding libraries
import de.fhpotsdam.unfolding.UnfoldingMap;
import de.fhpotsdam.unfolding.marker.Marker;
import de.fhpotsdam.unfolding.data.PointFeature;
import de.fhpotsdam.unfolding.marker.SimplePointMarker;
import de.fhpotsdam.unfolding.providers.Google;
import de.fhpotsdam.unfolding.providers.MBTilesMapProvider;
import de.fhpotsdam.unfolding.utils.MapUtils;

//Parsing library
import parsing.ParseFeed;

/** EarthquakeCityMap
 * An application with an interactive map displaying earthquake data.
 * Author: UC San Diego Intermediate Software Development MOOC team
 * @author Your name here
 * Date: July 17, 2015
 * */
public class EarthquakeCityMap extends PApplet {

	// You can ignore this.  It's to keep eclipse from generating a warning.
	private static final long serialVersionUID = 1L;

	// IF YOU ARE WORKING OFFLINE, change the value of this variable to true
	private static final boolean offline = false;
	
	// Less than this threshold is a light earthquake
	public static final float THRESHOLD_MODERATE = 5;
	// Less than this threshold is a minor earthquake
	public static final float THRESHOLD_LIGHT = 4;

	/** This is where to find the local tiles, for working without an Internet connection */
	public static String mbTilesString = "blankLight-1-3.mbtiles";
	
	// The map
	private UnfoldingMap map;
	
	//feed with magnitude 2.5+ Earthquakes
	private String earthquakesURL = "https://earthquake.usgs.gov/earthquakes/feed/v1.0/summary/2.5_week.atom";
	
    // Here is an example of how to use Processing's color method to generate 
    // an int that represents the color yellow.  
    private int yellow = color(255, 255, 150);
    private int blue = color(150, 150, 255);
    private int red = color(255, 150, 150);
    private int white = color(255, 255, 255);

	
	public void setup() {
		size(950, 600, OPENGL);

		if (offline) {
		    map = new UnfoldingMap(this, 200, 50, 700, 500, new MBTilesMapProvider(mbTilesString));
		    earthquakesURL = "2.5_week.atom"; 	// Same feed, saved Aug 7, 2015, for working offline
		}
		else {
			map = new UnfoldingMap(this, 200, 50, 700, 500, new Google.GoogleMapProvider());
			// IF YOU WANT TO TEST WITH A LOCAL FILE, uncomment the next line
			//earthquakesURL = "2.5_week.atom";
			
		}
		
	    map.zoomToLevel(2);
	    MapUtils.createDefaultEventDispatcher(this, map);	
			
	    // The List you will populate with new SimplePointMarkers
	    List<Marker> markers = new ArrayList<Marker>();

	    //Use provided parser to collect properties for each earthquake
	    //PointFeatures have a getLocation method
	    List<PointFeature> earthquakes = ParseFeed.parseEarthquake(this, earthquakesURL);
	    
	    for (PointFeature eq: earthquakes) {
	    	markers.add(createMarker(eq));
	    }
	    
	    map.addMarkers(markers);
	    
//	    // These print statements show you (1) all of the relevant properties 
//	    // in the features, and (2) how to get one property and use it
//	    if (earthquakes.size() > 0) {
//	    	PointFeature f = earthquakes.get(0);
//	    	System.out.println(f.getProperties());
//	    	Object magObj = f.getProperty("magnitude");
//	    	float mag = Float.parseFloat(magObj.toString());
//	    	// PointFeatures also have a getLocation method
//	    }
//	    
//	    // Here is an example of how to use Processing's color method to generate 
//	    // an int that represents the color yellow.  
//	    int yellow = color(255, 255, 150);
//	    int blue = color(150, 150, 255);
//	    int red = color(255, 150, 150);
	    
	    //TODO: Add code here as appropriate
	}
		
	// A suggested helper method that takes in an earthquake feature and 
	// returns a SimplePointMarker for that earthquake
	// TODO: Implement this method and call it from setUp, if it helps
	private SimplePointMarker createMarker(PointFeature feature)
	{
		// finish implementing and use this method, if it helps.	    
		SimplePointMarker mk = new SimplePointMarker(feature.getLocation());
		if ((float) feature.getProperty("magnitude") < 4) {
    		mk.setColor(blue);
    		mk.setStrokeWeight(0);
    	} else if ((float) feature.getProperty("magnitude") < 4.9) {
    		mk.setColor(yellow);
    		mk.setStrokeColor(yellow);
    		mk.setStrokeWeight(3);
    	} else {
    		mk.setColor(red);
    		mk.setStrokeColor(red);
    		mk.setStrokeWeight(6);
    	}
		return mk;
	}
	
	public void draw() {
	    background(10);
	    map.draw();
	    addKey();
	}


	// helper method to draw key in GUI
	// TODO: Implement this method to draw the key
	private void addKey() 
	{	
		// Remember you can use Processing's graphics methods here
		fill(white);
		
		int xbase = 25;
		int ybase = 50;
		int width = 150;
		int height = 250;
		
		rect(xbase, ybase, width, height);
		
		fill(0);
		textAlign(LEFT, CENTER);
		textSize(12);
		text("Earthquake Key", xbase+25, ybase+25);
		
		fill(blue);
		ellipse(xbase+35, ybase+70, 12, 12);
		fill(yellow);
		ellipse(xbase+35, ybase+90, 12, 12);
		fill(red);
		ellipse(xbase+35, ybase+110, 12, 12);
		
		textAlign(LEFT, CENTER);
		fill(0, 0, 0);
		text("Below 4.0", xbase+50, ybase+70);
		text("4.0+ Magnitude", xbase+50, ybase+90);
		text("5.0+ Magnitude", xbase+50, ybase+110);

	}
}
