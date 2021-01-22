
package sim.app.flockers;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

import sim.engine.*;
import sim.util.*;
import sim.field.continuous.*;

public class Flockers extends SimState
    {
    private static final long serialVersionUID = 1;

    public Continuous2D flockers;
    public double width = 500;
    public double height = 500;
    public int numFlockers = 10;
    public int numDiverting = 50;
    public double consistency = 1.0;
    public double momentum = 1.0;
    public double neighborhood = 10;
    public double jump = 0.2;  // how far do we move in a timestep?
    public String updateRule = "perron";
    public boolean isFixed = true;
    public String placement = "grid";
    public int numDesiredOrient = 2;
	public long nExecutions = 100;
	public String nFlockingInConnComp = "i X 10";
    
    public int getNumDesiredOrient() { return numDesiredOrient;	}
	public void setNumDesiredOrient(int numDesiredOrient) { this.numDesiredOrient = numDesiredOrient; }
    public long getnExecutions() { return nExecutions;	}
	public void setnExecutions(long nExecutions) { this.nExecutions = nExecutions; }
	public String getnFlockingInConnComp() { return nFlockingInConnComp; }
	public void setnFlockingInConnComp(String nFlockingInConnComp) { this.nFlockingInConnComp = nFlockingInConnComp; }
	public double getConsistency() { return consistency; }
    public void setConsistency(double val) { if (val >= 0.0) consistency = val; }
    public double getMomentum() { return momentum; }
    public void setMomentum(double val) { if (val >= 0.0) momentum = val; }
    public int getNumFlockers() { return numFlockers; }
    public void setNumFlockers(int val) { if (val >= 1) numFlockers = val; }
    public int getNumDiverting() { return numDiverting; }
    public void setNumDiverting(int val) { if (val >= 0) numDiverting = val; }
    public double getWidth() { return width; }
    public void setWidth(double val) { if (val > 0) width = val; }
    public double getHeight() { return height; }
    public void setHeight(double val) { if (val > 0) height = val; }
    public double getNeighborhood() { return neighborhood; }
    public void setNeighborhood(double val) { if (val > 0) neighborhood = val; }
    public String getUpdateRule() { return updateRule; }
    public void setUpdateRule(String str) { updateRule = str; }
    public boolean getIsFixedRule() { return isFixed; }
    public void setIsFixedRule(boolean fixed) { isFixed = fixed; }
    public String getPlacement() { return placement; }
    public void setPlacement(String str) { placement = str; }
    
    public Double2D[] getLocations()
        {
        if (flockers == null) return new Double2D[0];
        Bag b = flockers.getAllObjects();
        if (b==null) return new Double2D[0];
        Double2D[] locs = new Double2D[b.numObjs];
        for(int i =0; i < b.numObjs; i++)
            locs[i] = flockers.getObjectLocation(b.objs[i]);
        return locs;
        }

    // this is included to demonstrate MASON's parameter sweep facility, see the manual
    public double getMeanXLocation()
        {
        Double2D[] locations = getLocations();
        double avg = 0;
        for(int i = 0; i < locations.length; i++)
            {
            avg += locations[i].x;
            }
        if (locations.length > 0) 
            avg /= locations.length;
        return avg;
        }
    
    public Double2D[] getInvertedLocations()
        {
        if (flockers == null) return new Double2D[0];
        Bag b = flockers.getAllObjects();
        if (b==null) return new Double2D[0];
        Double2D[] locs = new Double2D[b.numObjs];
        for(int i =0; i < b.numObjs; i++)
            {
            locs[i] = flockers.getObjectLocation(b.objs[i]);
            locs[i] = new Double2D(locs[i].y, locs[i].x);
            }
        return locs;
        }

    /** Creates a Flockers simulation with the given random number seed. */
    public Flockers(long seed)
        {
        super(seed);
        }
    
    public void start()
        {
        super.start();
    	System.out.println("------------------------------------------------");
    	
//    	String[] filename = {theFlock.numDiverting + "-Diverting/TimeSteps", "Converged ", "Lost "};
//    	String[] filename = {"/TimeSteps "};
//    	for (int b = 0; b < 5; b++) {
//    		for (String f : filename) {
//    			extractToNewFile(b + f);
//			}    		
//		}
//    	
//    	extractToNewFile("CPU ");
//    	extractToNewFile("Runtime ");

        // set up the flockers field.  It looks like a discretization
        // of about neighborhood / 1.5 is close to optimal for us.  Hmph,
        // that's 16 hash lookups! I would have guessed that 
        // neighborhood * 2 (which is about 4 lookups on average)
        // would be optimal.  Go figure.
        flockers = new Continuous2D(neighborhood/1.5,width,height);
        
        Flocker.startTime = 0;
        Flocker.timePrinted = false;
        for (int b = 0; b < 5; b++) {
        	Flocker.totalStepsUntilConvergence[b] = 0;
            Flocker.totalConverged[b] = 0;
            Flocker.totalPrint[b] = false;
            Flocker.totalLost[b] = 0;
            Flocker.totalLostInCluster[b] = 0;
            Flocker.totalConvergedInCluster[b] = 0;
		}

		Random rand = new Random();
		int additional = rand.nextInt(numDiverting - 3), additional2 = 0;
		if (numDiverting - 3 - additional > 0)
			additional2 = rand.nextInt(numDiverting - 3 - additional);

        // make a bunch of flockers and schedule them.		
		
		if (placement == "grid"){
			/**
			 * Grid - 2 orientations
			 * */
			if (numDesiredOrient == 2) {
				Grid2Orient(rand, additional);
			} else {
				/**
				 * Grid - 3 orientations
				 * */
				if (numDesiredOrient == 3) {
					Grid3Orient(rand, additional, additional2);
				} else {
					/**
					 * Grid - 4 orientations
					 * */
					Grid4Orient(rand, additional, additional2);
				}
			}			
		} else {
			/**
			 *  Random - 2 orientations
			 * */
			if (numDesiredOrient == 2) {
		        Random2Orient(rand, additional);
			} else {
				/**
				 * Random - 3 orientations
				 * */
				if (numDesiredOrient == 3) {
					Random3Orient(rand, additional, additional2);
				} else {
					/**
					 * Random - 4 orientations
					 * */
					Random4Orient(rand, additional, additional2);
				}				
			}
		}	
		
        }
    
	private void Random2Orient(Random rand, int additional) {
		double min_y = 250.0, max_y = Double.MAX_VALUE;		
		double[] max;
		if (rand.nextBoolean()) {
        	if (isFixed)
        		disagreementMeasure(3,5);
        	
        	max = ConnectedCompRandom(100.0, 250.0, Math.PI, (short)0, false, true,(short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	max = ConnectedCompRandom(max[0], max[1], Math.PI, (short)1, false, true,(short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	max = ConnectedCompRandom(max[0], max[1], Math.PI, (short)2, true, false,(short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);

        	placeAdditionalDiverting(max[0]-15, 100.0, rand, additional,
        			max_y,min_y, Math.PI,(short)2,(short)0, nExecutions, numDesiredOrient);
        	
        	double secondMin_x = max[0];
        	max_y = max[2];
        	min_y = max[3];
        	max = ConnectedCompRandom(max[0], max[1], 0.0, (short)3, false, true,(short)1);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	max = ConnectedCompRandom(max[0], max[1], 0.0, (short)4, true, false,(short)1);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	
        	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, numDiverting - 3 - additional,
        			max_y, min_y, 0.0,(short)4,(short)1, nExecutions, numDesiredOrient);
		} else {
        	if (isFixed)
        		disagreementMeasure(2,5);
        	
        	max = ConnectedCompRandom(100.0, 250.0, Math.PI, (short)0, false, true,(short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	max = ConnectedCompRandom(max[0], max[1], Math.PI, (short)1, true, false,(short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	
        	placeAdditionalDiverting(max[0]-15, 100.0, rand, additional,
        			max_y,min_y, Math.PI,(short)1,(short)0, nExecutions, numDesiredOrient);
        	
        	double secondMin_x = max[0];
        	max_y = max[2];
        	min_y = max[3];
        	max = ConnectedCompRandom(max[0], max[1], 0.0, (short)2, false, true,(short)1);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	max = ConnectedCompRandom(max[0], max[1], 0.0, (short)3, false, true,(short)1);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	max = ConnectedCompRandom(max[0], max[1], 0.0, (short)4, true, false,(short)1);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	
        	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, numDiverting - 3 - additional,
        			max_y, min_y, 0.0,(short)4,(short)1, nExecutions, numDesiredOrient);
		}
	}
	
	private void Grid2Orient(Random rand, int additional) {
		double max_x;
		if (rand.nextBoolean()) {
        	if (isFixed)
        		disagreementMeasure(3,5);
        	
        	max_x = ConnectedCompGrid(100.0, 250.0, Math.PI, (short)0, false, true,(short)0);
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, Math.PI, (short)1, false, true,(short)0);
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, Math.PI, (short)2, true, false,(short)0);
        	
        	placeAdditionalDivertingGrid(max_x, 100.0, rand, additional,
        			(neighborhood * 4), Math.PI,(short)0);
        	
        	double secondMin_x = max_x;
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, 0.0, (short)3, false, true,(short)1);
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, 0.0, (short)4, true, false,(short)1);
        	
        	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, numDiverting - 3 - additional,
        			(neighborhood * (Math.floor(80 / 9))), 0.0,(short)1);
		} else {
        	if (isFixed)
        		disagreementMeasure(2,5);
        	
        	max_x = ConnectedCompGrid(100.0, 250.0, Math.PI, (short)0, false, true,(short)0);
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, Math.PI, (short)1, true, false,(short)0);
        	
        	placeAdditionalDivertingGrid(max_x, 100.0, rand, additional,
        			(neighborhood * 4), Math.PI,(short)0);
        	
        	double secondMin_x = max_x;
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, 0.0, (short)2, false, true,(short)1);
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, 0.0, (short)3, false, true,(short)1);
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, 0.0, (short)4, true, false,(short)1);
        	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, numDiverting - 3 - additional,
        			(neighborhood * (Math.floor(40 / 7))), 0.0,(short)1);
		}
	}
	
	private void Grid3Orient(Random rand, int additional, int additional2) {
		double max_x;
		if (rand.nextBoolean()) { 
			/**
			 * C_1 U C_2
			 * */
        	max_x = ConnectedCompGrid(100.0, 250.0, Math.PI, (short)0, false, true,(short)0);
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, Math.PI, (short)1, true, false,(short)0);
        	placeAdditionalDivertingGrid(max_x, 100.0, rand, additional,
        			(neighborhood * 2), Math.PI,(short)0);
        	
        	if (rand.nextBoolean()) {
        		/**
        		 * C_1 U C_2, C_3 U C_4, C_5
        		 * */
            	if (isFixed)
            		disagreementMeasure(2,4);
            	
            	double secondMin_x = max_x;

            	max_x = ConnectedCompGrid(max_x + 6, 250.0, Math.PI/2, (short)2, false, true,(short)1);
            	max_x = ConnectedCompGrid(max_x + 6, 250.0, Math.PI/2, (short)3, true, false,(short)1);            	
            	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, additional2,
            			(neighborhood * 5), Math.PI/2,(short)1);
            	
            	secondMin_x = max_x;
            	max_x = ConnectedCompGrid(max_x + 6, 250.0, 0.0, (short)4, false, true,(short)2);
            	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, 
            			numDiverting - 3 - additional - additional2,
            			(neighborhood * (Math.floor(50 / 8))), 0.0,(short)2); 
        	} else {
        		/**
        		 * C_1 U C_2, C_3 , C_4 U C_5
        		 * */
            	if (isFixed)
            		disagreementMeasure(2,3);
            	
            	double secondMin_x = max_x;
            	max_x = ConnectedCompGrid(max_x + 6, 250.0, Math.PI/2, (short)2, true, false,(short)1);
             	if (additional2 == 0) additional2++;
            	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, additional2,
            			(neighborhood * 5), Math.PI/2,(short)1);
            	
            	secondMin_x = max_x;
            	max_x = ConnectedCompGrid(max_x + 6, 250.0, 0.0, (short)3, false, true,(short)2);
            	max_x = ConnectedCompGrid(max_x + 6, 250.0, 0.0, (short)4, true, false,(short)2);
            	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, 
            			numDiverting - 3 - additional - additional2,
            			(neighborhood * (Math.floor(40 / 7))), 0.0,(short)2); 
        	}
        	
		} else {
			/**
    		 * C_1 , C_2 U C_3, C_4 U C_5
    		 * */
        	if (isFixed)
        		disagreementMeasure(1,3);
        	
        	max_x = ConnectedCompGrid(100.0, 250.0, Math.PI, (short)0, true, false,(short)0);
        	
        	if (additional == 0) additional++;
        	placeAdditionalDivertingGrid(max_x, 100.0, rand, additional,
        			(neighborhood * 2), Math.PI,(short)0);
        	
        	double secondMin_x = max_x;
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, Math.PI/2, (short)1, false, true,(short)1);
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, Math.PI/2, (short)2, true, false,(short)1);
        	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, additional2,
        			(neighborhood * 4), Math.PI/2,(short)1);    

        	
        	secondMin_x = max_x;
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, 0.0, (short)3, false, true,(short)2);
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, 0.0, (short)4, true, false,(short)2);
        	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, 
        			numDiverting - 3 - additional - additional2,
        			(neighborhood * (Math.floor(40 / 7))), 0.0,(short)2);
		}
	}
	
	private void Grid4Orient(Random rand, int additional, int additional2) {
		double max_x;
		int additional3 = 0;
		if (numDiverting - 4 - additional - additional2 > 0)
			additional3 = rand.nextInt(numDiverting - 4 - additional - additional2);
		if (rand.nextBoolean()) {
			/**
			 * C_1
			 * */
        	max_x = ConnectedCompGrid(100.0, 250.0, Math.PI/4, (short)0, true, false,(short)0);        	
        	if (additional == 0) additional++;
        	placeAdditionalDivertingGrid(max_x, 100.0, rand, additional,
        			(neighborhood * 2), Math.PI/4,(short)0);
        	
        	if (rand.nextBoolean()) {
        		/**
        		 * C_1, C_2
        		 * */
            	
            	double secondMin_x = max_x;

            	max_x = ConnectedCompGrid(max_x + 6, 250.0, 3*Math.PI/4, (short)1, true, false,(short)1);
            	if (additional2 == 0) additional2++;
            	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, additional2,
            			(neighborhood * 4), 3*Math.PI/4,(short)1);
            	
            	if (rand.nextBoolean()) {
            		/**
            		 * C_1, C_2, C_3, C_4 U C_5
            		 * */
                	if (isFixed)
                		disagreementMeasure4Orient(1, 2, 3);
            		
            		secondMin_x = max_x;
                	max_x = ConnectedCompGrid(max_x + 6, 250.0, 5*Math.PI/4, (short)2, true, false,(short)2);
                	if (additional3 == 0) additional3++;
                	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, additional3,
                			(neighborhood * 5), 5*Math.PI/4,(short)2);
                	
                	secondMin_x = max_x;
                	max_x = ConnectedCompGrid(max_x + 6, 250.0, 7*Math.PI/4, (short)3, false, true,(short)3);
                	max_x = ConnectedCompGrid(max_x + 6, 250.0, 7*Math.PI/4, (short)4, true, false,(short)3);
                	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, 
                			numDiverting - 3 - additional - additional2 - additional3,
                			(neighborhood * (Math.floor(40 / 7))), 7*Math.PI/4,(short)3);
            	} else {
            		/**
            		 * C_1, C_2, C_3 U C_4, C_5
            		 * */
                	if (isFixed)
                		disagreementMeasure4Orient(1, 2, 4);
                	
                	secondMin_x = max_x;
                	max_x = ConnectedCompGrid(max_x + 6, 250.0, 5*Math.PI/4, (short)2, false, true,(short)2);
                	max_x = ConnectedCompGrid(max_x + 6, 250.0, 5*Math.PI/4, (short)3, true, false,(short)2);
                	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, additional3,
                			(neighborhood * 5), 5*Math.PI/4,(short)2);
                	
                	secondMin_x = max_x;
                	max_x = ConnectedCompGrid(max_x + 6, 250.0, 7*Math.PI/4, (short)4, false, true,(short)3);
                	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, 
                			numDiverting - 3 - additional - additional2 - additional3,
                			(neighborhood * (Math.floor(50 / 8))), 7*Math.PI/4,(short)3);
            	}
        	} else {
        		/**
        		 * C_1, C_2 U C_3, C_4, C_5
        		 * */
            	if (isFixed)
            		disagreementMeasure4Orient(1, 3, 4);
            	
            	double secondMin_x = max_x;
            	max_x = ConnectedCompGrid(max_x + 6, 250.0, 3*Math.PI/4, (short)1, false, true,(short)1);
            	max_x = ConnectedCompGrid(max_x + 6, 250.0, 3*Math.PI/4, (short)2, true, false,(short)1);
            	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, additional3,
            			(neighborhood * 5), 3*Math.PI/4,(short)1);
            	
            	secondMin_x = max_x;
            	max_x = ConnectedCompGrid(max_x + 6, 250.0, 5*Math.PI/4, (short)3, true, false,(short)2);
            	if (additional3 == 0) additional3++;
            	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, additional3,
            			(neighborhood * 5), 5*Math.PI/4,(short)2);
            	
            	secondMin_x = max_x;
            	max_x = ConnectedCompGrid(max_x + 6, 250.0, 7*Math.PI/4, (short)4, false, true,(short)3);
            	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, 
            			numDiverting - 3 - additional - additional2 - additional3,
            			(neighborhood * (Math.floor(50 / 8))), 7*Math.PI/4,(short)3);
        	}
        	
		} else {
			/**
    		 * C_1 U C_2, C_3, C_4, C_5
    		 * */
        	if (isFixed)
        		disagreementMeasure4Orient(2,3,4);
        	
			max_x = ConnectedCompGrid(100.0, 250.0, Math.PI/4, (short)0, false, true,(short)0);
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, Math.PI/4, (short)1, true, false,(short)0);
        	placeAdditionalDivertingGrid(max_x, 100.0, rand, additional,
        			(neighborhood * 2), Math.PI/4,(short)0);
        	
        	double secondMin_x = max_x;
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, 3*Math.PI/4, (short)2, true, false,(short)1);
        	if (additional2 == 0) additional2++;
        	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, additional2,
        			(neighborhood * 5), 3*Math.PI/4,(short)1);
        	
        	secondMin_x = max_x;
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, 5*Math.PI/4, (short)3, true, false,(short)2);
        	if (additional3 == 0) additional3++;
        	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, additional3,
        			(neighborhood * 5), 5*Math.PI/4,(short)2);
        	
        	secondMin_x = max_x;
        	max_x = ConnectedCompGrid(max_x + 6, 250.0, 7*Math.PI/4, (short)4, false, true,(short)3);
        	placeAdditionalDivertingGrid(max_x, secondMin_x + 15, rand, 
        			numDiverting - 3 - additional - additional2 - additional3,
        			(neighborhood * (Math.floor(50 / 8))), 7*Math.PI/4,(short)3);        	
		}
	}
	
	private void Random3Orient(Random rand, int additional, int additional2) {
		double min_y = 250.0, max_y = Double.MAX_VALUE;		
		double[] max;
		if (rand.nextBoolean()) { 
			/**
			 * C_1 U C_2
			 * */
			max = ConnectedCompRandom(100.0, 250.0, Math.PI, (short)0, false, true,(short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	max = ConnectedCompRandom(max[0], max[1], Math.PI, (short)1, true, false,(short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);

        	placeAdditionalDiverting(max[0]-15, 100.0, rand, additional,
        			max_y,min_y, Math.PI,(short)1,(short)0, nExecutions, numDesiredOrient);
        	
        	if (rand.nextBoolean()) {
        		/**
        		 * C_1 U C_2, C_3 U C_4, C_5
        		 * */
            	if (isFixed)
            		disagreementMeasure(2,4);
            	
            	double secondMin_x = max[0];
            	max = ConnectedCompRandom(max[0], max[1], Math.PI/2, (short)2, false, true,(short)1);
            	max_y = max[2];
            	min_y = max[3];
            	max = ConnectedCompRandom(max[0], max[1], Math.PI/2, (short)3, true, false,(short)1);
            	max_y = Math.min(max[2], max_y);
            	min_y = Math.max(min_y, max[3]);
            	
            	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, additional2,
            			max_y, min_y, Math.PI/2,(short)3,(short)1, nExecutions, numDesiredOrient);
            	
            	secondMin_x = max[0];
            	double secondMiny = max[1];
            	max = ConnectedCompRandom(max[0], max[1], 0.0, (short)4, false, true,(short)2);
            	
            	if (numDiverting - 3 - additional - additional2 == 1) {
    		    	placeDivertingAgent(0.0, (short)4, secondMin_x+0.1, secondMiny,(short)2, nExecutions);
            	} else {
                	max_y = max[2];
                	min_y = max[3];
                	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, 
                			numDiverting - 3 - additional - additional2,
                			max_y, min_y, 0.0,(short)4,(short)2, nExecutions, numDesiredOrient); 
            	}     	            	
        	} else {
        		/**
        		 * C_1 U C_2, C_3 , C_4 U C_5
        		 * */
            	if (isFixed)
            		disagreementMeasure(2,3);
            	
            	double secondMin_x = max[0];
            	double secondMiny = max[1];
            	max = ConnectedCompRandom(max[0], max[1], Math.PI/2, (short)2, true, false,(short)1);
             	if (additional2 == 0) additional2++;
            	if (additional2 == 1) {
    		    	placeDivertingAgent(Math.PI/2, (short)2, secondMin_x+0.1, secondMiny,(short)1, nExecutions);
            	} else {
                	max_y = max[2];
                	min_y = max[3];
                	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, additional2,
                			max_y, min_y, Math.PI/2,(short)2,(short)1, nExecutions, numDesiredOrient);                	
            	}
            	
            	secondMin_x = max[0];
            	max = ConnectedCompRandom(max[0], max[1], 0.0, (short)3, false, true,(short)2);
            	max_y = max[2];
            	min_y = max[3];
            	max = ConnectedCompRandom(max[0], max[1], 0.0, (short)4, true, false,(short)2);
            	max_y = Math.min(max[2], max_y);
            	min_y = Math.max(min_y, max[3]);
            	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, 
            			numDiverting - 3 - additional - additional2,
            			max_y, min_y, 0.0,(short)4,(short)2, nExecutions, numDesiredOrient); 
        	}
        	
		} else {
			/**
    		 * C_1 , C_2 U C_3, C_4 U C_5
    		 * */
        	if (isFixed)
        		disagreementMeasure(1,3);
        	
        	max = ConnectedCompRandom(100.0, 250.0, Math.PI, (short)0, true, false,(short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	if (additional == 0) additional++;
        	if (additional == 1) {
		    	placeDivertingAgent(Math.PI, (short)0, 30.1, 250.0,(short)0, nExecutions);
        	} else {
        		placeAdditionalDiverting(max[0]-15, 100.0, rand, additional,
            			max_y, min_y, Math.PI,(short)0,(short)0, nExecutions, numDesiredOrient);	
        	}
        	
        	
        	double secondMin_x = max[0];
        	max = ConnectedCompRandom(max[0], max[1], Math.PI/2, (short)1, false, true,(short)1);
        	max_y = max[2];
        	min_y = max[3];
        	max = ConnectedCompRandom(max[0], max[1], Math.PI/2, (short)2, true, false,(short)1);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, additional2,
        			max_y, min_y, Math.PI/2,(short)2,(short)1, nExecutions, numDesiredOrient);    

        	
        	secondMin_x = max[0];
        	max = ConnectedCompRandom(max[0], max[1], 0.0, (short)3, false, true,(short)2);
        	max_y = max[2];
        	min_y = max[3];
        	max = ConnectedCompRandom(max[0], max[1], 0.0, (short)4, true, false,(short)2);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, 
        			numDiverting - 2 - additional - additional2,
        			max_y, min_y, 0.0,(short)4,(short)2, nExecutions, numDesiredOrient);
		}
	}
    
	private void Random4Orient(Random rand, int additional, int additional2) {
		double min_y = 250.0, max_y = Double.MAX_VALUE;		
		double[] max;
		int additional3 = 0;
		if (numDiverting - 4 - additional - additional2 > 0)
			additional3 = rand.nextInt(numDiverting - 4 - additional - additional2);
		if (rand.nextBoolean()) {
			/**
			 * C_1
			 * */
			max = ConnectedCompRandom(100.0, 250.0, Math.PI/4, (short)0, true, false, (short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	if (additional == 0) additional++;
        	if (additional == 1) {
		    	placeDivertingAgent(Math.PI/4, (short)0, 30.1, 250.0, (short)0, nExecutions);
        	} else {
        		placeAdditionalDiverting(max[0]-15, 100.0, rand, additional,
            			max_y, min_y, Math.PI/4,(short)0,(short)0, nExecutions, numDesiredOrient);	
        	}     	
        	if (rand.nextBoolean()) {
        		/**
        		 * C_1, C_2
        		 * */           	
            	double secondMin_x = max[0];
            	double secondMiny = max[1];
            	max = ConnectedCompRandom(max[0], max[1], 3*Math.PI/4, (short)1, true, false, (short)1);
            	max_y = max[2];
            	min_y = max[3];
            	if (additional2 == 0) additional2++;
            	if (additional2 == 1) {
    		    	placeDivertingAgent(3*Math.PI/4, (short)1, secondMin_x+0.1, secondMiny, (short)1, nExecutions);
            	} else {
            		placeAdditionalDiverting(max[0]-15, secondMin_x, rand, additional2,
                			max_y, min_y, 3*Math.PI/4,(short)1, (short)1, nExecutions, numDesiredOrient);
            	}
            	          	
            	if (rand.nextBoolean()) {
            		/**
            		 * C_1, C_2, C_3, C_4 U C_5
            		 * */
            		
                	if (isFixed)
                		disagreementMeasure4Orient(1, 2, 3);
            		
            		secondMin_x = max[0];
            		secondMiny = max[1];
                	max = ConnectedCompRandom(max[0], max[1], 5*Math.PI/4, (short)2, true, false, (short)2);
                	max_y = max[2];
                	min_y = max[3];
                	if (additional3 == 0) additional3++;
                	if (additional3 == 1) {
        		    	placeDivertingAgent(5*Math.PI/4, (short)2, secondMin_x+0.1, secondMiny, (short)2, nExecutions);
                	} else {
                		placeAdditionalDiverting(max[0]-15, secondMin_x, rand, additional3,
                    			max_y, min_y, 5*Math.PI/4,(short)2, (short)2, nExecutions, numDesiredOrient);
                	}
                	
                	secondMin_x = max[0];
                	max = ConnectedCompRandom(max[0], max[1], 7*Math.PI/4, (short)3, false, true, (short)3);
                	max_y = max[2];
                	min_y = max[3];
                	max = ConnectedCompRandom(max[0], max[1], 7*Math.PI/4, (short)4, true, false, (short)3);
                	max_y = Math.min(max[2], max_y);
                	min_y = Math.max(min_y, max[3]);
                	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, 
                			numDiverting - 4 - additional - additional2,
                			max_y, min_y, 7*Math.PI/4,(short)3, (short)3, nExecutions, numDesiredOrient);
            	} else {
            		/**
            		 * C_1, C_2, C_3 U C_4, C_5
            		 * */
                	if (isFixed)
                		disagreementMeasure4Orient(1, 2, 4);
                	
            		secondMin_x = max[0];
                	max = ConnectedCompRandom(max[0], max[1], 5*Math.PI/4, (short)2, false, true, (short)2);
                	max_y = max[2];
                	min_y = max[3];
                	max = ConnectedCompRandom(max[0], max[1], 5*Math.PI/4, (short)3, true, false, (short)2);
                	max_y = Math.min(max[2], max_y);
                	min_y = Math.max(min_y, max[3]);
                	
                	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, additional2,
                			max_y, min_y, 5*Math.PI/4,(short)3, (short)2, nExecutions, numDesiredOrient);
                	
                	secondMin_x = max[0];
                	secondMiny = max[1];
                	max = ConnectedCompRandom(max[0], max[1], 7*Math.PI/4, (short)4, false, true, (short)3);
                	
                	if (numDiverting - 3 - additional - additional2 == 1) {
        		    	placeDivertingAgent(7*Math.PI/4, (short)4, secondMin_x+0.1, secondMiny, (short)3, nExecutions);
                	} else {
                    	max_y = max[2];
                    	min_y = max[3];
                    	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, 
                    			numDiverting - 4 - additional - additional2,
                    			max_y, min_y, 7*Math.PI/4,(short)4, (short)3, nExecutions, numDesiredOrient); 
                	}     	            	
            	}
        	} else {
        		/**
        		 * C_1, C_2 U C_3, C_4, C_5
        		 * */
            	if (isFixed)
            		disagreementMeasure4Orient(1, 3, 4);
            	
        		double secondMin_x = max[0];
            	max = ConnectedCompRandom(max[0], max[1], 3*Math.PI/4, (short)1, false, true, (short)1);
            	max_y = max[2];
            	min_y = max[3];
            	max = ConnectedCompRandom(max[0], max[1], 3*Math.PI/4, (short)2, true, false, (short)1);
            	max_y = Math.min(max[2], max_y);
            	min_y = Math.max(min_y, max[3]);
            	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, additional2,
            			max_y, min_y, 3*Math.PI/4,(short)2, (short)1, nExecutions, numDesiredOrient);
            	
            	secondMin_x = max[0];
            	double secondMiny = max[1];
            	max = ConnectedCompRandom(max[0], max[1], 5*Math.PI/4, (short)3, true, false, (short)2);
            	max_y = max[2];
            	min_y = max[3];
            	if (additional3 == 0) additional3++;
            	if (additional3 == 1) {
    		    	placeDivertingAgent(5*Math.PI/4, (short)3, secondMin_x+0.1, secondMiny, (short)2, nExecutions);
            	} else {
            		placeAdditionalDiverting(max[0]-15, secondMin_x, rand, additional3,
                			max_y, min_y, 5*Math.PI/4,(short)3, (short)2, nExecutions, numDesiredOrient);
            	}
            	
            	secondMin_x = max[0];
            	secondMiny = max[1];
            	max = ConnectedCompRandom(max[0], max[1], 7*Math.PI/4, (short)4, false, true, (short)3);
            	
            	if (numDiverting - 3 - additional - additional2 == 1) {
    		    	placeDivertingAgent(7*Math.PI/4, (short)4, secondMin_x+0.1, secondMiny, (short)3, nExecutions);
            	} else {
                	max_y = max[2];
                	min_y = max[3];
                	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, 
                			numDiverting - 4 - additional - additional2,
                			max_y, min_y, 7*Math.PI/4,(short)4, (short)3, nExecutions, numDesiredOrient); 
            	}     	            	
        	}
        	
		} else {
			/**
    		 * C_1 U C_2, C_3, C_4, C_5
    		 * */
        	if (isFixed)
        		disagreementMeasure4Orient(2,3,4);
        	
			max = ConnectedCompRandom(100.0, 250.0, Math.PI/4, (short)0, false, true, (short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);
        	max = ConnectedCompRandom(max[0], max[1], Math.PI/4, (short)1, true, false,(short)0);
        	max_y = Math.min(max[2], max_y);
        	min_y = Math.max(min_y, max[3]);

        	placeAdditionalDiverting(max[0]-15, 100.0, rand, additional,
        			max_y,min_y, Math.PI/4,(short)1,(short)0, nExecutions, numDesiredOrient);
        	
        	double secondMin_x = max[0];
        	double secondMiny = max[1];
        	max = ConnectedCompRandom(max[0], max[1], 3*Math.PI/4, (short)2, true, false,(short)1);
        	max_y = max[2];
        	min_y = max[3];
        	if (additional2 == 0) additional2++;
        	if (additional2 == 1) {
		    	placeDivertingAgent(3*Math.PI/4, (short)2, secondMin_x+0.1, secondMiny,(short)1, nExecutions);
        	} else {
        		placeAdditionalDiverting(max[0]-15, secondMin_x, rand, additional2,
            			max_y, min_y, 3*Math.PI/4,(short)2,(short)1, nExecutions, numDesiredOrient);
        	}
        	
        	secondMin_x = max[0];
        	secondMiny = max[1];
        	max = ConnectedCompRandom(max[0], max[1], 5*Math.PI/4, (short)3, true, false,(short)2);
        	max_y = max[2];
        	min_y = max[3];
        	if (additional3 == 0) additional3++;
        	if (additional3 == 1) {
		    	placeDivertingAgent(5*Math.PI/4, (short)3, secondMin_x+0.1, secondMiny,(short)2, nExecutions);
        	} else {
        		placeAdditionalDiverting(max[0]-15, secondMin_x, rand, additional3,
            			max_y, min_y, 5*Math.PI/4,(short)3,(short)2, nExecutions, numDesiredOrient);
        	}
        	
        	secondMin_x = max[0];
        	secondMiny = max[1];
        	max = ConnectedCompRandom(max[0], max[1], 7*Math.PI/4, (short)4, false, true,(short)3);
        	
        	if (numDiverting - 4 - additional - additional2 == 1) {
		    	placeDivertingAgent(7*Math.PI/4, (short)4, secondMin_x+0.1, secondMiny,(short)3, nExecutions);
        	} else {
            	max_y = max[2];
            	min_y = max[3];
            	placeAdditionalDiverting(max[0]-15, secondMin_x, rand, 
            			numDiverting - 4 - additional - additional2,
            			max_y, min_y, 7*Math.PI/4,(short)4,(short)3, nExecutions, numDesiredOrient); 
        	}            	
		}
	}
	
	private void extractToNewFile(String f) {
		File toFile = new File("Diverting-4orient/" + f + this.getPlacement() + " " + numDiverting + " -init.txt");
		File toFileNew = new File("Diverting-4orient/" + f + this.getPlacement() + " " + numDiverting + "-init-new.txt");

		try {
			BufferedReader bufferedReader = new BufferedReader(new FileReader(toFile));
			BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(toFileNew));
			String line;
			while ((line = bufferedReader.readLine()) != null) {
				bufferedWriter.write(line.split("-")[0] + "\r\n");
			}
			bufferedWriter.close();
			bufferedReader.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	private void placeAdditionalDivertingGrid(double max_x, double min_x, Random rand,
			int nAdditionalDivert, double yDiff, double desiredOrient, short cID) {
		if (nAdditionalDivert != 0) {
			for (int i = 0; i < nAdditionalDivert; i++) {
				double xcor = min_x + (max_x - min_x) * rand.nextDouble();
				double ycor = 250.0 + yDiff * rand.nextDouble();
		    	placeDivertingAgent(desiredOrient, (short)0, xcor, ycor, cID, nExecutions);
			}
		}
	}
	
	private double ConnectedCompGrid(double initx, double inity, double desiredOrient, 
			short con, boolean noDiverting, boolean linearLine, short cID) {
		Random rand = new Random();
		long nFlocking = 0;
		if (nFlockingInConnComp.equals("equal")) {
			nFlocking = this.numFlockers;
		} else if (nFlockingInConnComp.equals("i X 10")) {
			nFlocking = (con+1) * 10;
		} else if (nFlockingInConnComp.equals("i X m")) {
			nFlocking = (con+1) * this.numDiverting;
		}
        double xcor = initx;
    	double ycor = inity;
    	double max_x = xcor;
    	double max_y = ycor;
    	double min_x = xcor;
    	double min_y = ycor;
		double rightestX = 0.0,rightestY = 0.0;

		int i = 1;
		while (nFlocking > Math.pow(i,2)) {
			i++;
		}
		
		int x = 0;
		int j = 1;
		int k = 1;
		    		
		for ( ; j <= i && x < nFlocking; j++) {
			k = 1;
			ycor = inity + (neighborhood-1) * j;
			for (; k <= i && x < nFlocking; k++) {
				xcor = initx + (neighborhood-1) * k;
				
	        	placeFlockingAgent(rand, xcor, ycor, con, desiredOrient, cID, nExecutions, numDesiredOrient);
	        	x++;
			}
    		
    		max_x = initx + (neighborhood-1) * i;
        	max_y = inity + (neighborhood-1) * (j-1) ;
        	min_x = initx + neighborhood-1;
        	min_y = inity + neighborhood-1;
        	rightestX = max_x;
    	}
    	

    	if (linearLine) {
    		max_y = min_y + neighborhood * (Math.floor(nFlocking / i) - 1)/2;

			min_y = min_y - 15/6 * Math.sqrt(7);
			max_y = max_y + 15/6 * Math.sqrt(7);
		}


    	if (!noDiverting) {
	    	xcor = linearLine? (rightestX + 15.0 /2) : min_x + (max_x - min_x) * rand.nextDouble();
	    	ycor = min_y + (max_y - min_y) * rand.nextDouble();

	    	placeDivertingAgent(desiredOrient, con, xcor, ycor, cID, nExecutions);
		}
    	
    	return max_x;
	}
	
	private double[] ConnectedCompRandom(double initx, double inity, double desiredOrient, 
			short con, boolean noDiverting, boolean linearLine, short cID) {
		Random rand = new Random();
		long nFlocking = (con+1) * 10;
        double xcor = initx;
    	double ycor = inity;
    	double max_x = xcor;
    	double max_y = ycor;
    	double min_x = xcor;
    	double min_y = ycor;
		double rightestX = 0.0,rightestY = 0.0;

		for(int x = 0; x < nFlocking; x++) {
			double tmp_xcor = xcor;
			double tmp_ycor = ycor;
			if (x != 0) {
            	while (true) {
					double r = neighborhood * Math.sqrt(rand.nextDouble());
    				double theta = rand.nextDouble() * 2 * Math.PI;
            		xcor = tmp_xcor + r * Math.cos(theta);
        			ycor = tmp_ycor + r * Math.sin(theta);
            		if (xcor > initx + 60.0 || xcor <= initx || ycor < 0) {
						continue;
					}
            		
                	break;
				}       
        	}
        	if (xcor >= max_x) {
        		max_x = xcor;
        		rightestX = xcor;
        		rightestY = ycor;
			}
        
        	max_y = Math.max(max_y, ycor);
        	min_x = Math.min(min_x, xcor);
        	min_y = Math.min(min_y, ycor);
        	placeFlockingAgent(rand, xcor, ycor, con, desiredOrient, cID, nExecutions, numDesiredOrient);
		}

    	if (linearLine) {
    		min_y = rightestY - 15/6 * Math.sqrt(7);
			max_y = rightestY + 15/6 * Math.sqrt(7);
		}


    	if (!noDiverting) {
	    	xcor = linearLine? (rightestX + 15.0 /2) : min_x + (max_x - min_x) * rand.nextDouble();
	    	ycor = min_y + (max_y - min_y) * rand.nextDouble();

	    	placeDivertingAgent(desiredOrient, con, xcor, ycor, cID, nExecutions);
		}
    	

    	
    	return new double[] {rightestX + 15,rightestY,max_y,min_y};
	}
	
	private void placeDivertingAgent(double desiredOrient, short con,
			double xcor, double ycor, short cID, long nExecutions) {
		Double2D location = new Double2D(xcor,ycor);
		Flocker flocker = new Flocker(location, updateRule, isFixed, true, (double) 1/61, con, desiredOrient, cID, nExecutions, numDesiredOrient);
		flocker.setOrientation2D(desiredOrient);
		flockers.setObjectLocation(flocker, location);
		flocker.flockers = flockers;
		flocker.theFlock = this;
		schedule.scheduleRepeating(flocker);
	}
    
	private void placeFlockingAgent(Random rand, double xcor, double ycor, short con, double desiredOrient, short cID, long nExecutions, int numDesiredOrient) {
		Double2D location = new Double2D(xcor, 
										 ycor);
		Flocker flocker = new Flocker(location, updateRule, isFixed, false,(double) 1/61, con, desiredOrient, cID, nExecutions, numDesiredOrient);
		double orient = 2 * rand.nextDouble() * Math.PI;
		flocker.setOrientation2D(orient);
		flockers.setObjectLocation(flocker, location);
		flocker.flockers = flockers;
		flocker.theFlock = this;
		schedule.scheduleRepeating(flocker);
	}
	
	private void placeAdditionalDiverting(double max_x, double min_x, Random rand,
			int nAdditionalDivert, double max_y, double min_y, double desiredOrient, short con, short cID, long nExecutions, int numDesiredOrient) {
		if (nAdditionalDivert != 0) {
			for (int i = 0; i < nAdditionalDivert; i++) {
				double xcor = min_x + (max_x - min_x) * rand.nextDouble();
				double ycor = min_y + (max_y - min_y) * rand.nextDouble();
		    	placeDivertingAgent(desiredOrient, (short)con, xcor, ycor, cID, nExecutions);
			}
		}
	}
	
	private void disagreementMeasure(int nFirstConns, int nSecondCons) {
		int firstCluster = 0;
		int i = 1;
		for (; i <= nFirstConns; i++) {
			firstCluster += 10 * i;
		}
		
		int k = firstCluster, secondCluster = 0;
		for (; i <= nSecondCons; i++) {
			secondCluster += 10 * i;
		}
		
		k += secondCluster;
		
		int thirdCluster = 0;
		for (; i <= 5; i++) {
			thirdCluster += 10 * i;
		}
		
		k += thirdCluster;
		
		double dis = (double)(Math.pow(k,2) - Math.pow(firstCluster, 2) - 
				Math.pow(secondCluster, 2) - Math.pow(thirdCluster, 2))/(Math.pow(k,2) - k);
		
		File toFile = new File("Diverting-4orient/DisMeasure " + placement + " " + numDiverting + " " + 
				"-init.txt");
		try {
			BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(toFile,true));
			bufferedWriter.write(dis + "\n");
			bufferedWriter.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
    	System.out.println("Disagreement Measure: " + dis);
	}
	
	private void disagreementMeasure4Orient(int nFirstConns, int nSecondCons, int nThirdCons) {
		int firstCluster = 0;
		int i = 1;
		for (; i <= nFirstConns; i++) {
			firstCluster += 10 * i;
		}
		
		int k = firstCluster, secondCluster = 0;
		for (; i <= nSecondCons; i++) {
			secondCluster += 10 * i;
		}
		
		k += secondCluster;
		
		int thirdCluster = 0;
		for (; i <= nThirdCons; i++) {
			thirdCluster += 10 * i;
		}
		
		k += thirdCluster;
		
		int fourthCluster = 0;
		for (; i <= 5; i++) {
			fourthCluster += 10 * i;
		}
		
		k += fourthCluster;
		
		double dis = (double)(Math.pow(k,2) - Math.pow(firstCluster, 2) - 
				Math.pow(secondCluster, 2) - Math.pow(thirdCluster, 2) -
				Math.pow(fourthCluster, 2))/(Math.pow(k,2) - k);
		
		File toFile = new File("Diverting-4orient/DisMeasure " + placement + " " + numDiverting + " " + 
				"-init.txt");
		try {
			BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(toFile,true));
			bufferedWriter.write(dis + "\n");
			bufferedWriter.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
    	System.out.println("Disagreement Measure: " + dis);
	}
	
    public static void main(String[] args)
        {
        doLoop(Flockers.class, args);
        System.exit(0);
        }    
    }
