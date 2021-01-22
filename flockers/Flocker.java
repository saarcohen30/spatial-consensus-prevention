
package sim.app.flockers;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.management.ManagementFactory;
import java.lang.management.ThreadMXBean;

import sim.engine.*;
import sim.field.continuous.*;
import sim.util.*;

public class Flocker implements Steppable, sim.portrayal.Orientable2D
    {
    
	private static final int nClusters = 2;
	private static final long serialVersionUID = 1;
    public static long[] totalStepsUntilConvergence = {0,0,0,0,0};
    public static long[] totalConverged = {0,0,0,0,0};
    public static long[] totalConvergedInCluster = {0,0,0,0,0};
    public static long[] totalLost = {0,0,0,0,0};
    public static long[] totalLostInCluster = {0,0,0,0,0};
    public static boolean[] totalPrint = {false,false,false,false,false};
    public static int[] execution = {0,0,0,0,0};
    public static long startTime = 0;
    public static boolean timePrinted = false;

    public static final int T = 200;
    public static final int tFlock = 2800;

    public Double2D loc = new Double2D(0,0);
    public Double2D lastd = new Double2D(0,0);
    public Continuous2D flockers;
    public Flockers theFlock;
    public String updateRule;
    public boolean isFixed;
    public boolean isDiverting;
    public long nSteps = 0;
    public boolean printed = false;
    public double epsilon;
    public short connComp;
    public double desiredOrient;
    public short clusterID;
    public long nExecutions;
    public int numDesiredOrient;
    
    public Flocker(Double2D location, String update, boolean fixed, boolean divert, double e,
    		short con, double desired, short cID, long nExecutions, int numDesiredOrient) 
    { 
    	loc = location; 
    	updateRule = update;
    	isFixed = fixed;
    	isDiverting = divert;
    	epsilon = e;
    	connComp = con;
    	desiredOrient = desired;
    	clusterID = cID;
    	this.nExecutions = nExecutions;
    	this.numDesiredOrient = numDesiredOrient;
    }
    
    public Bag getNeighbors()
        {
        return flockers.getNeighborsExactlyWithinDistance(loc, theFlock.neighborhood, false);
        }
    
    public double getOrientation() { return orientation2D(); }
        
    public void setOrientation2D(double val)
        {
        lastd = new Double2D(Math.cos(val),Math.sin(val));
        }
    
    public double orientation2D()
        {
        if (lastd.x == 0 && lastd.y == 0) return 0;
        return Math.atan2(lastd.y, lastd.x);
        }
    
    public Double2D momentum()
        {
        return lastd;
        }

    public double consistency(Bag b, Continuous2D flockers)
        {
        if (b==null || b.numObjs == 0) return 0.0;
        
        double orient = 0; 
        int i = 0;
        int count = 0;
        for(i = 0; i < b.numObjs;i++)
            {

        	if (updateRule == "calcDiff") {
        		orient += calcDiff(((Flocker)b.objs[i]).getOrientation() - this.getOrientation());
        	} else {
        		if (updateRule == "perron") {
        			orient += ((Flocker)b.objs[i]).getOrientation() - this.getOrientation();
        		}
        	}
            count++;
            }
        if (count > 0 && updateRule == "calcDiff") { orient /= count; }
        else {
        	if (updateRule == "perron") {
        		orient /= count;
        		
        	}
        }

        return orient;
        }
    
    private double calcDiff(double orientDiff) 
    {    	
    	if(orientDiff >= - Math.PI && orientDiff <= Math.PI) {
    		return orientDiff;
    	} else { 
    		if(orientDiff < - Math.PI ) {
    			return 2 * Math.PI + orientDiff;
    		} else return 2 * Math.PI - orientDiff; 
    	}
    }
    
    public void step(SimState state)
        {
    	if (Flocker.startTime == 0){
    		Flocker.startTime = System.nanoTime();
    	}
    	
    	final Flockers flock = (Flockers)state;
        loc = flock.flockers.getObjectLocation(this);
        
        Bag b = getNeighbors();
        
        double orient = this.getOrientation();            
        
        if (!this.isDiverting) orient +=  consistency(b, flock.flockers);

//        orient +=  consistency(b, flock.flockers);
        
        double dx = Math.cos(orient);
        double dy = Math.sin(orient);

        lastd = new Double2D(dx,dy);
        
        //Toroidal version
//        double nextDx = this.flockers.stx(loc.x + dx);
//        double nextDy = this.flockers.sty(loc.y + dy);
        
        //Non-Toroidal version

        double nextDx = loc.x + dx;
        double nextDy = loc.y + dy;
        
        loc = new Double2D(nextDx, nextDy);        
        
        setOrientation2D(orient);
        if (!isFixed) 
        {
        	flock.flockers.setObjectLocation(this, loc);
        }
        
        this.nSteps++;
        if ((Math.abs(orient - desiredOrient) < 0.01 || Math.abs(orient - (desiredOrient - 2*Math.PI)) < 0.01) 
        		&& !isDiverting && !printed) {
        	Flocker.totalStepsUntilConvergence[connComp] = this.nSteps;
        	Flocker.totalConverged[connComp]++;
        	Flocker.totalConvergedInCluster[clusterID]++;
        	
        	printed = true;
        	}
        
        if (!isFixed && this.nSteps - Flocker.totalStepsUntilConvergence[connComp] > Flocker.T) {
        	/* There exists a subset of flocking agents with cardinality 0 < k' < k 
        	 * and all orientations are within e of alpha for more than 
        	 * T time steps.
        	 * */
        	if (Math.abs(orient - desiredOrient) >= 0.01 && !printed) {
        		Flocker.totalLost[connComp]++;
        		Flocker.totalLostInCluster[clusterID]++;
            	printed = true;
        	}
        }
    	
        long nFlocking = 10 * (connComp+1);
        String directory = System.getProperty("user.dir") + "/" + numDesiredOrient + (isFixed? "fixed" : "switch");
        File dir = new File(directory);
        if (!dir.exists()) {
        	dir.mkdir();
        }
        
        if (Flocker.totalConverged[connComp] + Flocker.totalLost[connComp] == nFlocking && 
        	!Flocker.totalPrint[connComp]) {
        	dir = new File(directory + "/" + connComp);
        	if (!dir.exists()) {
            	dir.mkdir();
            }
        	
        	writeResultsToFile(directory + "/" + connComp + "/TimeSteps ", Flocker.totalStepsUntilConvergence[connComp]);
        	writeResultsToFile("Converged ", Flocker.totalConverged[connComp]);
        	writeResultsToFile(directory + "/" + connComp + "/Lost ", Flocker.totalLost[connComp]);
          	
          	
        	Flocker.execution[connComp]++;
        	Flocker.totalPrint[connComp] = true;
        	for (int i = 0; i < Flocker.totalPrint.length; i++) {
				if (!Flocker.totalPrint[i])
					return;
			}
        	
        	if (!Flocker.timePrinted){
            	writeResultsToFile(directory + "/CPU ", 
            			getCPUTime());
            	writeResultsToFile(directory + "/Runtime ", 
            			System.nanoTime() - Flocker.startTime);
            	disagreementMeasure(directory);

            	Flocker.timePrinted = true;
            	if (Flocker.execution[0] < nExecutions) {
                	Flockers.doLoop(Flockers.class, new String[] {});
				} else {
					System.exit(0);
				}
        	}

        }
        }

	private void writeResultsToFile(String filename, long data) {
		File toFile = new File(filename + theFlock.getPlacement() + " " + theFlock.numDiverting + " " + 
				"-init.txt");
		try {
			BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(toFile,true));
			bufferedWriter.write(data + "\n");
			bufferedWriter.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		String[] param = filename.split("/");		
    	printResult(param[param.length - 1] + ": " + data + " - ");

	}
	
	private void writeResultsToFileDouble(String filename, double data) {
		File toFile = new File(filename + theFlock.getPlacement() + " " + theFlock.numDiverting + " " + 
				"-init.txt");
		try {
			BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(toFile,true));
			bufferedWriter.write(data + "\n");
			bufferedWriter.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
    	printResult(filename + ": " + data + " - ");

	}
    
    private void printResult(String str) {
    	System.out.println(str + Flocker.execution[connComp] + "-" + connComp);
    }
    
    /** Get CPU time in nanoseconds. */
    private long getCPUTime() {
    	ThreadMXBean bean = ManagementFactory.getThreadMXBean( );
        return bean.isCurrentThreadCpuTimeSupported( ) ?
                bean.getCurrentThreadCpuTime( ) : 0L;
    }
    
    private void disagreementMeasure(String directory) {
		int k = 150;
		double denom = Math.pow(k,2);
		for (int i = 0; i < Flocker.nClusters; i++) {
			denom -= Math.pow(totalConvergedInCluster[i],2);
			denom -= totalLostInCluster[i];
		}
		
		double dis = (double)(denom)/(Math.pow(k,2) - k);
		
		writeResultsToFileDouble(directory + "/DisMeasure ", dis);
		
    	System.out.println("Disagreement Measure: " + dis);
	}
    }
