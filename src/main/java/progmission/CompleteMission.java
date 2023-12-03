package progmission;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import org.slf4j.Logger;

import fr.cnes.sirius.patrius.attitudes.Attitude;
import fr.cnes.sirius.patrius.attitudes.AttitudeLaw;
import fr.cnes.sirius.patrius.attitudes.AttitudeLawLeg;
import fr.cnes.sirius.patrius.attitudes.AttitudeLeg;
import fr.cnes.sirius.patrius.attitudes.AttitudeProvider;
import fr.cnes.sirius.patrius.attitudes.ConstantSpinSlew;
import fr.cnes.sirius.patrius.attitudes.StrictAttitudeLegsSequence;
import fr.cnes.sirius.patrius.attitudes.TargetGroundPointing;
import fr.cnes.sirius.patrius.bodies.ExtendedOneAxisEllipsoid;
import fr.cnes.sirius.patrius.events.CodedEvent;
import fr.cnes.sirius.patrius.events.CodedEventsLogger;
import fr.cnes.sirius.patrius.events.GenericCodingEventDetector;
import fr.cnes.sirius.patrius.events.Phenomenon;
import fr.cnes.sirius.patrius.events.postprocessing.AndCriterion;
import fr.cnes.sirius.patrius.events.postprocessing.ElementTypeFilter;
import fr.cnes.sirius.patrius.events.postprocessing.NotCriterion;
import fr.cnes.sirius.patrius.events.postprocessing.Timeline;
import fr.cnes.sirius.patrius.events.sensor.SensorVisibilityDetector;
import fr.cnes.sirius.patrius.frames.Frame;
import fr.cnes.sirius.patrius.frames.FramesFactory;
import fr.cnes.sirius.patrius.frames.TopocentricFrame;
import fr.cnes.sirius.patrius.frames.transformations.Transform;
import fr.cnes.sirius.patrius.math.geometry.euclidean.threed.Vector3D;
import fr.cnes.sirius.patrius.math.util.FastMath;
import fr.cnes.sirius.patrius.math.util.MathLib;
import fr.cnes.sirius.patrius.propagation.analytical.KeplerianPropagator;
import fr.cnes.sirius.patrius.propagation.events.EventDetector;
import fr.cnes.sirius.patrius.propagation.events.ThreeBodiesAngleDetector;
import fr.cnes.sirius.patrius.time.AbsoluteDate;
import fr.cnes.sirius.patrius.time.AbsoluteDateInterval;
import fr.cnes.sirius.patrius.time.AbsoluteDateIntervalsList;
import fr.cnes.sirius.patrius.utils.exception.PatriusException;
import fr.cnes.sirius.patrius.orbits.pvcoordinates.PVCoordinates;
import fr.cnes.sirius.patrius.orbits.pvcoordinates.PVCoordinatesProvider;
import fr.cnes.sirius.patrius.assembly.models.SensorModel;
import fr.cnes.sirius.patrius.propagation.events.ConstantRadiusProvider;
import reader.Site;
import utils.ConstantsBE;
import utils.LogUtils;
import utils.ProjectUtils;




/**
 * This class implements the context of an Earth Observation mission.
 *
 * @author herberl
 */
public class CompleteMission extends SimpleMission {
	
	final Frame eme2000 = this.getEme2000();
	
	final ExtendedOneAxisEllipsoid earth = this.getEarth();
	
	public double ComputeSlewDurationBetweenObs(Map<TargetAccess,AttitudeLawLeg> mapAllAttitudeLegs , TargetAccess currenttargetaccess, TargetAccess othertargetaccess) throws PatriusException {
		
		final KeplerianPropagator propagator = createDefaultPropagator();
		
    	final AttitudeLeg currentSiteObsLeg = mapAllAttitudeLegs.get(currenttargetaccess);
    	final AbsoluteDate currentObsEnd = currentSiteObsLeg.getEnd();
    	final Attitude endCurrentObsAttitude = currentSiteObsLeg.getAttitude(propagator, currentObsEnd, this.getEme2000());
    	
    	final AbsoluteDate currentObsStart = currentSiteObsLeg.getDate();
    	final Attitude startCurrentObsAttitude = currentSiteObsLeg.getAttitude(propagator, currentObsStart, this.getEme2000());
    	
    	final AttitudeLeg otherSiteObsLeg = mapAllAttitudeLegs.get(othertargetaccess);
    	final AbsoluteDate otherObsEnd = otherSiteObsLeg.getEnd();
    	final Attitude endOtherObsAttitude = otherSiteObsLeg.getAttitude(propagator, otherObsEnd, this.getEme2000());
    	
    	final AbsoluteDate otherObsStart = otherSiteObsLeg.getDate();
    	final Attitude startOtherObsAttitude = otherSiteObsLeg.getAttitude(propagator, otherObsStart, this.getEme2000());
		
		double slewDurationEndCurrentToStartOther = this.getSatellite().computeSlewDuration(endCurrentObsAttitude, startOtherObsAttitude);
		double slewDurationEndOtherToStartCurrent = this.getSatellite().computeSlewDuration(endOtherObsAttitude, startCurrentObsAttitude);
		
		if (otherObsStart.compareTo(currentObsStart)<0) {
			return slewDurationEndOtherToStartCurrent; 			
		}
		
		else {
			return slewDurationEndCurrentToStartOther;
		}
	}
	
	/**
	 * This class implements a tool to compare the Accesses in order to choose them in the ObservationPlan.
	 *
	 */
	class TargetAccess implements Comparable<TargetAccess> {
		private AbsoluteDate midDate;
		private Site site;
		
		public AttitudeLawLeg MakeObsLeg() {
			
			// Creating an attitude leg for the whole mission
			final AttitudeLaw fullobservationLaw = createObservationLaw(this.site);
			

			// Reducing the time interval to a the minimum value for the integration time
			final AbsoluteDate obsStart = this.midDate.shiftedBy(-ConstantsBE.INTEGRATION_TIME / 2);
			final AbsoluteDate obsEnd = this.midDate.shiftedBy(ConstantsBE.INTEGRATION_TIME / 2);
			final AbsoluteDateInterval obsInterval = new AbsoluteDateInterval(obsStart, obsEnd);
			
			// Then, we create our AttitudeLawLeg, that we name using the name of the target
			final String legName = "OBS_" + site.getName();
			final AttitudeLawLeg obsLeg = new AttitudeLawLeg(fullobservationLaw, obsInterval, legName);

			return obsLeg;
			
		}

		public TargetAccess(AbsoluteDate midDate, Site site) {
			this.midDate = midDate;
			this.site = site;
		}

		public AbsoluteDate getmidDate() {
			return midDate;
		}

		public Site getSite() {
			return site;
		}

		public int compareDates(TargetAccess ot1, TargetAccess ot2) {
			return ot1.getmidDate().compareTo(ot2.getmidDate());
		}
		
		/**
		 * This methods computes the score of the access
		 * @return the access score {@link double}
		 * 
		 */
		public double computeAccessScore() {
			
			final AbsoluteDate midDate = this.getmidDate();
			
			final Site site = this.getSite();
			
			try {
			
			// Creating a new propagator to compute the satellite's pv coordinates
			final KeplerianPropagator propagator = createDefaultPropagator();

			// Calculating the satellite PVCoordinates at middleDate
			final PVCoordinates satPv1 = propagator.getPVCoordinates(midDate, eme2000);

			// Calculating the Site PVCoordinates at middleDate
			final TopocentricFrame siteFrame1 = new TopocentricFrame(earth, site.getPoint(), site.getName());
			final PVCoordinates sitePv1 = siteFrame1.getPVCoordinates(midDate, eme2000);

			// Calculating the normalized site-sat vector at middleDate
			final Vector3D siteSatVectorEme20001 = satPv1.getPosition().subtract(sitePv1.getPosition()).normalize();

			// Calculating the vector normal to the surface at the Site at middleDate
			final Vector3D siteNormalVectorEarthFrame1 = siteFrame1.getZenith();
			final Transform earth2Eme20001 = siteFrame1.getParentShape().getBodyFrame().getTransformTo(eme2000, midDate);
			final Vector3D siteNormalVectorEme20001 = earth2Eme20001.transformPosition(siteNormalVectorEarthFrame1);

			// Finally, we can compute the incidence angle = angle between
			// siteNormalVectorEme2000 and siteSatVectorEme2000
			final double incidenceAngle1 = Vector3D.angle(siteNormalVectorEme20001, siteSatVectorEme20001);
			
			final double score = site.getScore()*MathLib.cos(incidenceAngle1);
			
			return score;
			
			} catch (PatriusException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			return 0;
			
			
		}
		
		
		@Override
		public String toString() {
			return "target access [target=" + this.getSite().getName() + ", midDate=" + this.getmidDate() + "]";
		}

		/*
		* Implementation of the compareTo method that will be used to rank city accesses among themselves.
		* It might have been simpler for the greedy algorithm considering that the list of cities
		* is already well sorted, and it would have been enough to retrieve the access with the best incidence
		* once incompatible accesses were removed.
		* However, this way allows for more flexible entries.
		*/
		@Override
		public int compareTo(TargetAccess ot2){
			
			double scoreT1 = this.computeAccessScore();
			
			double scoreT2 = ot2.computeAccessScore();
			
			if (scoreT1<scoreT2) {return -1;}
			
			else if (scoreT1>scoreT2) {return 1;}
			
			else {return 0;}
			
		}
	}

	/**
	 * Logger for this class.
	 */
	private final  Logger logger = LogUtils.GLOBAL_LOGGER;
	
	/**
	 * Maximum checking interval (s) for the event detection during the orbit
	 * propagation.
	 */
	public static final double MAXCHECK_EVENTS = 120.0;

	/**
	 * Default convergence threshold (s) for the event computation during the orbit
	 * propagation.
	 */
	public static final double TRESHOLD_EVENTS = 1.e-4;

	/**
	 * This {@link Map} will be used to enumerate each site access {@link Timeline},
	 * that is to say a {@link Timeline} with access windows respecting all
	 * observation conditions. This object corresponds to the access plan, which
	 * will be computed in the computeAccessPlan() method.
	 */
	private final Map<Site, Timeline> accessPlan;

	/**
	 * This {@link Map} will be used to enumerate each site's programmed
	 * observation. We suggest to use an {@link AttitudeLawLeg} to encapsulate the
	 * guidance law of each observation. This object corresponds to the observation
	 * plan, which will be computed in the computeObservationPlan() method.
	 */
	private final Map<Site, AttitudeLawLeg> observationPlan;

	/**
	 * {@link StrictAttitudeLegsSequence} representing the cinematic plan during the
	 * whole mission horizon. Each {@link AttitudeLeg} corresponds to a diffrent
	 * attitude law : either nadir pointing, target pointing or a slew between two
	 * laws. This object corresponds to the cinematic plan, which will be computed
	 * in the computeCinematicPlan() method.
	 */
	private final StrictAttitudeLegsSequence<AttitudeLeg> cinematicPlan;


	/**
	 * Constructor for the {@link CompleteMission} class.
	 *
	 * @param missionName   Name of the mission
	 * @param numberOfSites Number of target {@link Site} to consider, please give a
	 *                      number between 1 and 100.
	 * @throws PatriusException      Can be raised by Patrius when building
	 *                               particular objects. Here it comes from
	 *                               {@link FramesFactory}
	 * @throws IllegalStateException if the mission horizon is too short
	 */
	public CompleteMission(final String missionName, int numberOfSites) throws PatriusException {

		// Since this class extends the SimpleMission class, we need to use the super
		// constructor to instantiate our instance of CompleteMission. All the
		// attributes of the super class will be instantiated during this step.
		super(missionName, numberOfSites);

		// Initialize the mission plans with empty maps. You will fill those HashMaps in
		// the "compute****Plan()" methods.
		this.accessPlan = new HashMap<>();
		this.observationPlan = new HashMap<>();
		this.cinematicPlan = new StrictAttitudeLegsSequence<>();
		

	}
	

	/**
	 * 
	 * Compute the access plan of the mission. This plan respect the 3 conditions for 
	 * each site (Illumination, Visibility, No Dazzling).
	 * 
	 * @return the sites access plan with one {@link Timeline} per {@link Site}
	 * @throws PatriusException If a {@link PatriusException} occurs during the
	 *                          computations
	 */
	public Map<Site, Timeline> computeAccessPlan() throws PatriusException {
		
		// For each site, creating the corresponding timeline, and then add it to the accessPlan
		for (Site targetSite : this.getSiteList()) {
			Timeline siteAccessTimeline = createSiteAccessTimeline(targetSite);
			this.accessPlan.put(targetSite, siteAccessTimeline);
		}
		
		return this.accessPlan;
	}

	/**
	 * 
	 * Computes the observation plan.
	 * 
	 * @return The chosen {@link AttitudeLawLeg} for each
	 *         {@link Site}
	 * @throws PatriusException si une {@link PatriusException} se produit 
	 * 			pendant le calcul
	 */
	public Map<Site, AttitudeLawLeg> computeObservationPlan() throws PatriusException {

		/*
		* The structure of the chosen algorithm requires a complete list of accesses:
		* targetAccesses, which will be built sorted in descending order of access scores
		* to perform a simple greedy approach.
		*
		* The targetObservations list will be filled later as we check
		* the compatibility of accesses while traversing the targetAccesses list.
		*
		* accessedSites will be populated simultaneously with targetObservations for simplicity
		* when checking whether a site has been visited previously or not.
		*
		* The HashMap mapAllAttitudeLegs, on the other hand, will be used to calculate durations between observations
		* for verifying access compatibility.
		*/
		
		// Declaring useful variables
		List<TargetAccess> targetAccesses = new ArrayList<>();
		
		List<TargetAccess> targetObservations = new ArrayList<>();
		
		List<Site> accessedSites = new ArrayList<>();
		
		List<Site> targets = this.getSiteList();
		
		Map<TargetAccess, AttitudeLawLeg> mapAllAttitudeLegs = new HashMap<>();
		

		/*
		* An initial traversal of the Sites and access windows allows us to construct the list and
		* the useful HashMap that we will subsequently sort.
		*/
		
		// Iterating on sites.
		for (Site target : targets)  {

			final Timeline timeline = this.accessPlan.get(target);
	
			for (final Phenomenon accessWindow : timeline.getPhenomenaList()) {
				// The Phenomena are sorted chronologically so the accessIntervals List is too
			    final AbsoluteDateInterval accessInterval = accessWindow.getTimespan();

				final AbsoluteDate debutAccess = accessInterval.getLowerData();
				final AbsoluteDate finAccess = accessInterval.getUpperData();
				
				final AbsoluteDate middleDate = accessInterval.getMiddleDate();
				
				final TargetAccess obsTarget = new TargetAccess(middleDate,target);
				
				if(finAccess.durationFrom(debutAccess)>= ConstantsBE.INTEGRATION_TIME) {
					
					/*
					* A simple check to verify that the access is compatible with the observation duration,
					* likely not necessary but makes the code more resilient in case of access
					* at the extreme edge of the visibility angle.
					*/
				
					targetAccesses.add(obsTarget);

					mapAllAttitudeLegs.put(obsTarget, obsTarget.MakeObsLeg());
				
				}
				
			}
		}
		
		/*
		* Sorting accesses according to the score taking into account the incidence
		*/
		
		Collections.sort(targetAccesses,Collections.reverseOrder());
		
		for (int current = 0 ; current < targetAccesses.size(); current++) {	
				
			/* Automatically fills the first element of the observations list with the 
			 * first city encountered
			 * AND is observable for more than ConstantsBE.INTEGRATION_TIME OR ELSE
			 * checks if the current access is compatible or not
			 * with the previously selected accesses based on the delay between the
			 * two and the time required to change attitude
			*/
			
			TargetAccess targetaccess=targetAccesses.get(current);
			
			Site target = targetaccess.getSite();
			
			AbsoluteDate middleDate = targetaccess.getmidDate();
				
			if (targetObservations.isEmpty()) {

				this.observationPlan.put(target, targetaccess.MakeObsLeg());
				
				targetObservations.add(targetaccess);
				accessedSites.add(target);
				
				logger.info("Target site : " + target.getName() + "   observed from :  " + 
				targetaccess.MakeObsLeg().getDate().toString() + "  to " + targetaccess.MakeObsLeg().getEnd().toString());
				
				
			}
			
			else if (!accessedSites.contains(target)){
				
				// By default, we assume the accesses are compatible, and we don't check
				boolean accesValide = true;
				
				for (int i = 0 ; i < targetObservations.size(); i++){
			
			    	double slewDuration = ComputeSlewDurationBetweenObs(mapAllAttitudeLegs, targetaccess, targetObservations.get(i));
			    	
			    	/*
			    	* If the gap between the two accesses is less than the time required to change the
			    	* pointing and perform the observation, then the access is marked as invalid
			    	*/
					
					if (Math.abs(middleDate.durationFrom(targetObservations.get(i).getmidDate()))<ConstantsBE.INTEGRATION_TIME+slewDuration) 
					{
						accesValide = false;
						break;
						
					}
					
				}
			
				if (accesValide){
	
					this.observationPlan.put(target, targetaccess.MakeObsLeg());
					
					targetObservations.add(targetaccess);
					accessedSites.add(target);
					
					logger.info("Target site : " + target.getName() + "   observed from :  " + 
					targetaccess.MakeObsLeg().getDate().toString() + "  to " + targetaccess.MakeObsLeg().getEnd().toString());
					
					}
				
				}
					
			}

		return this.observationPlan;
	}

	/**
	 * 
	 * 
	 * Computes the cinematic plan.
	 * 
	 * Here you need to compute the cinematic plan, which is the cinematic chain of
	 * attitude law legs (observation, default law and slews) needed to perform the
	 * mission. Usually, we start and end the mission in default law and during the
	 * horizon, we alternate between default law, observation legs and slew legs.
	 * 
	 * @return a {@link StrictAttitudeLegsSequence} that gives all the cinematic
	 *         plan of the {@link Satellite}. It is a chronological sequence of all
	 *         the {@link AttitudeLawLeg} that are necessary to define the
	 *         {@link Attitude} of the {@link Satellite} during all the mission
	 *         horizon. Those legs can have 3 natures : pointing a target site,
	 *         pointing nadir and performing a slew between one of the two previous
	 *         kind of legs.
	 * @throws PatriusException
	 */
	public StrictAttitudeLegsSequence<AttitudeLeg> computeCinematicPlan() throws PatriusException {

		// Here we compute the Cinematic Plan, based on the Observation Plan that need to be calculated before.
		
		
		// We create a list of site, to be considered as a sequence of observation to realize, chronologically ordered
		List<Site> sitesSeq = new ArrayList<>(this.observationPlan.keySet());
		
		// We sort the list, to be able to build a chronologically correct plan
		Collections.sort(sitesSeq, Comparator.comparing((Site site) -> this.observationPlan.get(site).getTimeInterval()));
		
		// Now we get some useful values for building the plan
		
		// Getting our Nadir law
	    final AttitudeLaw nadirLaw = this.getSatellite().getDefaultAttitudeLaw();

		// Getting all the dates we need to compute our slews
	    final AbsoluteDate start = this.getStartDate();
	    final AbsoluteDate end = this.getEndDate();
		
	    // Getting the number of iteration, that will be used as number of iterations.
		final int n = observationPlan.size();
		
		
		// Iterating on observations to plan
		
		for (int i = 0; i<n; i++) {
			
			
			// First, getting some useful values for all cases
			// Getting the i-th target site to observe
			final Site targetSite = sitesSeq.get(i);
			
			// Getting the associated observation leg defined in the observation plan for this site
		    final AttitudeLeg siteObsLeg = observationPlan.get(targetSite);
		    
		    //Getting the dates to compute our slews
		    final AbsoluteDate obsStart = siteObsLeg.getDate();
		    final AbsoluteDate obsEnd = siteObsLeg.getEnd();
			
		    // Getting the propagator that will be used to compute Attitudes
	    	final KeplerianPropagator propagator = this.createDefaultPropagator();
	    
		    /** Now, we distinguish the 3 differents cases:
		     *  - if i = 0, first observation to complete : need to add the initial Nadir law and the associated transition
		     *  - if i = n-1, last observation to complete : need to add the final Nadir Law and the associated transition
		     *  - elsewhere, "regular" observation to complete : just the slew to the observation and the observation
		     *  
		     * For the "regular" and the "n-1" case, if we have enough time (greater than 2 * getSatellite().getMaxSlewDuration()) to
		     * go back to a Nadir attitude, we do so. A "if" condition is there to check if it is the case.
		     *  
		     * 
		     * **/
	    	
	    	
	    	
		    if (i==0) {
		    	
		    	// First observation to complete
		    	// Getting useful attitudes
		    	final AbsoluteDate endNadirLawInitial = obsStart.shiftedBy(-getSatellite().getMaxSlewDuration());
		    	final Attitude endNadir1Attitude = nadirLaw.getAttitude(propagator, endNadirLawInitial, getEme2000());
		    	final Attitude startObsAttitude = siteObsLeg.getAttitude(propagator, obsStart, getEme2000());
			    
		    	
		    	// We create our two Nadir legs using the dates we computed
			    final AttitudeLawLeg nadirInitial = new AttitudeLawLeg(nadirLaw, start, endNadirLawInitial, "Nadir_Law_Initial");
			    // Computing the slew
				// From initial Nadir law to first site observation
			    final String slewInitialName = "Slew_Nadir_to_" + targetSite.getName();
			    final ConstantSpinSlew slewInitial = new ConstantSpinSlew(endNadir1Attitude, startObsAttitude, slewInitialName);
			    
				
				// Finally we can add all those legs to our cinametic plan, in the chronological order
				this.cinematicPlan.add(nadirInitial);
				this.cinematicPlan.add(slewInitial);
				this.cinematicPlan.add(siteObsLeg);
			    
			    
		    }
		    else if (i==n-1) {
		    	// Last observation to complete
		    	
		    	//Getting necessary info about last attitude of the last observation (to compute the slew from there)
		    	final Site lastTargetSite = sitesSeq.get(i-1);
		    	final AttitudeLeg lastSiteObsLeg = observationPlan.get(lastTargetSite);
		    	final AbsoluteDate lastObsEnd = lastSiteObsLeg.getEnd();
		    	final Attitude endLastObsAttitude = lastSiteObsLeg.getAttitude(propagator, lastObsEnd, getEme2000());
		    	
		    	// getting attitude for the current observation
		    	final Attitude startObsAttitude = siteObsLeg.getAttitude(propagator, obsStart, getEme2000());
			    final Attitude endObsAttitude = siteObsLeg.getAttitude(propagator, obsEnd, getEme2000());
			    
			    //Getting the info about final Nadir Law to reach at the end
		    	final AbsoluteDate startNadirLaw2 = obsEnd.shiftedBy(+getSatellite().getMaxSlewDuration());
		    	final Attitude startNadir2Attitude = nadirLaw.getAttitude(propagator, startNadirLaw2, getEme2000());
			    
		    	
		    	//If we have enough time we come back to Nadir between the two observations (this also ads Nadir laws between passes
		    	if(obsStart.durationFrom(lastObsEnd) > getSatellite().getMaxSlewDuration()* 2) {
		    		
		    		//Getting the info about the Nadir law to reach between the two observations
			    	final AbsoluteDate startNadirLawTemp = lastObsEnd.shiftedBy(+getSatellite().getMaxSlewDuration());
			    	final AbsoluteDate endNadirLawTemp = obsStart.shiftedBy(-getSatellite().getMaxSlewDuration());
			    	final Attitude startNadirTempAttitude = nadirLaw.getAttitude(propagator, startNadirLawTemp, getEme2000());
			    	
		    		// Computing the attitude legs
					// From last observation to temporary Nadir
				    final String nameSlewLastObs2Nadir = "Slew_" + lastTargetSite.getName() + "_to_Nadir";
				    final ConstantSpinSlew slewLastObs2Nadir = new ConstantSpinSlew(endLastObsAttitude, startNadirTempAttitude, nameSlewLastObs2Nadir);
				    
				    // Creating the temporary Nadir leg
				    final AttitudeLawLeg nadirTemp = new AttitudeLawLeg(nadirLaw,startNadirLawTemp,endNadirLawTemp , "Nadir_Law_Temp");

				    // From temporary Nadir to current observation
				    final String slewNameNadir2Obs = "Slew_Nadir_to_" + targetSite.getName();
				    final Attitude endNadirTempAttitude = nadirTemp.getAttitude(propagator, endNadirLawTemp, getEme2000());
				    final ConstantSpinSlew slewNadir2Obs = new ConstantSpinSlew(endNadirTempAttitude, startObsAttitude, slewNameNadir2Obs);
				   
				    // From current site observation to final nadir law 
				    final String slewNameFinal = "Slew_"+ targetSite.getName()+"_to_Nadir" ;
				    final ConstantSpinSlew slewFinal = new ConstantSpinSlew(endObsAttitude, startNadir2Attitude, slewNameFinal);

					// Creating the final Nadir legs using the dates we computed
				    final AttitudeLawLeg nadirFinal = new AttitudeLawLeg(nadirLaw, startNadirLaw2, end, "Nadir_Law_Final");
				    
				    
					// Finally we can add all those legs to our cinematic plan, in the chronological
					// order
				    this.cinematicPlan.add(slewLastObs2Nadir);
				    this.cinematicPlan.add(nadirTemp);
				    this.cinematicPlan.add(slewNadir2Obs);
					this.cinematicPlan.add(siteObsLeg);
					this.cinematicPlan.add(slewFinal);
					this.cinematicPlan.add(nadirFinal);
		    	} else {
		    		// The satellite doosn't have enough time to go back to Nadir between observations
		    		
				    // Computing the attitude legs
			    	// From last observation to current site observation
				    final String slewNameSite2Site = "Slew_" + lastTargetSite.getName() + "_to_" + targetSite.getName();
				    final ConstantSpinSlew slewSite2Site = new ConstantSpinSlew(endLastObsAttitude, startObsAttitude, slewNameSite2Site);
				    
					// From current site observation to final nadir law 
				    final String slewNameFinal = "Slew_"+ targetSite.getName()+"_to_Nadir" ;
				    final ConstantSpinSlew slewFinal = new ConstantSpinSlew(endObsAttitude, startNadir2Attitude, slewNameFinal);
	
					// Creating the final Nadir legs using the dates we computed
				    final AttitudeLawLeg nadirFinal = new AttitudeLawLeg(nadirLaw, startNadirLaw2, end, "Nadir_Law_Final");
				    
				    
				 // Finally we can add all those legs to our cinematic plan, in the chronological
					// order
				    this.cinematicPlan.add(slewSite2Site);
					this.cinematicPlan.add(siteObsLeg);
					this.cinematicPlan.add(slewFinal);
					this.cinematicPlan.add(nadirFinal);
			    	}

		    	
		    }
		    else {
		    	// The "regular" case (for an observation that is not the first neither the last)
		    	
		    	//Getting necessary info about last attitude of the last observation (to compute the slew from there)
		    	final Site lastTargetSite = sitesSeq.get(i-1);
		    	final AttitudeLeg lastSiteObsLeg = observationPlan.get(lastTargetSite);
		    	final AbsoluteDate lastObsEnd = lastSiteObsLeg.getEnd();
		    	final Attitude endLastObsAttitude = lastSiteObsLeg.getAttitude(propagator, lastObsEnd, getEme2000());
		    	
		    	//Getting attitude for the current observation
		    	final Attitude startObsAttitude = siteObsLeg.getAttitude(propagator, obsStart, getEme2000());
		    	
		    	//If we have enough time we go back to Nadir Attitude
		    	if(obsStart.durationFrom(lastObsEnd) > getSatellite().getMaxSlewDuration() * 2) {
		    		
		    		//Getting the info about the temporary Nadir law to achieve
			    	final AbsoluteDate startNadirLawTemp = lastObsEnd.shiftedBy(+getSatellite().getMaxSlewDuration());
			    	final AbsoluteDate endNadirLawTemp = obsStart.shiftedBy(-getSatellite().getMaxSlewDuration());
			    	final Attitude startNadirTempAttitude = nadirLaw.getAttitude(propagator, startNadirLawTemp, getEme2000());
			    	
		    		// Computing attitude legs
					// From last observation to temporary Nadir
				    final String nameSlewLastObs2Nadir = "Slew_" + lastTargetSite.getName() + "_to_Nadir";
				    final ConstantSpinSlew slewLastObs2Nadir = new ConstantSpinSlew(endLastObsAttitude, startNadirTempAttitude, nameSlewLastObs2Nadir);
				    
				    // Creating the temporary Nadir leg
				    final AttitudeLawLeg nadirTemp = new AttitudeLawLeg(nadirLaw,startNadirLawTemp,endNadirLawTemp , "Nadir_Law_Temp");

				    // From temporary Nadir to current observation
				    final String slewNameNadir2Obs = "Slew_Nadir_to_" + targetSite.getName();
				    final Attitude endNadirTempAttitude = nadirTemp.getAttitude(propagator, endNadirLawTemp, getEme2000());
				    final ConstantSpinSlew slewNadir2Obs = new ConstantSpinSlew(endNadirTempAttitude, startObsAttitude, slewNameNadir2Obs);
				   
					// Finally we can add all those legs to our cinematic plan, in the chronological
					// order
				    this.cinematicPlan.add(slewLastObs2Nadir);
				    this.cinematicPlan.add(nadirTemp);
				    this.cinematicPlan.add(slewNadir2Obs);
					this.cinematicPlan.add(siteObsLeg);
		    		
		    		
		    	}else {
		    		// The satellite doosn't have enough time to go back to Nadir between observations
				    
				    // Computing the attitude legs
					// From last observation to current site observation
				    final String slewName = "Slew_" + lastTargetSite.getName() + "_to_" + targetSite.getName();
				    final ConstantSpinSlew slew1 = new ConstantSpinSlew(endLastObsAttitude, startObsAttitude, slewName);
				    
					// Finally we can add all those legs to our cinametic plan, in the chronological
					// order
					this.cinematicPlan.add(slew1);
					this.cinematicPlan.add(siteObsLeg);
		    	}
		    
		    }
		    
		   
		}
		
		return this.cinematicPlan;
	}

	/**
	 * This method creates the accessibility Timeline for a given site
	 * 
	 * @param targetSite Input target {@link Site}
	 * @return The {@link Timeline} of all the access {@link Phenomenon} for the
	 *         input {@link Site}.
	 * @throws PatriusException If a {@link PatriusException} occurs.
	 */
	private Timeline createSiteAccessTimeline(Site targetSite) throws PatriusException {
		
		// In case there were already Event Detectors on the satellite, we clear them.
		this.getSatellite().getPropagator().clearEventsDetectors();
		
		
		// For each of the 3 constraints to respect, we create a Timeline of validity
		//Visibility
	    final Timeline timeline_visibility = createSiteVisibilityTimeline(targetSite);
	    //Illumination
	    final Timeline timeline_illumination = createSiteIlluminationTimeline(targetSite);
	    //Dazzling
	    final Timeline timeline_dazzling = createSiteDazzlingTimeline(targetSite);
	

	    // Combining timelines (keeping only the intersection of the tree)
	    
	    // Initiating a timeline
		final Timeline siteAccessTimeline = new Timeline(
				new AbsoluteDateInterval(this.getStartDate(), this.getEndDate()));
		
		// Adding the phenomena of all the considered timelines
		//Dazzling
		for (final Phenomenon phenom : timeline_dazzling.getPhenomenaList()) {
			siteAccessTimeline.addPhenomenon(phenom);
		}
		//Visibility
		for (final Phenomenon phenom : timeline_visibility.getPhenomenaList()) {
			siteAccessTimeline.addPhenomenon(phenom);
		}
		// Illumination
		for (final Phenomenon phenom : timeline_illumination.getPhenomenaList()) {
			siteAccessTimeline.addPhenomenon(phenom);
		}
		
		

		//Defininf criterions for performing the intersection
		// Visibility and Illumination
		final AndCriterion visibilityANDillumination = new AndCriterion("Visibility", "Illumination",
				"VisibilityAndIllumination", "Ensure that the targeted site is visible and illuminated");
		// Applying our criterion adds all the new phenonmena inside the global timeline
		visibilityANDillumination.applyTo(siteAccessTimeline);
		
		// No Dazzling
		final NotCriterion NoDazzling = new NotCriterion("Dazzling", 
				"NoDazzling", "Ensure that the sun doesnt' dazzle the sat");
		// Applying our criterion adds all the new phenonmena inside the global timeline
		NoDazzling.applyTo(siteAccessTimeline);
		
		// Visibility, Illumination and No Dazzling
		final AndCriterion FullVisu = new AndCriterion("VisibilityAndIllumination", "NoDazzling",
				"FullVisu", "Ensure that the targeted site is visible and illuminated and not dazzled");
		// Applying our criterion adds all the new phenonmena inside the global timeline
		FullVisu.applyTo(siteAccessTimeline);
		
		// Filtering among all elements, keeping the ones that respects all conditions
		final ElementTypeFilter obsConditionFilter = new ElementTypeFilter("FullVisu", false);
		// Finally, we filter the global timeline to keep only X1 AND X2 phenomena
		obsConditionFilter.applyTo(siteAccessTimeline);

		// Log the final access timeline associated to the current target
		logger.info("\n" + targetSite.getName());
		ProjectUtils.printTimeline(siteAccessTimeline);

		return siteAccessTimeline;
	}

	/**
	 * This method creates the Visibility Timeline for a given site
	 * 
	 * @param targetSite Input target {@link Site}
	 * @return The {@link Timeline} containing all the {@link Phenomenon} relative
	 *         to the X phenomenon to monitor.
	 * @throws PatriusException If a {@link PatriusException} occurs when creating
	 *                          the {@link Timeline}.
	 */
	private Timeline createSiteVisibilityTimeline(Site targetSite) throws PatriusException {
	    
		// Creating a visibility detector to provide to the propagator and track the events 
		EventDetector visibilityDetector = createConstraintVisibilityDetector(targetSite);
		
		// Adding it to a new propagator
		this.createDefaultPropagator().addEventDetector(visibilityDetector);
		
		//Implementing the monitoring of detection of visibility
		final GenericCodingEventDetector codingVisibilityDetector = new GenericCodingEventDetector(visibilityDetector,
				"Visibility starting", "Visibility ending", true, "Visibility");
		final CodedEventsLogger visibilityLogger = new CodedEventsLogger();
		final EventDetector visibilityMonitorDetector = visibilityLogger.monitorDetector(codingVisibilityDetector);
		
		// Adding the Monitor detector to the propagator
		this.getSatellite().getPropagator().addEventDetector(visibilityMonitorDetector);

		// Finally propagating the orbit, this will store the info in the logger
		this.getSatellite().getPropagator().propagate(this.getStartDate(), this.getEndDate());
		
		// Creating a Timeline from the visibilityLogger
		final Timeline VisibilityTimeline = new Timeline(visibilityLogger,
				new AbsoluteDateInterval(this.getStartDate(), this.getEndDate()), null);

		return VisibilityTimeline;
	}
	

	
	
	/**
	 * This method creates the Dazzling Timeline for a given site
	 * 
	 * @param targetSite Input target {@link Site}
	 * @return The {@link Timeline} containing all the {@link Phenomenon} relative
	 *         to the X phenomenon to monitor.
	 * @throws PatriusException If a {@link PatriusException} occurs when creating
	 *                          the {@link Timeline}.
	 */
	private Timeline createSiteDazzlingTimeline(Site targetSite) throws PatriusException {
		
		// Creating an event detector
	    final EventDetector constraint_Dazzling_Detector = createConstraintDazzlingDetector(targetSite);
	    
	    // Adding the event detector to a new propagator
	    this.createDefaultPropagator().addEventDetector(constraint_Dazzling_Detector);

	    //Implementing the monitoring of detection of dazzling
		final GenericCodingEventDetector codingDazzlingDetector = new GenericCodingEventDetector(constraint_Dazzling_Detector,
				"Dazzling starting", "Dazzling ending", true, "Dazzling");
		final CodedEventsLogger eventDazzlingLogger = new CodedEventsLogger();
		final EventDetector eventDazzlingDetector = eventDazzlingLogger.monitorDetector(codingDazzlingDetector);

		// Adding it to the propagator
		this.getSatellite().getPropagator().addEventDetector(eventDazzlingDetector);

		// Finally propagating the orbit, this will store the info in the logger
		this.getSatellite().getPropagator().propagate(this.getStartDate(), this.getEndDate());
		
		// Creating the timeline from the logger
		final Timeline phenomenonDazzlingTimeline = new Timeline(eventDazzlingLogger,
				new AbsoluteDateInterval(this.getStartDate(), this.getEndDate()), null);

		return phenomenonDazzlingTimeline;
	}

/**
	 * This method creates the Illumination Timeline for a given site
	 * 
	 * @param targetSite Input target {@link Site}
	 * @return The {@link Timeline} containing all the {@link Phenomenon} relative
	 *         to the X phenomenon to monitor.
	 * @throws PatriusException If a {@link PatriusException} occurs when creating
	 *                          the {@link Timeline}.
	 */
	private Timeline createSiteIlluminationTimeline(Site targetSite) throws PatriusException {
		
		// Creating an event detector for illumination
	    final EventDetector constraintIlluminationDetector = createConstraintIlluminationDetector(targetSite);

		// Adding it a a new propagator
	    this.createDefaultPropagator().addEventDetector(constraintIlluminationDetector);

	    //Implementing the monitoring of detection of illumination
		final GenericCodingEventDetector codingEventIlluminationDetector = new GenericCodingEventDetector(constraintIlluminationDetector,
				"Start of illumination", "End of illumination", true, "Illumination");
		final CodedEventsLogger eventIlluminationLogger = new CodedEventsLogger();
		final EventDetector eventIlluminationDetector = eventIlluminationLogger.monitorDetector(codingEventIlluminationDetector);
		
		//Adding it to the propagator
		this.getSatellite().getPropagator().addEventDetector(eventIlluminationDetector);

		// Finally propagating the orbit, this will store the info in the logger
		this.getSatellite().getPropagator().propagate(this.getStartDate(), this.getEndDate());
		
		//Creating a timeline from the logger
		final Timeline phenomenonIlluminationTimeline = new Timeline(eventIlluminationLogger,
				new AbsoluteDateInterval(this.getStartDate(), this.getEndDate()), null);

		return phenomenonIlluminationTimeline;
	}
	
	/**
	 *  This method creates an visibility detector
	 * 
	 * @param targetSite of interest
	 * @return An {@link EventDetector} answering the constraint of visibility
	 */

	private EventDetector createConstraintVisibilityDetector(Site targetSite) {
		
		// Creating a Sensor model
		SensorModel visibilitySensorModel = new SensorModel(this.getSatellite().getAssembly(), Satellite.SENSOR_NAME);
		
		// Adding Earth as a masking Celestial Body
		visibilitySensorModel.addMaskingCelestialBody(this.getEarth());
		
		// Getting location of the target site
		PVCoordinatesProvider sitePVCoordinates = new TopocentricFrame(
				this.getEarth(),
				targetSite.getPoint(),
				targetSite.getName()
		);
		
		// Implementing the Sensor model with the location of the target site
		visibilitySensorModel.setMainTarget(sitePVCoordinates, new ConstantRadiusProvider(0.0));

		
		return new SensorVisibilityDetector(visibilitySensorModel,
				MAXCHECK_EVENTS, TRESHOLD_EVENTS, EventDetector.Action.CONTINUE, EventDetector.Action.CONTINUE);

	}

	/**
	 * This method creates an Illumination detector
	 * @param targetSite of interest
	 * @return An {@link EventDetector} answering the constraint of Illumination.
	 */
	private EventDetector createConstraintIlluminationDetector(Site targetSite) {
		
		// Getting the target site location
		PVCoordinatesProvider siteCoordinates = new TopocentricFrame(
				this.getEarth(),
				targetSite.getPoint(),
				targetSite.getName());
		
		// Computing the angle of illumination given the constraint to satisfy
		final double angleIllumination = FastMath.toRadians(180 - ConstantsBE.MAX_SUN_INCIDENCE_ANGLE);
		
		// Creating the event detector for the angle of illumination
		EventDetector incidenceAngleDetector = new ThreeBodiesAngleDetector(this.getEarth(), siteCoordinates, this.getSun(), angleIllumination, MAXCHECK_EVENTS, TRESHOLD_EVENTS, EventDetector.Action.CONTINUE );
		
		return incidenceAngleDetector;
	}

	/**
	 * This method creates a Dazzling Detector
	 * @param targetSite of interest
	 * @return An {@link EventDetector} answering the constraint (for example a
	 *         {@link SensorVisibilityDetector} for a visibility constraint).
	 */
	private EventDetector createConstraintDazzlingDetector(Site targetSite) {
		
		// Getting the target site location
		PVCoordinatesProvider sitePVCoordinates = new TopocentricFrame(
				this.getEarth(),
				targetSite.getPoint(),
				targetSite.getName()
		);
		
		//Creating the event detector
		ThreeBodiesAngleDetector Dazzling_detector = new ThreeBodiesAngleDetector(sitePVCoordinates, this.getSun(),ThreeBodiesAngleDetector.BodyOrder.FIRST,FastMath.toRadians(ConstantsBE.MAX_SUN_PHASE_ANGLE), MAXCHECK_EVENTS, TRESHOLD_EVENTS, EventDetector.Action.CONTINUE);
		
		return Dazzling_detector;
	}

	/**
	 * 
	 * Create an observation leg, that is to say an {@link AttitudeLaw} that give
	 * the {@link Attitude} (pointing direction) of the {@link Satellite} in order
	 * to perform the observation of the input target {@link Site}.
	 * 
	 * An {@link AttitudeLaw} is an {@link AttitudeProvider} providing the method
	 * {@link AttitudeProvider#getAttitude()} which can be used to compute the
	 * {@link Attitude} of the {@link Satellite} at any given {@link AbsoluteDate}
	 * (instant) during the mission horizon.
	 * 
	 * An {@link AttitudeLaw} is valid at anu time in theory.
	 * 
	 * @param target Input target {@link Site}
	 * @return An {@link AttitudeLawLeg} adapted to the observation.
	 */
	private AttitudeLaw createObservationLaw(Site targetSite) {

		// Creating the observation leg for the satellite pointing the target site
		TargetGroundPointing targetGroundPointing = new TargetGroundPointing(this.getEarth(), targetSite.getPoint(), Vector3D.MINUS_K, Vector3D.PLUS_I);

		return targetGroundPointing;
		
	}

	
	
	/**
	 * @return the accessPlan
	 */
	public Map<Site, Timeline> getAccessPlan() {
		return this.accessPlan;
	}

	/**
	 * @return the observationPlan
	 */
	public Map<Site, AttitudeLawLeg> getObservationPlan() {
		return this.observationPlan;
	}

	/**
	 * @return the cinematicPlan
	 */
	public StrictAttitudeLegsSequence<AttitudeLeg> getCinematicPlan() {
		return this.cinematicPlan;
	}

	@Override
	public String toString() {
		return "CompleteMission [name=" + this.getName() + ", startDate=" + this.getStartDate() + ", endDate="
				+ this.getEndDate() + ", satellite=" + this.getSatellite() + "]";
	}
}