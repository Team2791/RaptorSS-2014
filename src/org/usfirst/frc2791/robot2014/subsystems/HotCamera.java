package org.usfirst.frc2791.robot2014.subsystems;

import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;

/**
 *
 * @author mwtidd
 */
public class HotCamera{
    
    /**
     * FILTER SETTINGS
     */
    
    //Minimum area of particles to be considered
    private final int AREA_MINIMUM = 150;
    
    //Maximum number of particles to process
    private final int MAX_PARTICLES = 8;
    
    //Color settings for particle filtering
    private final int RED_LOW = 0;
    private final int RED_HIGH = 100;
    private final int GREEN_LOW = 110;
    private final int GREEN_HIGH = 255;
    private final int BLUE_LOW = 235;
    private final int BLUE_HIGH = 255;
    
    
    
    /**
     * TARGET SETINGS
     */
    
    //Mimum width the height ratio of valid targets;
    private final int MIN_WIDTH_TO_HEIGHT_RATIO = 3;
    
    //The minimum width of a valid target;
    private final int MIN_WIDTH = 80;
    
    //Minimum number of positive hot samples
    private final int MIN_POSITIVE_SAMPLES = 3;
    
    private final int MIN_EXPOSURE = 0;
    private final int MAX_EXPOSURE = 0;
    
    public static CriteriaCollection criteria;
    public static AxisCamera camera;
    
    private static ColorImage colorImage;
    private static BinaryImage thresholdImage;
    private static BinaryImage filteredImage;
    
    private static int positiveSampleCount = 0;
    
    private boolean initialized = false;
    
    public void initialize(){
        if(initialized){
            ExceptionLogger.logAlreadyInitalizedWarning();
            return; 
        }
        camera = AxisCamera.getInstance();  // get an instance of the camera
        //once the camera is overexposed save the exposure setting
        if(camera.getExposureControl().value > MIN_EXPOSURE && 
                camera.getExposureControl().value < MAX_EXPOSURE){
            camera.writeExposureControl(AxisCamera.ExposureT.hold);
        }
        
        criteria = new CriteriaCollection();      // create the criteria for the particle filter
        criteria.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
        initialized = true;
    }
    
    public boolean isHot(){
        //return hot if enough samples have passed as hot
        if(positiveSampleCount >= MIN_POSITIVE_SAMPLES){
            return true;
        }
        
        try{
            //check to see if this sample is hot, if it is increment the positive sample count
            if(determineHotness()){
                positiveSampleCount++;
            }
        }finally{
            /**
            * all images in Java must be freed after they are used since they are allocated out
            * of C data structures. Not calling free() will cause the memory to accumulate over
            * each pass of this loop.
            */
            try{
                if(filteredImage != null){
                    filteredImage.free();
                }

                if(thresholdImage != null){
                    thresholdImage.free();
                }

                if(colorImage != null){
                    colorImage.free();
                }
            }catch (NIVisionException ex) {
                ExceptionLogger.logFreeImageException();
            }  
        }
        
        return false;
    }
    
    private boolean determineHotness(){
        if(!initialized){
            ExceptionLogger.logNotInitalizedWarning();
            initialize();
            return false;
        }

        //get the image from the camera
        colorImage = getColorImage();
        if(colorImage == null) return false;
        
        //apply the color filter
        thresholdImage = getThresholdImage(colorImage);
        if(thresholdImage == null) return false;

        //filter out particles that are less than the mimimum size
        filteredImage = getFilteredImage(thresholdImage);
        if(filteredImage == null) return false;

        //get the count of paticles after the filters are applied
        int particleCount = getNumberParticles(filteredImage);
        if(particleCount == 0) return false;

        //loop through each of the partcles and see if you find one that is wide enought to be hot
        for (int i = 0; i < MAX_PARTICLES && i < particleCount; i++) {

            ParticleAnalysisReport report = getParticleAnalysisReport(filteredImage, i);
            if(report == null){
                //unable to get a report for this particle so skip it
                continue;
            }
                
            
            double widthToHeightRatio = report.boundingRectWidth / report.boundingRectHeight;
            //the target is roughly 24" x 4" or a 6 : 1 ratio
            //skip particles that have a ration of less than 3:1
            if(widthToHeightRatio < MIN_WIDTH_TO_HEIGHT_RATIO ){
                continue;
            }
            
            //make sure the target is at least a certain number of pixels wide
            if(report.boundingRectWidth > MIN_WIDTH){
                //found a target that meets the criteria
                return true;
            }
        }

        //no target matching the criteria was found
        return false;
    }
    
    /**
     * A wrapper for getting the image from the camera.
     * Handles exceptions gracefully.
     * 
     * 
     * @return the image from the camera, null if something when wrong
     */
    public ColorImage getColorImage(){
        try{
            return camera.getImage();
        } catch (AxisCameraException ex) {
            ExceptionLogger.logGetImageException(ex.getClass().getName());
            return null;
        } catch (NIVisionException ex) {
            ExceptionLogger.logGetImageException(ex.getClass().getName());
            return null;
        }
    }
    
    /**
     * A wrapper function for applying color filters.
     * Handles exceptions gracefully. 
     * 
     * @param image the image from the camera
     * @return an image with a RGB color filter applied, null if something when wrong
     */
    private BinaryImage getThresholdImage(ColorImage image){
        try{
            return image.thresholdRGB(RED_LOW,RED_HIGH,GREEN_LOW,GREEN_HIGH,BLUE_LOW,BLUE_HIGH);
        } catch (NIVisionException ex) {
            ExceptionLogger.logParticleFilterException();
            return null;
        } 
    }
    
    /**
     * A wrapper for applying particle size filters.
     * Handles exceptions gracefully
     * 
     * @param image the image with the color filter applied
     * @return an image filtering out small particles, null if something when wrong
     */
    private BinaryImage getFilteredImage(BinaryImage image){
        try{
            // filter out small particles
            return image.particleFilter(criteria);
         } catch (NIVisionException ex) {
            ExceptionLogger.logParticleFilterException();
            return null;
        }  
    }
    
    /**
     * A wrapper function for counting the number of particles.
     * Handles exceptions gracefully
     * 
     * @param filteredImage the image with both color and particle size filters applied
     * @return the number of particles in the filtered image, 0 if something when wrong
     */
    private int getNumberParticles(BinaryImage filteredImage){
        try {
            return filteredImage.getNumberParticles();
        } catch (NIVisionException ex) {
            ExceptionLogger.logParticleCountException();
            return 0;
        }
    }
    
    /**
     * A wrapper function for counting the number of particles.
     * Handles exceptions gracefully
     * 
     * @param filteredImage the image with both color and particle size filters applied
     * @param particleNumber the index of the current particle in the filtered image
     * @return a report with information about that particle, null if something when wrong
     */
    private ParticleAnalysisReport getParticleAnalysisReport(BinaryImage filteredImage, int particleNumber){
        try {
            return filteredImage.getParticleAnalysisReport(particleNumber);
        } catch (NIVisionException ex) {
            ExceptionLogger.logParticleReportExcepeiton(particleNumber);
            return null;
        }
    }

}
