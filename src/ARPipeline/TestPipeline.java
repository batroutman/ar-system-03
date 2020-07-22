package ARPipeline;

import org.opencv.features2d.FlannBasedMatcher;
import org.opencv.features2d.ORB;
import org.opencv.core.MatOfKeyPoint;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;

public class TestPipeline extends ARPipeline{
	
	double TOLERANCE = 220.0;
	double SEARCH_BOX_WIDTH = 60.0;
	
	ArrayList<KeyFrame> keyframes = new ArrayList<KeyFrame>();
	KeyFrame currentKeyFrame = null;
	ArrayList<Correspondence> lastCorrespondences = null;
	
	float rotAngle = 0;
	Pose pose = new Pose();
	
	protected Thread mainThread = new Thread() {
		@Override
		public void run() {
			mainloop();
		}
	};

	public TestPipeline(){
		super();
	}

	public TestPipeline(FrameBuffer inputFrameBuffer) {
		super();
		this.setInputFrameBuffer(inputFrameBuffer);
	}

	public TestPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer) {
		super();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
	}
	
	public TestPipeline(FrameBuffer inputFrameBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public TestPipeline(FrameBuffer inputFrameBuffer, PoseBuffer outputPoseBuffer, FrameBuffer outputFrameBuffer) {
		super();
		this.setInputFrameBuffer(inputFrameBuffer);
		this.setOutputPoseBuffer(outputPoseBuffer);
		this.setOutputFrameBuffer(outputFrameBuffer);
	}

	public void start() {
		this.mainThread.start();
	}
	public void stop() {
		this.mainThread.interrupt();
	}
	
	public static KeyFrame generateKeyFrame(Mat descriptors, MatOfKeyPoint keypoints) {
		
		KeyFrame keyframe = new KeyFrame();
		Pose pose = new Pose();
		pose.setOrigin();
		keyframe.setPose(pose);
		keyframe.setDescriptorsMat(descriptors);
		
		List<KeyPoint> keypointList = keypoints.toList();
		MapPoint firstMp = new MapPoint(descriptors.row(0));
		// may have issues here. negative y? offset?
		firstMp.setUV(keypointList.get(0).pt.x, keypointList.get(0).pt.y);
		MapPointKDTree tree = new MapPointKDTree(firstMp);
		
		for (int i = 1; i < keypointList.size(); i++) {
			MapPoint mp = new MapPoint(descriptors.row(i));
			mp.setUV(keypointList.get(i).pt.x, keypointList.get(i).pt.y);
			tree.insert(mp);
		}
		
		System.out.println("depth: " + tree.depth());
		
		keyframe.setDescriptors(tree);
		return keyframe;
		
	}
	
	public ArrayList<Correspondence> getCorrespondences(KeyFrame keyframe, Mat descriptors, MatOfKeyPoint keypoints, double tolerance) {
		
		ArrayList<Correspondence> correspondences = new ArrayList<Correspondence>();
		List<KeyPoint> keypointList = keypoints.toList();
		
		// if we don't have any correspondences for the last frame, try to get correspondences directly from keyframe
		if (this.lastCorrespondences == null) {
			for (int rowIndex = 0; rowIndex < descriptors.rows(); rowIndex++) {
				Mat row = descriptors.row(rowIndex);
				MapPoint nearest = keyframe.getDescriptors().nearest(row);
				if (ARUtils.descriptorDistance(row, nearest.getDescriptor()) <= tolerance) {
					Correspondence c = new Correspondence();
					c.setMapPoint(nearest);
					c.setLastFrameDescriptor(row);
					c.setLastFrameU(keypointList.get(rowIndex).pt.x);
					c.setLastFrameV(keypointList.get(rowIndex).pt.y);
					correspondences.add(c);
				}
			}
		}
		// if we do have correspondences for the last frame, attempt to match correspondences based on position
		else {
			for (Correspondence lastC:this.lastCorrespondences) {
				MapPoint bestMP = null;
				Double bestDist = null;
				Mat bestDescriptor = null;
				Double bestU = null;
				Double bestV = null;
				for (int pointIndex = 0; pointIndex < keypointList.size(); pointIndex++) {
					if (Math.abs(lastC.lastFrameU - keypointList.get(pointIndex).pt.x) > SEARCH_BOX_WIDTH || Math.abs(lastC.lastFrameV - keypointList.get(pointIndex).pt.y) > SEARCH_BOX_WIDTH) continue;
					Mat row = descriptors.row(pointIndex);
					MapPoint candidateMP = lastC.getMapPoint();
					double dist = ARUtils.descriptorDistance(row, candidateMP.getDescriptor());
					if (dist <= tolerance && (bestDist == null || dist < bestDist)) {
						bestMP = candidateMP;
						bestDist = dist;
						bestDescriptor = row;
						bestU = keypointList.get(pointIndex).pt.x;
						bestV = keypointList.get(pointIndex).pt.y;
					}
				}
				if (bestMP != null) {
					Correspondence c = new Correspondence();
//					bestMP.setDescriptor(bestDescriptor);
					c.setMapPoint(bestMP);
					c.setLastFrameDescriptor(bestDescriptor);
					c.setLastFrameU(bestU);
					c.setLastFrameV(bestV);
					correspondences.add(c);
				}
			}
		}
		
		
		return correspondences;
		
	}
	
	Mat oldDesc = new Mat();
	int count  = 0;
	protected void mainloop() {
		Frame currentFrame = this.inputFrameBuffer.getCurrentFrame();
		boolean keepGoing = true;
		while (keepGoing) {
			
			if (currentFrame == null) {
				keepGoing = false;
				
				getStats(oldDesc);
				
				continue;
			}
			
			// find ORB features
			int patchSize = 5;
			ORB orb = ORB.create(100);
			orb.setScoreType(ORB.FAST_SCORE);
			orb.setPatchSize(patchSize);
			orb.setNLevels(1);
			MatOfKeyPoint keypoints = new MatOfKeyPoint();
			Mat descriptors = new Mat();
			Mat image = ARUtils.frameToMat(currentFrame);
			orb.detectAndCompute(image, new Mat(), keypoints, descriptors);
			descriptors.convertTo(descriptors, CvType.CV_32F);
			ARUtils.boxHighlight(currentFrame, keypoints, patchSize);
			oldDesc = descriptors;
			
			// what do I want to do?
			// find correspondences between descriptors and mappoints in used in current keyframe
			//		a. if I have 0 keyframes, generate one and set pose to origin. continue.
					if (this.keyframes.size() == 0) {
						this.currentKeyFrame = generateKeyFrame(descriptors, keypoints);
						this.keyframes.add(this.currentKeyFrame);
					}
			//		b. otherwise, 
					else {
						// calculate correspondences
						FlannBasedMatcher flann = FlannBasedMatcher.create();
						ArrayList<MatOfDMatch> matches = new ArrayList<MatOfDMatch>();
						flann.knnMatch(descriptors, this.currentKeyFrame.getDescriptorsMat(), matches, 1);
						System.out.println("matches.get(0).toList(): " + matches.get(0).toList().get(0).queryIdx + ", " + matches.get(0).toList().get(0).trainIdx + ", " + matches.get(0).toList().get(0).distance);
						System.out.println("matches.size(): " + matches.size());
						
			// 			if I have at least 30(?) correspondences, then simply compute 5-point sfm with them. continue

							// 5-point sfm

			//			i. else (I don't have at least 30(?) correspondences), but I have at least 10, then generate new keyframe and use the same correspondences for sfm. set current keyframe to new keyframe. continue.

							// 5-point sfm (with current correspondences)

			//			ii. else (<10 correspondences), check other keyframes for keyframe with most correspondences. If >= 10 correspondences, set as current keyframe and use the correspondences for sfm. if < 40, generate new keyframe.
			// 			iii. else, tracking lost (handle this later)
					}
			
			
			// rotate cube as demo that pose can be modified and displayed
			rotAngle += 0.002f;
			pose.setR11((float)Math.cos(rotAngle));
			pose.setR22((float)Math.cos(rotAngle));
			pose.setR12(-(float)Math.sin(rotAngle));
			pose.setR21((float)Math.sin(rotAngle));
			
			synchronized (this.outputPoseBuffer) {
				this.outputPoseBuffer.pushPose(pose);
			}
			
			// for demo, just push the unaltered frame along to the output buffer
			synchronized (this.outputFrameBuffer) {
				this.outputFrameBuffer.pushFrame(currentFrame);
			}

			currentFrame = this.inputFrameBuffer.getCurrentFrame();
			
			
			
		}
	}
	
	public void getStats(Mat desc) {
		for (int i = 0; i < desc.cols(); i++) {
			ArrayList<Double> column = new ArrayList<Double>();
			for (int j = 0; j < desc.rows(); j++) {
				column.add(desc.get(j, i)[0]);
			}
			System.out.println("====  " + i + "  ===============================");
			System.out.println("Min    : " + ARUtils.min(column));
			System.out.println("Max    : " + ARUtils.max(column));
			System.out.println("Average: " + ARUtils.mean(column));
			System.out.println("Std Dev: " + ARUtils.stdDev(column));
		}
	}
	
	

}