package ARPipeline;

public abstract class ARPipeline {

	public ARPipeline() {}

	protected FrameBuffer inputFrameBuffer = null;
	protected PoseBuffer outputPoseBuffer = null;
	protected FrameBuffer outputFrameBuffer = null;

	public abstract void start();
	public abstract void stop();

	public FrameBuffer getInputFrameBuffer() {
		return inputFrameBuffer;
	}

	public void setInputFrameBuffer(FrameBuffer inputFrameBuffer) {
		this.inputFrameBuffer = inputFrameBuffer;
	}

	public PoseBuffer getOutputPoseBuffer() {
		return outputPoseBuffer;
	}

	public void setOutputPoseBuffer(PoseBuffer outputPoseBuffer) {
		this.outputPoseBuffer = outputPoseBuffer;
	}

	public FrameBuffer getOutputFrameBuffer() {
		return outputFrameBuffer;
	}

	public void setOutputFrameBuffer(FrameBuffer outputFrameBuffer) {
		this.outputFrameBuffer = outputFrameBuffer;
	}

	

}