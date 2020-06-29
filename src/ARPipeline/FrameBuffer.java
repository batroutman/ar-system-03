package ARPipeline;

public interface FrameBuffer {

	public void pushFrame(Frame frame);
	public Frame getCurrentFrame();

}