using System.Collections.Generic;
using WindowsPreview.Kinect;

namespace KinectDroneControl.KinectDroneInterface
{
    interface IGestureTranslator
    {
        KinectInputState Translate(
            Dictionary<JointType, CameraSpacePoint> jointPositions, 
            bool flying,
            HandState leftState,
            HandState rightState,
            CoordinateMapper mapper);
    }
}
