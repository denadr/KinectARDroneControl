using System.Collections.Generic;
using Windows.Foundation;
using WindowsPreview.Kinect;

namespace KinectDroneControl.KinectDroneInterface
{
    interface IGestureTranslator
    {
        KinectInputState Translate(
            HandState leftState, 
            HandState rightState, 
            Dictionary<JointType, Point> jointPositions, 
            bool flying);
    }
}
