using System.Collections.Generic;
using WindowsPreview.Kinect;

namespace KinectDroneControl.KinectDroneInterface
{
    class JoystickTranslator : IGestureTranslator
    {
        public KinectInputState Translate(Dictionary<JointType, CameraSpacePoint> jointPositions, bool flying, HandState leftState, HandState rightState, CoordinateMapper mapper)
        {
            var state = new KinectInputState(flying);

            // TODO

            return state;
        }
    }
}
