using Windows.UI;
using WindowsPreview.Kinect;

namespace KinectDroneControl.Extensions.Kinect
{
    static class HandStateUtilities
    {
        public static Color HandStateToColor(HandState handState)
        {
            switch (handState)
            {
                case HandState.Open:
                    return Colors.Green;
                case HandState.Closed:
                    return Colors.Red;
                case HandState.Lasso:
                    return Colors.Blue;
            }
            return Colors.Transparent;
        }
    }
}
