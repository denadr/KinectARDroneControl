namespace KinectDroneControl.Extensions.ArDrone
{
    static class Constants
    {
        public static float Ascend = 0.1f;
        public static float Speed  = 0.1f;
        public static float Rotate = 0.1f;
        public static float Roll   = 0.1f;

        public static void Reset()
        {
            Ascend = 0.1f;
            Speed = 0.1f;
            Rotate = 0.1f;
            Roll = 0.1f;
        }
    }
}
