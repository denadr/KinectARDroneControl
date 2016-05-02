namespace KinectDroneControl.Extensions.ArDrone
{
    static class Constants
    {
        public static float Gaz = 0.1f;
        public static float Pitch  = 0.1f;
        public static float Yaw = 0.1f;
        public static float Roll   = 0.1f;

        public static void Reset()
        {
            Gaz = 0.1f;
            Pitch = 0.1f;
            Yaw = 0.1f;
            Roll = 0.1f;
        }
    }
}
