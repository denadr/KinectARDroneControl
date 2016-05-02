namespace KinectDroneControl.KinectDroneInterface
{
    class KinectInputState
    {
        public float Pitch;
        public float Roll;
        public float Yaw;
        public float Gaz;

        public bool Flying;
        public InputStateMode Mode;

        public KinectInputState(bool flies, float pitch = 0, float roll = 0, 
                                float yaw = 0, float gaz = 0, InputStateMode mode = InputStateMode.Control)
        {
            Pitch = pitch;
            Roll = roll;
            Yaw = yaw;
            Gaz = gaz;
            Flying = flies;
            Mode = mode;
        }
    }

    enum InputStateMode
    {
        Takeoff,
        Land,
        Control
    }
}
