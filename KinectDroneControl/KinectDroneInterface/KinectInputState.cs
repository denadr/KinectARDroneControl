namespace KinectDroneControl.KinectDroneInterface
{
    class KinectInputState
    {
        public float SpeedUp;
        public float Roll;
        public float Rotate;
        public float Ascend;
        public bool Flying;
        public InputStateMode Mode;
        public KinectInputState(bool flies, float speed = 0, float roll = 0, 
                                float rotate = 0, float ascend = 0, InputStateMode mode = InputStateMode.Control)
        {
            SpeedUp = speed;
            Roll = roll;
            Rotate = rotate;
            Ascend = ascend;
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
