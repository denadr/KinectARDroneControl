namespace KinectDroneControl.KinectDroneInterface
{
    class KinectInputState
    {
        public float SpeedUp;
        public float Roll;
        public float Rotate;
        public float Ascend;
        public bool Flying;
        public string Flag;
        public KinectInputState(bool flies, float speed = 0, float roll = 0, 
                                float rotate = 0, float ascend = 0, string flag = "")
        {
            SpeedUp = speed;
            Roll = roll;
            Rotate = rotate;
            Ascend = ascend;
            Flying = flies;
            Flag = flag;
        }
    }
}
