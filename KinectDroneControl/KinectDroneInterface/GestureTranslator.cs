using System;
using System.Collections.Generic;
using Windows.Foundation;
using WindowsPreview.Kinect;

namespace KinectDroneControl.KinectDroneInterface
{
    class GestureTranslator : IGestureTranslator
    {
        private readonly float r_DefaultAscendValue = 0.5f;
        private readonly float r_DefaultRotateValue = 0.3f;

        public KinectInputState Translate(HandState leftState, HandState rightState, Dictionary<JointType, Point> jointPositions, bool flying)
        {
            var state = new KinectInputState(flying);
            
            if (!((leftState == HandState.NotTracked) || (leftState == HandState.Unknown)
              || (rightState == HandState.NotTracked) || (rightState == HandState.Unknown)))
            {
                var leftRel = GetRelativePosition(jointPositions[JointType.HandLeft]);
                var rightRel = GetRelativePosition(jointPositions[JointType.HandRight]);
                var headRel = GetRelativePosition(jointPositions[JointType.Head]);
                var neckRel = GetRelativePosition(jointPositions[JointType.SpineShoulder]);
                var midRel = GetRelativePosition(jointPositions[JointType.SpineMid]);

                if (flying)
                {
                    // *** land *** (both hands showing lasso?)
                    if (((leftState == HandState.Lasso) && (rightState == HandState.Lasso))
                        && (leftRel.Y > midRel.Y) && (rightRel.Y > midRel.Y) 
                        && ApproximatelyEqual((float)leftRel.Y, (float)rightRel.Y))
                    {
                        state.Flag = "land";
                        state.Flying = false;
                        return state;
                    }

                    // *** ascend/descend value *** (both hands higher than head / lower than mid?)
                    if (ApproximatelyEqual((float)leftRel.Y, (float)rightRel.Y)) // hold vertical position
                    {
                        if ((leftRel.Y > midRel.Y) && (rightRel.Y > midRel.Y)) // descend
                            state.Ascend = -r_DefaultAscendValue;
                        else if ((leftRel.Y < headRel.Y) && (rightRel.Y < headRel.Y)) // ascend
                            state.Ascend = r_DefaultAscendValue;
                    }

                    // *** rotate value *** (left hand closed and right hand opened or vice versa?)
                    if ((leftState == HandState.Closed) && (rightState == HandState.Open)) // rotate left
                        state.Rotate = -r_DefaultRotateValue;
                    else if ((leftState == HandState.Open) && (rightState == HandState.Closed)) // rotate right
                        state.Rotate = r_DefaultRotateValue;

                    // *** speed value *** (both hands opened/lasso?)
                    if ((leftState == HandState.Open) && (rightState == HandState.Open))
                        state.SpeedUp = 0.3f; // must be !=null to be set to the global speed value
                    else if ((leftState == HandState.Lasso) || (rightState == HandState.Lasso))
                        state.SpeedUp = -0.3f;

                    // *** roll value *** (right hand higher and left hand lower than neck and vice versa?
                    //                     neck, left hand and right hand approximately equal height?)
                    if ((((leftRel.Y > neckRel.Y) && (rightRel.Y < neckRel.Y)) ||
                        ((leftRel.Y < neckRel.Y) && (rightRel.Y > neckRel.Y))) &&
                        !ApproximatelyEqual((float)leftRel.Y, (float)rightRel.Y, (float)neckRel.Y)) // dont roll?
                    {
                        if (leftRel.Y > rightRel.Y) // roll left
                            state.Roll = -EstimateControlFactor((float)rightRel.Y, (float)leftRel.Y);
                        else if (leftRel.Y < rightRel.Y) // roll right
                            state.Roll = EstimateControlFactor((float)leftRel.Y, (float)rightRel.Y);
                    }
                }
                else
                {
                    // *** take off *** (both hands closed and higher than head?)   0.0966 = 40/414
                    if ((leftState == HandState.Closed) && (rightState == HandState.Closed)
                        && ((leftRel.Y - headRel.Y) < -0.0966) && ((rightRel.Y - headRel.Y) < -0.0966))
                    {
                        state.Flag = "takeoff";
                        state.Flying = true;
                    }
                }
            }
            
            return state;
        }

        private Point GetRelativePosition(Point point)
        {
            return new Point(point.X / 512, point.Y / 414);
        }

        private bool ApproximatelyEqual(float p1, float p2, float p3 = -1)
        {
            if (p3 < 0) return (Math.Abs(p1 - p2) < 0.06f); // 0.06 = 25/414    ;   0.12 = 50/414
            else return (Math.Abs(p1 - p3) < 0.06f) && (Math.Abs(p2 - p3) < 0.06f) && (Math.Abs(p1 - p2) < 0.12f);
        }

        // the lower value has a higher position and vice versa
        private float EstimateControlFactor(float higherPos, float lowerPos)
        {
            return (1 - higherPos / lowerPos);
        }
    }
}
