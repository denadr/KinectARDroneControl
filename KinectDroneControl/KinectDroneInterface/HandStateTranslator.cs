using System;
using System.Collections.Generic;
using KinectDroneControl.Extensions.ArDrone;
using WindowsPreview.Kinect;

namespace KinectDroneControl.KinectDroneInterface
{
    class HandStateTranslator : IGestureTranslator
    {
        public KinectInputState Translate(Dictionary<JointType, CameraSpacePoint> jointPositions, bool flying, HandState leftState, HandState rightState, CoordinateMapper mapper)
        {
            var state = new KinectInputState(flying);
            
            if (!((leftState == HandState.NotTracked) || (leftState == HandState.Unknown)
              || (rightState == HandState.NotTracked) || (rightState == HandState.Unknown)))
            {
                var leftHandY = mapper.MapCameraPointToDepthSpace(jointPositions[JointType.HandLeft]).Y;
                var rightHandY = mapper.MapCameraPointToDepthSpace(jointPositions[JointType.HandRight]).Y;
                var headY = mapper.MapCameraPointToDepthSpace(jointPositions[JointType.Head]).Y;
                var neckY = mapper.MapCameraPointToDepthSpace(jointPositions[JointType.SpineShoulder]).Y;
                var middleY = mapper.MapCameraPointToDepthSpace(jointPositions[JointType.SpineMid]).Y;

                if (flying)
                {
                    // *** land *** (both hands showing lasso?)
                    if (((leftState == HandState.Lasso) && (rightState == HandState.Lasso))
                        && (leftHandY > middleY) && (rightHandY > middleY) 
                        && ApproximatelyEqual(leftHandY, rightHandY))
                    {
                        state.Mode = InputStateMode.Land;
                        state.Flying = false;
                        return state;
                    }

                    // *** ascend/descend value *** (both hands higher than head / lower than mid?)
                    if (ApproximatelyEqual(leftHandY, rightHandY)) // hold vertical position
                    {
                        if ((leftHandY > middleY) && (rightHandY > middleY)) // descend
                            state.Ascend = -Constants.Ascend;
                        else if ((leftHandY < headY) && (rightHandY < headY)) // ascend
                            state.Ascend = Constants.Ascend;
                    }

                    // *** rotate value *** (left hand closed and right hand opened or vice versa?)
                    if ((leftState == HandState.Closed) && (rightState == HandState.Open)) // rotate left
                        state.Rotate = -Constants.Rotate;
                    else if ((leftState == HandState.Open) && (rightState == HandState.Closed)) // rotate right
                        state.Rotate = Constants.Rotate;

                    // *** speed value *** (both hands opened/lasso?)
                    if ((leftState == HandState.Open) && (rightState == HandState.Open))
                        state.SpeedUp = -Constants.Speed; // must be !=null to be set to the global speed value
                    else if ((leftState == HandState.Lasso) || (rightState == HandState.Lasso))
                        state.SpeedUp = Constants.Speed;

                    // *** roll value *** (right hand higher and left hand lower than neck and vice versa?
                    //                     neck, left hand and right hand approximately equal height?)
                    if ((((leftHandY > neckY) && (rightHandY < neckY)) ||
                        ((leftHandY < neckY) && (rightHandY > neckY))) &&
                        !ApproximatelyEqual(leftHandY, rightHandY, neckY)) // dont roll?
                    {
                        if (leftHandY > rightHandY) // roll left
                            state.Roll = -EstimateRollFactor(rightHandY, leftHandY);
                        else if (leftHandY < rightHandY) // roll right
                            state.Roll = EstimateRollFactor(leftHandY, rightHandY);
                    }
                }
                else
                {
                    // *** take off *** (both hands closed and higher than head?)
                    if ((leftState == HandState.Closed) && (rightState == HandState.Closed)
                        && ((leftHandY - headY) < -0.0966) && ((rightHandY - headY) < -0.0966))
                    {
                        state.Mode = InputStateMode.Takeoff;
                        state.Flying = true;
                    }
                }
            }
            
            return state;
        }
        
        private bool ApproximatelyEqual(float p1, float p2, float p3 = -1)
        {
            if (p3 < 0) return (Math.Abs(p1 - p2) < 20f);
            else return (Math.Abs(p1 - p3) < 30f) && (Math.Abs(p2 - p3) < 30f) && (Math.Abs(p1 - p2) < 60f);
        }

        // the lower value has a higher position and vice versa
        private float EstimateRollFactor(float higherPos, float lowerPos)
        {
            return Constants.Roll;
            //return (1 - higherPos / lowerPos);
        }
    }
}
