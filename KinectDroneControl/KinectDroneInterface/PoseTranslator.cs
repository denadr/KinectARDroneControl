using System;
using System.Collections.Generic;
using WindowsPreview.Kinect;

namespace KinectDroneControl.KinectDroneInterface
{
    class PoseTranslator : IGestureTranslator
    {
        public KinectInputState Translate(Dictionary<JointType, CameraSpacePoint> jointPositions, bool flying, HandState leftState, HandState rightState, CoordinateMapper mapper)
        {
            var state = new KinectInputState(flying);

            if (!(leftState == HandState.NotTracked || rightState == HandState.NotTracked))
            {
                var leftHand = jointPositions[JointType.HandLeft];
                var rightHand = jointPositions[JointType.HandRight];
                var head = jointPositions[JointType.Head];
                var neck = jointPositions[JointType.SpineShoulder];
                var middle = jointPositions[JointType.SpineMid];

                var leftHandDepth = mapper.MapCameraPointToDepthSpace(jointPositions[JointType.HandLeft]);
                var rightHandDepth = mapper.MapCameraPointToDepthSpace(jointPositions[JointType.HandRight]);
                var headDepth = mapper.MapCameraPointToDepthSpace(jointPositions[JointType.Head]);
                var neckDepth = mapper.MapCameraPointToDepthSpace(jointPositions[JointType.SpineShoulder]);
                var middleDepth = mapper.MapCameraPointToDepthSpace(jointPositions[JointType.SpineMid]);

                if (flying)
                {
                    // Landing
                    if (leftState == HandState.Lasso && rightState == HandState.Lasso
                        && (leftHandDepth.Y > middleDepth.Y) && (rightHandDepth.Y > middleDepth.Y)
                        && ApproximatelyEqualDepth(leftHandDepth.Y, rightHandDepth.Y))
                    {
                        state.Mode = InputStateMode.Land;
                        state.Flying = false;
                        return state;
                    }

                    // Ascend/Descend
                    if (ApproximatelyEqualDepth(leftHandDepth.Y, rightHandDepth.Y))
                    {
                        if ((leftHandDepth.Y > neckDepth.Y) && (rightHandDepth.Y > neckDepth.Y)) // descend
                        {
                            state.Ascend = -EstimateAscendFactor(leftHand.Y, rightHand.Y, neck.Y);
                        }
                        else if ((leftHandDepth.Y < neckDepth.Y) && (rightHandDepth.Y < neckDepth.Y)) // ascend
                        {
                            state.Ascend = EstimateAscendFactor(leftHand.Y, rightHand.Y, neck.Y);
                        }
                    }

                    if (!ApproximatelyEqualCamera(leftHand.Z, neck.Z) &&
                        !ApproximatelyEqualCamera(rightHand.Z, neck.Z))
                    {
                        // Rotation
                        if (leftHand.Z > neck.Z && rightHand.Z < neck.Z)
                        {
                            state.Rotate = -EstimateRotateFactor(leftHand.Z, rightHand.Z);
                        }
                        else if (leftHand.Z < neck.Z && rightHand.Z > neck.Z)
                        {
                            state.Rotate = EstimateRotateFactor(leftHand.Z, rightHand.Z);
                        }
                        
                        // Speed
                        else if (leftHand.Z > neck.Z && rightHand.Z > neck.Z)
                        {
                            state.SpeedUp = EstimateSpeedFactor(leftHand.Z, rightHand.Z, neck.Z);
                        }
                        else if (leftHand.Z < neck.Z && rightHand.Z < neck.Z)
                        {
                            state.SpeedUp = -EstimateSpeedFactor(leftHand.Z, rightHand.Z, neck.Z);
                        }
                    }

                    // Roll
                    if (!ApproximatelyEqualCamera(leftHand.Y, rightHand.Y, neck.Y) && // dont roll?
                        (((leftHandDepth.Y > neckDepth.Y) && (rightHandDepth.Y < neckDepth.Y)) ||
                        ((leftHandDepth.Y < neckDepth.Y) && (rightHandDepth.Y > neckDepth.Y))))
                    {
                        if (leftHandDepth.Y > rightHandDepth.Y) // roll left
                        {
                            state.Roll = -EstimateRollFactor(rightHandDepth.Y, leftHandDepth.Y);
                        }
                        else if (leftHandDepth.Y < rightHandDepth.Y) // roll right
                        {
                            state.Roll = EstimateRollFactor(leftHandDepth.Y, rightHandDepth.Y);
                        }
                    }
                }
                else
                {
                    // Takeoff
                    if (leftState == HandState.Lasso && rightState == HandState.Lasso
                        && ((leftHandDepth.Y < headDepth.Y) && ((rightHandDepth.Y < headDepth.Y))
                        && ApproximatelyEqualDepth(leftHandDepth.Y, rightHandDepth.Y)))
                    {
                        state.Mode = InputStateMode.Takeoff;
                        state.Flying = true;
                        return state;
                    }
                }
            }

            return state;
        }

        private float EstimateSpeedFactor(float leftZ, float rightZ, float neckZ)
        {
            //return Constants.Speed;
            var handsZ = (leftZ + rightZ) / 2;
            var diff = Math.Abs(handsZ - neckZ);
            return diff > 1 ? 1 : diff;
        }

        private float EstimateRotateFactor(float leftZ, float rightZ)
        {
            //return Constants.Rotate;
            var diff = Math.Abs(leftZ - rightZ);
            var temp = diff / 2;
            return temp > 1 ? 1 : temp;
        }

        private float EstimateAscendFactor(float leftY, float rightY, float neckY)
        {
            //return Constants.Ascend;
            var handsY = (leftY + rightY) / 2;
            var diff = Math.Abs(handsY - neckY);
            return diff > 1 ? 1 : diff;
        }

        private bool ApproximatelyEqualDepth(float p1, float p2, float p3 = -1)
        {
            if (p3 < 0)
                return (Math.Abs(p1 - p2) < 20f);
            else
                return (Math.Abs(p1 - p3) < 0.06f) && (Math.Abs(p2 - p3) < 0.06f) && (Math.Abs(p1 - p2) < 0.12f);
        }

        private bool ApproximatelyEqualCamera(float p1, float p2, float p3 = -1)
        {
            if (p3 < 0)
                return (Math.Abs(p1 - p2) < 0.2f);
            else
                return (Math.Abs(p1 - p3) < 0.2f) && (Math.Abs(p2 - p3) < 0.2f) && (Math.Abs(p1 - p2) < 0.3f);
        }

        // the lower value has a higher position and vice versa
        private float EstimateRollFactor(float higherPos, float lowerPos)
        {
            //return Constants.Roll;
            return (1 - higherPos / lowerPos);
        }
    }
}
