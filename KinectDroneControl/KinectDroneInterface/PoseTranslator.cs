using System;
using System.Collections.Generic;
using KinectDroneControl.Extensions.ArDrone;
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

                    // Gaz
                    if (ApproximatelyEqualDepth(leftHandDepth.Y, rightHandDepth.Y))
                    {
                        if ((leftHandDepth.Y > neckDepth.Y) && (rightHandDepth.Y > neckDepth.Y)) // descend
                        {
                            state.Gaz = -EstimateGaz(leftHand.Y, rightHand.Y, neck.Y) * Constants.Gaz;
                        }
                        else if ((leftHandDepth.Y < neckDepth.Y) && (rightHandDepth.Y < neckDepth.Y)) // ascend
                        {
                            state.Gaz = EstimateGaz(leftHand.Y, rightHand.Y, neck.Y) * Constants.Gaz;
                        }
                    }

                    if (!ApproximatelyEqualCamera(leftHand.Z, neck.Z) &&
                        !ApproximatelyEqualCamera(rightHand.Z, neck.Z))
                    {
                        // Yaw
                        if (leftHand.Z > neck.Z && rightHand.Z < neck.Z)
                        {
                            state.Yaw = -EstimateYaw(leftHand.Z, rightHand.Z) * Constants.Yaw;
                        }
                        else if (leftHand.Z < neck.Z && rightHand.Z > neck.Z)
                        {
                            state.Yaw = EstimateYaw(leftHand.Z, rightHand.Z) * Constants.Yaw;
                        }
                        
                        // Pitch
                        else if (leftHand.Z > neck.Z && rightHand.Z > neck.Z)
                        {
                            state.Pitch = EstimatePitch(leftHand.Z, rightHand.Z, neck.Z) * Constants.Pitch;
                        }
                        else if (leftHand.Z < neck.Z && rightHand.Z < neck.Z)
                        {
                            state.Pitch = -EstimatePitch(leftHand.Z, rightHand.Z, neck.Z) * Constants.Pitch;
                        }
                    }

                    // Roll
                    if (!ApproximatelyEqualCamera(leftHand.Y, rightHand.Y, neck.Y) && // dont roll?
                        (((leftHandDepth.Y > neckDepth.Y) && (rightHandDepth.Y < neckDepth.Y)) ||
                        ((leftHandDepth.Y < neckDepth.Y) && (rightHandDepth.Y > neckDepth.Y))))
                    {
                        if (leftHandDepth.Y > rightHandDepth.Y) // roll left
                        {
                            state.Roll = -EstimateRoll(rightHandDepth.Y, leftHandDepth.Y) * Constants.Roll;
                        }
                        else if (leftHandDepth.Y < rightHandDepth.Y) // roll right
                        {
                            state.Roll = EstimateRoll(leftHandDepth.Y, rightHandDepth.Y) * Constants.Roll;
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

        private float EstimatePitch(float leftZ, float rightZ, float neckZ)
        {
            var handsZ = (leftZ + rightZ) / 2;
            var diff = Math.Abs(handsZ - neckZ);
            return diff > 1 ? 1 : diff;
        }

        private float EstimateYaw(float leftZ, float rightZ)
        {
            var diff = Math.Abs(leftZ - rightZ);
            var temp = diff / 2;
            return temp > 1 ? 1 : temp;
        }

        private float EstimateGaz(float leftY, float rightY, float neckY)
        {
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
        private float EstimateRoll(float higherPos, float lowerPos)
        {
            return (1 - higherPos / lowerPos);
        }
    }
}
