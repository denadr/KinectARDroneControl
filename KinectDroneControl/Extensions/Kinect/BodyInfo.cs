﻿using System;
using System.Collections.Generic;
using Windows.UI;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Shapes;
using WindowsPreview.Kinect;

namespace KinectDroneControl.Extensions.Kinect
{
    class BodyInfo
    {
        private const double JointThickness = 8.0;

            public bool Updated { get; set; }
            public Color BodyColor { get; set; }
            public Ellipse HandLeftEllipse { get; set; } // ellipse representing left handstate
            public Ellipse HandRightEllipse { get; set; } // ellipse representing right handstate
            public Dictionary<JointType, Ellipse> JointPoints { get; private set; } // dictionary of all joints in a body
            public TupleList<JointType, JointType> Bones { get; private set; } // definition of bones
            public Dictionary<Tuple<JointType, JointType>, Line> BoneLines { get; private set; } // collection of bones associated with the line object

            public BodyInfo(Color bodyColor)
            {
                BodyColor = bodyColor;

                // create hand state ellipses
                HandLeftEllipse = new Ellipse()
                {
                    Visibility = Visibility.Collapsed
                };

                HandRightEllipse = new Ellipse()
                {
                    Visibility = Visibility.Collapsed
                };

                // a joint defined as a jointType with a point location in XY space represented by an ellipse
                JointPoints = new Dictionary<JointType, Ellipse>();

                // pre-populate list of joints and set to non-visible initially
                foreach (JointType jointType in Enum.GetValues(typeof(JointType)))
                {
                    JointPoints.Add(jointType, new Ellipse()
                    {
                        Visibility = Visibility.Collapsed,
                        Fill = new SolidColorBrush(BodyColor),
                        Width = JointThickness,
                        Height = JointThickness
                    });
                }

                // collection of bones
                BoneLines = new Dictionary<Tuple<JointType, JointType>, Line>();

                // a bone defined as a line between two joints
                Bones = new TupleList<JointType, JointType>
                {
                    // Torso
                    { JointType.Head, JointType.Neck },
                    { JointType.Neck, JointType.SpineShoulder },
                    { JointType.SpineShoulder, JointType.SpineMid },
                    { JointType.SpineMid, JointType.SpineBase },
                    { JointType.SpineShoulder, JointType.ShoulderRight },
                    { JointType.SpineShoulder, JointType.ShoulderLeft },
                    { JointType.SpineBase, JointType.HipRight },
                    { JointType.SpineBase, JointType.HipLeft },

                    // Right Arm
                    { JointType.ShoulderRight, JointType.ElbowRight },
                    { JointType.ElbowRight, JointType.WristRight },
                    { JointType.WristRight, JointType.HandRight },
                    { JointType.HandRight, JointType.HandTipRight },
                    { JointType.WristRight, JointType.ThumbRight },

                    // Left Arm
                    { JointType.ShoulderLeft, JointType.ElbowLeft },
                    { JointType.ElbowLeft, JointType.WristLeft },
                    { JointType.WristLeft, JointType.HandLeft },
                    { JointType.HandLeft, JointType.HandTipLeft },
                    { JointType.WristLeft, JointType.ThumbLeft },

                    // Right Leg
                    { JointType.HipRight, JointType.KneeRight },
                    { JointType.KneeRight, JointType.AnkleRight },
                    { JointType.AnkleRight, JointType.FootRight },
                
                    // Left Leg
                    { JointType.HipLeft, JointType.KneeLeft },
                    { JointType.KneeLeft, JointType.AnkleLeft },
                    { JointType.AnkleLeft, JointType.FootLeft },
                };

                // pre-populate list of bones that are non-visible initially
                foreach (var bone in Bones)
                {
                    BoneLines.Add(bone, new Line()
                    {
                        Stroke = new SolidColorBrush(BodyColor),
                        Visibility = Visibility.Collapsed
                    });
                }
            }

        public void Clear()
        {
            foreach (var joint in JointPoints)// collapse all joint ellipses
            {
                joint.Value.Visibility = Visibility.Collapsed;
            }
            foreach (var bone in Bones)// collapse all bone lines
            {
                BoneLines[bone].Visibility = Visibility.Collapsed;
            }
            // collapse handstate ellipses
            HandLeftEllipse.Visibility = Visibility.Collapsed;
            HandRightEllipse.Visibility = Visibility.Collapsed;
        }
       
    }
}
