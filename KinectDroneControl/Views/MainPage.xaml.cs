using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Threading.Tasks;
using System.Windows.Input;
using ARDrone2Client.Common;
using GalaSoft.MvvmLight.Command;
using KinectDroneControl.Extensions.Kinect;
using KinectDroneControl.KinectDroneInterface;
using Windows.Foundation;
using Windows.UI;
using Windows.UI.Popups;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Controls.Primitives;
using Windows.UI.Xaml.Media;
using Windows.UI.Xaml.Shapes;
using WindowsPreview.Kinect;

namespace KinectDroneControl.Views
{
    public sealed partial class MainPage : Page, INotifyPropertyChanged, IDisposable
    {
        #region Constants
        private const double c_HighConfidenceHandSize = 40;
        private const double c_LowConfidenceHandSize = 20;
        private const double c_JointThickness = 8.0;
        private const double c_TrackedBoneThickness = 4.0;
        private const double c_InferredBoneThickness = 1.0;
        private const double c_ClipBoundsThickness = 5;
        private const float c_InferredZPositionClamp = 0.1f;
        #endregion

        #region Private Members
        private float m_JointSpaceWidth;
        private float m_JointSpaceHeight;

        private KinectSensor m_KinectSensor;
        private BodyFrameReader m_BodyFrameReader;
        private Body[] m_Bodies;
        private Canvas m_Canvas;
        private BodyInfo[] m_BodyInfos;
        private List<Color> m_BodyColors = new List<Color>
            {
                Colors.Red,
                Colors.Orange,
                Colors.Green,
                Colors.Blue,
                Colors.Indigo,
                Colors.Violet
            };
        private Rectangle m_LeftClipEdge;
        private Rectangle m_RightClipEdge;
        private Rectangle m_TopClipEdge;
        private Rectangle m_BottomClipEdge;

        private int m_BodyCount
        {
            get { return m_BodyInfos == null ? 0 : m_BodyInfos.Length; }
            set
            {
                if (value == 0)
                {
                    m_BodyInfos = null;
                    return;
                }
                // creates instances of BodyInfo objects for potential number of bodies
                if (m_BodyInfos == null || m_BodyInfos.Length != value)
                {
                    m_BodyInfos = new BodyInfo[value];
                    for (int bodyIndex = 0; bodyIndex < m_Bodies.Length; bodyIndex++)
                        m_BodyInfos[bodyIndex] = new BodyInfo(m_BodyColors[bodyIndex]);
                }
            }
        }

        private IGestureTranslator m_Translator = new GestureTranslator();
        #endregion

        #region Public Fields
        private string m_StatusText;
        public string StatusText
        {
            get { return m_StatusText; }
            set
            {
                if (m_StatusText != value)
                {
                    m_StatusText = value;
                    RaisePropertyChanged(nameof(StatusText));
                }
            }
        }

        private bool m_ControlLocked;
        public bool ControlLocked
        {
            get { return m_ControlLocked; }
            set
            {
                if (value != m_ControlLocked)
                {
                    m_ControlLocked = value;
                    RaisePropertyChanged(nameof(ControlLocked));
                }
            }
        }

        private int m_BatteryState;
        public int BatteryState
        {
            get { return m_BatteryState; }
            set
            {
                if (value != m_BatteryState)
                {
                    m_BatteryState = value;
                    RaisePropertyChanged(nameof(BatteryState));
                }
            }
        }

        private bool m_IsConnected;
        public bool IsConnected
        {
            get { return m_IsConnected; }
            set
            {
                if (value != m_IsConnected)
                {
                    m_IsConnected = value;
                    RaisePropertyChanged(nameof(IsConnected));
                }
            }
        }

        private bool m_IsFlying;
        public bool IsFlying
        {
            get { return m_IsFlying; }
            set
            {
                if (value != m_IsFlying)
                {
                    m_IsFlying = value;
                    RaisePropertyChanged(nameof(IsFlying));
                }
            }
        }
        #endregion

        #region Commands
        private ICommand m_ConnectDisconnectCommand;
        public ICommand ConnectDisconnectCommand
        {
            get
            {
                return m_ConnectDisconnectCommand ?? (m_ConnectDisconnectCommand = new RelayCommand(ConnectDisconnect));
            }
        }

        private ICommand m_TakeoffLandCommand;
        public ICommand TakeoffLandCommand
        {
            get
            {
                return m_TakeoffLandCommand ?? (m_TakeoffLandCommand = new RelayCommand(TakeoffLand));
            }
        }

        private ICommand m_ControlLockCommand;
        public ICommand ControlLockCommand
        {
            get
            {
                return m_ControlLockCommand ?? (m_ControlLockCommand = new RelayCommand(LockControl));
            }
        }
        #endregion

        #region PropertyChanged
        public event PropertyChangedEventHandler PropertyChanged;
        public void RaisePropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }
        #endregion

        #region Contruction & Diposing
        public MainPage()
        {
            //InitializeDrone();
            InitializeKinect();

            DataContext = this;
            InitializeComponent();

            m_Canvas.Clip = new RectangleGeometry();// set the clip rectangle to prevent rendering outside the canvas
            m_Canvas.Clip.Rect = new Rect(0.0, 0.0, DisplayGrid.Width, DisplayGrid.Height);
            PopulateVisualObjects();
            DisplayGrid.Children.Add(m_Canvas);
            ResetFactors();
            UpdateUIElements();
            //VideoGrid.Children.Add(DroneVideoElem);
            //DroneVideoElem.Visibility = Visibility.Collapsed;
        }

        private void PopulateVisualObjects()
        {
            // create clipped edges and set to collapsed initially
            m_LeftClipEdge = new Rectangle()
            {
                Fill = new SolidColorBrush(Colors.Red),
                Width = c_ClipBoundsThickness,
                Height = DisplayGrid.Height,
                Visibility = Visibility.Collapsed
            };
            m_RightClipEdge = new Rectangle()
            {
                Fill = new SolidColorBrush(Colors.Red),
                Width = c_ClipBoundsThickness,
                Height = DisplayGrid.Height,
                Visibility = Visibility.Collapsed
            };
            m_TopClipEdge = new Rectangle()
            {
                Fill = new SolidColorBrush(Colors.Red),
                Width = DisplayGrid.Width,
                Height = c_ClipBoundsThickness,
                Visibility = Visibility.Collapsed
            };
            m_BottomClipEdge = new Rectangle()
            {
                Fill = new SolidColorBrush(Colors.Red),
                Width = DisplayGrid.Width,
                Height = c_ClipBoundsThickness,
                Visibility = Visibility.Collapsed
            };
            foreach (var bodyInfo in m_BodyInfos)
            {
                // add left and right hand ellipses of all bodies to canvas
                m_Canvas.Children.Add(bodyInfo.HandLeftEllipse);
                m_Canvas.Children.Add(bodyInfo.HandRightEllipse);
                foreach (var joint in bodyInfo.JointPoints)// add joint ellipses of all bodies to canvas
                {
                    m_Canvas.Children.Add(joint.Value);
                }
                foreach (var bone in bodyInfo.Bones)// add bone lines of all bodies to canvas
                {
                    m_Canvas.Children.Add(bodyInfo.BoneLines[bone]);
                }
            }
            // add clipped edges rectangles to main canvas
            m_Canvas.Children.Add(m_LeftClipEdge);
            m_Canvas.Children.Add(m_RightClipEdge);
            m_Canvas.Children.Add(m_TopClipEdge);
            m_Canvas.Children.Add(m_BottomClipEdge);
            // position the clipped edges
            Canvas.SetLeft(m_LeftClipEdge, 0);
            Canvas.SetTop(m_LeftClipEdge, 0);
            Canvas.SetLeft(m_RightClipEdge, DisplayGrid.Width - c_ClipBoundsThickness);
            Canvas.SetTop(m_RightClipEdge, 0);
            Canvas.SetLeft(m_TopClipEdge, 0);
            Canvas.SetTop(m_TopClipEdge, 0);
            Canvas.SetLeft(m_BottomClipEdge, 0);
            Canvas.SetTop(m_BottomClipEdge, DisplayGrid.Height - c_ClipBoundsThickness);
        }

        private void MainPage_Unloaded(object sender, RoutedEventArgs e)
        {
            Dispose();
        }

        public void Dispose()
        {
            if (m_BodyFrameReader != null)
            {
                m_BodyFrameReader.Dispose();
                m_BodyFrameReader = null;
            }
            if (m_Bodies != null)
            {
                foreach (var body in m_Bodies)
                {
                    if (body != null)
                        body.Dispose();
                }
            }
            if (m_KinectSensor != null)
            {
                m_KinectSensor.Close();
                m_KinectSensor = null;
            }
        }
        #endregion

        #region Kinect
        private void InitializeKinect()
        {
            m_KinectSensor = KinectSensor.GetDefault();
            var frameDescription = m_KinectSensor.DepthFrameSource.FrameDescription;// get the depth (display) extents
            // get size of joint space
            m_JointSpaceWidth = frameDescription.Width;
            m_JointSpaceHeight = frameDescription.Height;
            m_Bodies = new Body[m_KinectSensor.BodyFrameSource.BodyCount];// get total number of bodies from BodyFrameSource
            m_BodyFrameReader = m_KinectSensor.BodyFrameSource.OpenReader();
            m_BodyFrameReader.FrameArrived += Reader_BodyFrameArrived;
            m_KinectSensor.IsAvailableChanged += KinectSensor_IsAvailableChanged;
            // sets total number of possible tracked bodies
            // create ellipses and lines for drawing bodies
            m_Canvas = new Canvas();
            m_KinectSensor.Open();
            m_BodyCount = m_KinectSensor.BodyFrameSource.BodyCount;

            StatusText = m_KinectSensor.IsAvailable ? "Kinect Sensor is running!" : "No ready Kinect Sensor found!";
        }

        private async void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            bool hasTrackedBody = false;
            using (var bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    bodyFrame.GetAndRefreshBodyData(m_Bodies);
                    dataReceived = true;
                }
            }
            if (dataReceived)
            {
                bool alreadyControlled = false;
                for (int bodyIndex = 0; bodyIndex < m_Bodies.Length; bodyIndex++)
                {
                    var body = m_Bodies[bodyIndex];
                    if (body.IsTracked)
                    {
                        UpdateClippedEdges(body, hasTrackedBody);
                        var jointPointsInDepthSpace = UpdateBody(body, m_BodyInfos[bodyIndex]);

                        hasTrackedBody = true;

                        if (/*IsConnected && !ControlLocked && */!alreadyControlled) // convert the gestures of the body and send them as a command to the drone
                        {
                            //await SendGesturesToDrone(m_Translator.Translate(body.HandLeftState, body.HandRightState, jointPointsInDepthSpace, IsFlying));
                            alreadyControlled = true; // Allow only the first tracked body to control
                        }
                    }
                    else
                        m_BodyInfos[bodyIndex].Clear();
                }
                if (!hasTrackedBody)
                    ClearClippedEdges();
            }
        }

        private Dictionary<JointType, Point> UpdateBody(Body body, BodyInfo bodyInfo)
        {
            var joints = body.Joints;
            var jointPointsInDepthSpace = new Dictionary<JointType, Point>();
            var coordinateMapper = m_KinectSensor.CoordinateMapper;
            foreach (var jointType in body.Joints.Keys)
            {
                // sometimes the depth(Z) of an inferred joint may show as negative
                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                var position = body.Joints[jointType].Position;
                if (position.Z < 0)
                    position.Z = c_InferredZPositionClamp;
                // map joint position to depth space
                var depthSpacePoint = coordinateMapper.MapCameraPointToDepthSpace(position);
                jointPointsInDepthSpace[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                // modify the joint's visibility and location
                UpdateJoint(bodyInfo.JointPoints[jointType], joints[jointType], jointPointsInDepthSpace[jointType]);
                // modify hand ellipse colors based on hand states
                // modity hand ellipse sizes based on tracking confidences
                if (jointType == JointType.HandRight)
                    UpdateHand(bodyInfo.HandRightEllipse, body.HandRightState, body.HandRightConfidence, jointPointsInDepthSpace[jointType]);
                if (jointType == JointType.HandLeft)
                    UpdateHand(bodyInfo.HandLeftEllipse, body.HandLeftState, body.HandLeftConfidence, jointPointsInDepthSpace[jointType]);
            }
            foreach (var bone in bodyInfo.Bones)
            {
                UpdateBone(bodyInfo.BoneLines[bone], joints[bone.Item1], joints[bone.Item2],
                                jointPointsInDepthSpace[bone.Item1],
                                jointPointsInDepthSpace[bone.Item2]);
            }

            return jointPointsInDepthSpace;
            //if (IsConnected && !ControlLocked) // convert the gestures of the body and send them as a command to the drone
            //    await SendGesturesToDrone(m_Translator.Translate(body.HandLeftState, body.HandRightState, jointPointsInDepthSpace, IsFlying));
        }

        private void UpdateJoint(Ellipse ellipse, Joint joint, Point point)
        {
            var trackingState = joint.TrackingState;
            if (trackingState != TrackingState.NotTracked)// only draw if joint is tracked or inferred
            {
                if (trackingState == TrackingState.Tracked)
                    ellipse.Fill = new SolidColorBrush(Colors.Green);
                else // inferred joints are yellow
                    ellipse.Fill = new SolidColorBrush(Colors.Yellow);
                Canvas.SetLeft(ellipse, point.X - c_JointThickness / 2);
                Canvas.SetTop(ellipse, point.Y - c_JointThickness / 2);
                ellipse.Visibility = Visibility.Visible;
            }
            else
                ellipse.Visibility = Visibility.Collapsed;
        }

        private void UpdateHand(Ellipse ellipse, HandState handState, TrackingConfidence trackingConfidence, Point point)
        {
            ellipse.Fill = new SolidColorBrush(HandStateUtilities.HandStateToColor(handState));
            // draw handstate ellipse based on tracking confidence
            ellipse.Width = ellipse.Height = (trackingConfidence == TrackingConfidence.Low) ? c_LowConfidenceHandSize : c_HighConfidenceHandSize;
            ellipse.Visibility = Visibility.Visible;
            if (!double.IsInfinity(point.X) && !double.IsInfinity(point.Y))// don't draw handstate if hand joints are not tracked
            {
                Canvas.SetLeft(ellipse, point.X - ellipse.Width / 2);
                Canvas.SetTop(ellipse, point.Y - ellipse.Width / 2);
            }
        }

        private void UpdateBone(Line line, Joint startJoint, Joint endJoint, Point startPoint, Point endPoint)
        {
            // don't draw if neither joints are tracked
            if (startJoint.TrackingState == TrackingState.NotTracked || endJoint.TrackingState == TrackingState.NotTracked)
            {
                line.Visibility = Visibility.Collapsed;
                return;
            }
            line.StrokeThickness = c_InferredBoneThickness;// all lines are inferred thickness unless both joints are tracked
            if (startJoint.TrackingState == TrackingState.Tracked &&
                endJoint.TrackingState == TrackingState.Tracked)
                line.StrokeThickness = c_TrackedBoneThickness;
            line.Visibility = Visibility.Visible;
            line.X1 = startPoint.X;
            line.Y1 = startPoint.Y;
            line.X2 = endPoint.X;
            line.Y2 = endPoint.Y;
        }

        private void UpdateClippedEdges(Body body, bool hasTrackedBody)
        {
            var clippedEdges = body.ClippedEdges;
            if (clippedEdges.HasFlag(FrameEdges.Left))
                m_LeftClipEdge.Visibility = Visibility.Visible;
            else if (!hasTrackedBody)// don't clear this edge if another body is triggering clipped edge
                m_LeftClipEdge.Visibility = Visibility.Collapsed;

            if (clippedEdges.HasFlag(FrameEdges.Right))
                m_RightClipEdge.Visibility = Visibility.Visible;
            else if (!hasTrackedBody)
                m_RightClipEdge.Visibility = Visibility.Collapsed;

            if (clippedEdges.HasFlag(FrameEdges.Top))
                m_TopClipEdge.Visibility = Visibility.Visible;
            else if (!hasTrackedBody)
                m_TopClipEdge.Visibility = Visibility.Collapsed;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
                m_BottomClipEdge.Visibility = Visibility.Visible;
            else if (!hasTrackedBody)
                m_BottomClipEdge.Visibility = Visibility.Collapsed;
        }

        private void ClearClippedEdges()
        {
            m_LeftClipEdge.Visibility = Visibility.Collapsed;
            m_RightClipEdge.Visibility = Visibility.Collapsed;
            m_TopClipEdge.Visibility = Visibility.Collapsed;
            m_BottomClipEdge.Visibility = Visibility.Collapsed;
        }
        
        private void KinectSensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            StatusText = m_KinectSensor.IsAvailable ? "Kinect Sensor is running!" : "Kinect Sensor not available!";
        }
        #endregion

        #region Drone
        private DroneClient m_Drone;
        private double m_GlobalSpeed;
        private double m_CurveFactor;
        private const double c_DefaultSpeed = 0.1f;
        private const double m_DefaultCurveFactor = 0.3f;
        private DispatcherTimer m_BatteryTimer;
        //private readonly string videoUrl = "ardrone://192.168.1.1";
        //MediaElement DroneVideoElem;
        private void InitializeDrone()
        {
            m_Drone = DroneClient.Instance;
            m_GlobalSpeed = c_DefaultSpeed;
            m_CurveFactor = m_DefaultCurveFactor;
            m_BatteryTimer = new DispatcherTimer();
            m_BatteryTimer.Tick += BatteryTimer_Tick;
            m_BatteryTimer.Interval = new TimeSpan(0, 0, 5); // 10 seconds
            //DroneVideoElem = new MediaElement();
            //DroneVideoElem.MediaFailed += DroneVideoElem_MediaFailed;
        }

        private void GlobalSpeedSliderValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            m_GlobalSpeed = e.NewValue;
        }

        private void CurveFactorSliderValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            m_CurveFactor = e.NewValue;
        }

        private void BatteryTimer_Tick(object sender, object e)
        {
            if (IsConnected)
            {
                BatteryState = int.Parse(m_Drone.NavigationDataViewModel.BatteryPercentage);
            }
        }

        private async Task SendGesturesToDrone(KinectInputState gestureCommand)
        {
            if (gestureCommand.Flag == "land")
            {
                m_Drone.InputState.Update(0, 0, 0, 0);
                m_Drone.Land();
                ControlLocked = true;
            }
            else if (gestureCommand.Flag == "takeoff")
            {
                if (m_Drone.NavigationDataViewModel.BatteryIsLow.Equals("False"))
                {
                    m_Drone.TakeOff();
                    m_Drone.InputState.Update(0, 0, 0, 0);
                }
                else
                    await (new MessageDialog("Battery too low.")).ShowAsync();
            }
            else
            {
                double speed = 0;
                if (gestureCommand.SpeedUp > 0)
                    speed = -m_GlobalSpeed;
                else if (gestureCommand.SpeedUp < 0)
                    speed = m_GlobalSpeed;
                m_Drone.InputState.Update((float)(gestureCommand.Roll * m_CurveFactor), (float)speed, gestureCommand.Rotate, gestureCommand.Ascend);
            }
            IsFlying = gestureCommand.Flying;
            UpdateUIElements();
        }

        private async void ConnectDisconnect()
        {
            if (!IsConnected)
            {
                if (await m_Drone.ConnectAsync())
                {
                    m_BatteryTimer.Start();
                    IsConnected = true;
                    ControlLocked = true;
                    //ShowDroneCamera();
                }
                else
                    await (new MessageDialog("Connection failed...got WLAN-Connection to Drone?")).ShowAsync();
            }
            else
            {
                m_Drone.Dispose();
                m_Drone.Close();
                IsConnected = false;
            }
            ResetFactors();
            UpdateUIElements();
        }

        private async void TakeoffLand()
        {
            if (!IsFlying)
            {
                if (m_Drone.NavigationDataViewModel.BatteryIsLow.Equals("False"))
                {
                    m_Drone.TakeOff();
                    m_Drone.InputState.Update(0, 0, 0, 0);
                    IsFlying = true;
                }
                else
                    await (new MessageDialog("Battery too low.")).ShowAsync();
            }
            else
            {
                m_Drone.InputState.Update(0, 0, 0, 0);
                ControlLocked = true;
                m_Drone.Land();
                IsFlying = false;
            }
            ResetFactors();
            UpdateUIElements();
        }

        private void LockControl()
        {
            ControlLocked = !ControlLocked;
            m_Drone.InputState.Update(0, 0, 0, 0);
            ResetFactors();
            UpdateUIElements();
        }

        //private void ChangeViewButton_Click(object sender, RoutedEventArgs e)
        //{
        //    if (DroneVideoElem.Visibility == Visibility.Collapsed)
        //    {
        //        DroneVideoElem.Visibility = Visibility.Visible;
        //        ShowDroneCamera();
        //    }
        //    else
        //    {
        //        DroneVideoElem.Source = null;
        //        DroneVideoElem.Visibility = Visibility.Collapsed;
        //    }

        //    if (KinectViewBox.Visibility == Visibility.Collapsed)
        //        KinectViewBox.Visibility = Visibility.Visible;
        //    else KinectViewBox.Visibility = Visibility.Collapsed;
        //}

        //private void ShowDroneCamera()
        //{
        //    //IRandomAccessStream stream;
        //    //DroneVideoElem.SetSource(stream, "video/mp4");
        //    DroneVideoElem.Source = new Uri(@videoUrl);
        //    DroneVideoElem.Play(); 
        //}

        //private async void DroneVideoElem_MediaFailed(object sender, ExceptionRoutedEventArgs e)
        //{
        //    await (new MessageDialog(e.ErrorMessage)).ShowAsync();
        //}

        private void ResetFactors()
        {
            m_GlobalSpeed = c_DefaultSpeed;
            m_CurveFactor = m_DefaultCurveFactor;
            ResetControlSliders();
        }

        private void ResetControlSliders()
        {
            m_SpeedupSlider.Value = m_GlobalSpeed;
            m_CurveSlider.Value = m_CurveFactor;
        }

        private readonly Brush r_MasterYellow = new SolidColorBrush(Colors.Yellow);
        private readonly Brush r_TransparentBrush = new SolidColorBrush(Colors.Transparent);
        private void UpdateUIElements()
        {
            if (!IsConnected)
            {
                //KinectViewBox.Visibility = Visibility.Visible;
                //DroneVideoElem.Visibility = Visibility.Collapsed;
                //ChangeViewButton.IsEnabled = false;
                IsFlying = false;
                ControlLocked = false;
            }
        }
        #endregion
    }
}
