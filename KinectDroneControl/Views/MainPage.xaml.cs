using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Threading.Tasks;
using System.Windows.Input;
using ARDrone2Client.Common;
using GalaSoft.MvvmLight.Command;
using KinectDroneControl.Extensions.ArDrone;
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
        private CoordinateMapper m_CoordinateMapper;
        private BodyFrameReader m_BodyFrameReader;
        private Body[] m_Bodies;
        private Canvas m_Canvas;
        private BodyInfo[] m_BodyInfos;
        private List<Color> m_BodyColors = new List<Color>
            {
                Colors.Cyan,
                Colors.Orange,
                Colors.Yellow,
                Colors.Blue,
                Colors.Indigo,
                Colors.Violet
            };

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

        private IGestureTranslator m_Translator;
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

        private SolidColorBrush m_AccentColor;
        public SolidColorBrush AccentColor
        {
            get { return m_AccentColor; }
            set
            {
                if (value != m_AccentColor)
                {
                    m_AccentColor = value;
                    RaisePropertyChanged(nameof(AccentColor));
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
            InitializeKinect();
            InitializeDrone();

            DataContext = this;
            InitializeComponent();
            
            AccentColor = new SolidColorBrush(Colors.WhiteSmoke);

            m_Canvas.Clip = new RectangleGeometry();// set the clip rectangle to prevent rendering outside the canvas
            m_Canvas.Clip.Rect = new Rect(0.0, 0.0, DisplayGrid.Width, DisplayGrid.Height);
            PopulateVisualObjects();
            DisplayGrid.Children.Add(m_Canvas);
            ResetFactors();
            //VideoGrid.Children.Add(DroneVideoElem);
            //DroneVideoElem.Visibility = Visibility.Collapsed;
        }

        private void PopulateVisualObjects()
        {
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
            //if (m_Drone != null)
            //{
            //    m_Drone.Dispose();
            //    m_Drone.Close();
            //}
        }
        #endregion

        #region Kinect
        private void InitializeKinect()
        {
            m_KinectSensor = KinectSensor.GetDefault();
            m_CoordinateMapper = m_KinectSensor.CoordinateMapper;
            var frameDescription = m_KinectSensor.DepthFrameSource.FrameDescription;// get the depth (display) extents
            // get size of joint space
            m_JointSpaceWidth = frameDescription.Width;
            m_JointSpaceHeight = frameDescription.Height;
            m_Bodies = new Body[m_KinectSensor.BodyFrameSource.BodyCount];// get total number of bodies from BodyFrameSource
            m_BodyFrameReader = m_KinectSensor.BodyFrameSource.OpenReader();
            m_BodyFrameReader.FrameArrived += Reader_BodyFrameArrived;
            m_KinectSensor.IsAvailableChanged += KinectSensor_IsAvailableChanged;
            m_Canvas = new Canvas();
            m_KinectSensor.Open();
            m_BodyCount = m_KinectSensor.BodyFrameSource.BodyCount;

            StatusText = m_KinectSensor.IsAvailable ? "Kinect Sensor is running!" : "No ready Kinect Sensor found!";
        }

        private async void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
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
                        var jointPointsInDepthSpace = UpdateBody(body, m_BodyInfos[bodyIndex]);
                        
                        if (IsConnected && !ControlLocked && !alreadyControlled) // convert the gestures of the body and send them as a command to the drone
                        {
                            var state = m_Translator.Translate(jointPointsInDepthSpace, IsFlying, body.HandLeftState, body.HandRightState, m_CoordinateMapper);
                            await SendGesturesToDrone(state);

                            alreadyControlled = true; // Allow only the first tracked body to control
                            AccentColor = new SolidColorBrush(m_BodyColors[bodyIndex]);
                            // DEBUG
                            //m_GazBlock.Text = "Gaz: " + state.Gaz;
                            //m_PitchBlock.Text = "Pitch: " + state.Pitch;
                            //m_YawBlock.Text = "Yaw: " + state.Yaw;
                            //m_RollBlock.Text = "Roll: " + state.Roll;
                        }
                    }
                    else
                        m_BodyInfos[bodyIndex].Clear();
                }
            }
        }

        private Dictionary<JointType, CameraSpacePoint> UpdateBody(Body body, BodyInfo bodyInfo)
        {
            var joints = body.Joints;
            var jointPointsInDepthSpace = new Dictionary<JointType, Point>();
            var jointPositions = new Dictionary<JointType, CameraSpacePoint>();
            var coordinateMapper = m_KinectSensor.CoordinateMapper;
            foreach (var jointType in body.Joints.Keys)
            {
                // sometimes the depth(Z) of an inferred joint may show as negative
                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                var position = body.Joints[jointType].Position;
                if (position.Z < 0)
                    position.Z = c_InferredZPositionClamp;
                jointPositions[jointType] = position;
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

            return jointPositions;
        }

        private void UpdateJoint(Ellipse ellipse, Joint joint, Point point)
        {
            var trackingState = joint.TrackingState;
            if (trackingState != TrackingState.NotTracked)// only draw if joint is tracked or inferred
            {
                if (trackingState == TrackingState.Tracked)
                    ellipse.Fill = new SolidColorBrush(Colors.Green);
                else // inferred joints are red
                    ellipse.Fill = new SolidColorBrush(Colors.Red);
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
        
        private void KinectSensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            StatusText = m_KinectSensor.IsAvailable ? "Kinect Sensor is running!" : "Kinect Sensor not available!";
        }
        #endregion

        #region Drone
        private DroneClient m_Drone;
        private DispatcherTimer m_BatteryTimer;
        //private readonly string videoUrl = "ardrone://192.168.1.1";
        //MediaElement DroneVideoElem;
        private void InitializeDrone()
        {
            m_Drone = DroneClient.Instance;
            //m_Drone.SetIndoorConfiguration();

            m_BatteryTimer = new DispatcherTimer();
            m_BatteryTimer.Tick += BatteryTimer_Tick;
            m_BatteryTimer.Interval = new TimeSpan(0, 0, 10); // 10 seconds
            //DroneVideoElem = new MediaElement();
            //DroneVideoElem.MediaFailed += DroneVideoElem_MediaFailed;
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
            if (gestureCommand.Mode == InputStateMode.Land)
            {
                m_Drone.InputState.Update(0, 0, 0, 0);
                m_Drone.Land();
                ControlLocked = true;
            }
            else if (gestureCommand.Mode == InputStateMode.Takeoff)
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
                m_Drone.InputState.Update(gestureCommand.Roll, gestureCommand.Pitch, gestureCommand.Yaw, gestureCommand.Gaz);
            }
            IsFlying = gestureCommand.Flying;
        }
        
        private void HandStates_Checked(object sender, RoutedEventArgs e)
        {
            m_Translator = new HandStateTranslator();
        }

        private void Pose_Checked(object sender, RoutedEventArgs e)
        {
            m_Translator = new PoseTranslator();
        }

        private void PitchSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            Constants.Pitch = (float)e.NewValue / 10;
        }

        private void RollSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            Constants.Roll = (float)e.NewValue / 10;
        }

        private void YawSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            Constants.Yaw = (float)e.NewValue / 10;
        }

        private void GazSlider_ValueChanged(object sender, RangeBaseValueChangedEventArgs e)
        {
            Constants.Gaz = (float)e.NewValue / 10;
        }

        private void ResetFactors()
        {
            Constants.Reset();
            ResetControlSliders();
        }

        private void ResetControlSliders()
        {
            m_PitchSlider.Value = Constants.Pitch * 10;
            m_RollSlider.Value = Constants.Roll * 10;
            m_YawSlider.Value = Constants.Yaw * 10;
            m_GazSlider.Value = Constants.Gaz * 10;
        }

        private async void ConnectDisconnect()
        {
            if (!IsConnected)
            {
                await ConnectDrone();
            }
            else
            {
                DisconnectDrone();
            }
            //ResetFactors();
        }

        private async Task ConnectDrone()
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

        private void DisconnectDrone()
        {
            m_Drone.Dispose();
            m_Drone.Close();
            IsConnected = false;
        }

        private async void TakeoffLand()
        {
            if (!IsFlying)
            {
                await Takeoff();
            }
            else
            {
                Land();
            }
            //ResetFactors();
        }

        private async Task Takeoff()
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

        private void Land()
        {
            m_Drone.InputState.Update(0, 0, 0, 0);
            ControlLocked = true;
            m_Drone.Land();
            IsFlying = false;
        }

        private void LockControl()
        {
            ControlLocked = !ControlLocked;
            if (IsFlying)
                m_Drone.InputState.Update(0, 0, 0, 0);
            //ResetFactors();
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

        //private void UpdateUIElements()
        //{
        //    if (!IsConnected)
        //    {
        //        //KinectViewBox.Visibility = Visibility.Visible;
        //        //DroneVideoElem.Visibility = Visibility.Collapsed;
        //        //ChangeViewButton.IsEnabled = false;
        //        IsFlying = false;
        //        ControlLocked = false;
        //    }
        //}
        #endregion
    }
}
